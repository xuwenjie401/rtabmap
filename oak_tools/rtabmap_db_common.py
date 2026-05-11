#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sqlite3
import struct
import zlib

import cv2
import numpy as np


DEFAULT_OUTPUT_DIR = Path("/home/wjxu22/Datasets/outputs/rtab")
DEFAULT_DB_BASENAME = "pointcloud.db"
DEPTH_RVL_MAGIC = b"DEPTHRVL"


@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    local_transform: np.ndarray


def resolve_db_path(db_path: Path | None, search_dir: Path = DEFAULT_OUTPUT_DIR) -> Path:
    if db_path is not None:
        resolved = db_path.expanduser().resolve()
        if not resolved.is_file():
            raise FileNotFoundError(f"Database not found: {resolved}")
        return resolved

    search_dir = search_dir.expanduser().resolve()
    if not search_dir.is_dir():
        raise FileNotFoundError(f"Search directory not found: {search_dir}")

    preferred = search_dir / DEFAULT_DB_BASENAME
    if preferred.is_file():
        return preferred

    candidates = sorted(search_dir.glob("*.db"))
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise FileNotFoundError(f"No .db file found under {search_dir}")
    raise FileNotFoundError(
        f"Multiple .db files found under {search_dir}, please pass --db-path explicitly."
    )


def cv_type_to_dtype_channels(cv_type: int) -> tuple[np.dtype, int]:
    depth = cv_type & 7
    channels = 1 + (cv_type >> 3)
    depth_to_dtype = {
        0: np.uint8,
        1: np.int8,
        2: np.uint16,
        3: np.int16,
        4: np.int32,
        5: np.float32,
        6: np.float64,
    }
    if depth not in depth_to_dtype:
        raise ValueError(f"Unsupported OpenCV depth code: {depth}")
    return np.dtype(depth_to_dtype[depth]), channels


def uncompress_mat(blob: bytes) -> np.ndarray:
    if len(blob) < 12:
        raise ValueError("Compressed matrix blob is too small")

    rows, cols, cv_type = struct.unpack_from("<iii", blob, len(blob) - 12)
    raw = zlib.decompress(blob[:-12])
    dtype, channels = cv_type_to_dtype_channels(cv_type)
    array = np.frombuffer(raw, dtype=dtype)

    if channels == 1:
        return array.reshape(rows, cols).copy()
    return array.reshape(rows, cols, channels).copy()


class RvlDecoder:
    def __init__(self, data: bytes) -> None:
        self._data = data
        self._offset = 0
        self._word = 0
        self._nibbles_written = 0

    def decode_vle(self) -> int:
        value = 0
        bits = 29
        while True:
            if self._nibbles_written == 0:
                self._word = struct.unpack_from("<I", self._data, self._offset)[0]
                self._offset += 4
                self._nibbles_written = 8

            nibble = self._word & 0xF0000000
            value |= ((nibble << 1) & 0xFFFFFFFF) >> bits
            self._word = (self._word << 4) & 0xFFFFFFFF
            self._nibbles_written -= 1
            bits -= 3
            if (nibble & 0x80000000) == 0:
                return value


def decode_rvl_depth(blob: bytes) -> np.ndarray:
    if not blob.startswith(DEPTH_RVL_MAGIC):
        raise ValueError("Depth blob is not DEPTHRVL")

    cols = struct.unpack_from("<I", blob, 8)[0]
    rows = struct.unpack_from("<I", blob, 12)[0]
    decoder = RvlDecoder(blob[16:])
    output = np.empty(rows * cols, dtype=np.uint16)

    previous = 0
    out_index = 0
    remaining = rows * cols
    while remaining > 0:
        zeros = decoder.decode_vle()
        if zeros:
            output[out_index : out_index + zeros] = 0
            out_index += zeros
            remaining -= zeros

        nonzeros = decoder.decode_vle()
        remaining -= nonzeros
        for _ in range(nonzeros):
            positive = decoder.decode_vle()
            delta = (positive >> 1) ^ -(positive & 1)
            current = previous + delta
            output[out_index] = current
            previous = current
            out_index += 1

    return output.reshape(rows, cols)


def decode_image(blob: bytes) -> np.ndarray:
    image = cv2.imdecode(np.frombuffer(blob, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError("Failed to decode image blob")
    if image.ndim == 2:
        image = np.repeat(image[:, :, None], 3, axis=2)
    elif image.shape[2] == 4:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2RGB)
    else:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image


def decode_depth(blob: bytes) -> np.ndarray:
    if blob.startswith(DEPTH_RVL_MAGIC):
        return decode_rvl_depth(blob)

    depth = cv2.imdecode(np.frombuffer(blob, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if depth is None:
        raise ValueError("Failed to decode depth blob")
    if depth.ndim == 3 and depth.shape[2] == 4:
        depth = depth.view(np.float32).reshape(depth.shape[:2])
    return depth


def transform_from_12(values: np.ndarray) -> np.ndarray:
    if values.shape[0] != 12:
        raise ValueError("Transform must contain 12 floats")
    matrix = np.eye(4, dtype=np.float32)
    matrix[:3, :] = values.reshape(3, 4)
    return matrix


def parse_pose_blob(blob: bytes | None) -> np.ndarray | None:
    if blob is None or len(blob) != 48:
        return None
    values = np.frombuffer(blob, dtype="<f4", count=12).astype(np.float32)
    return transform_from_12(values)


def parse_camera_models(blob: bytes) -> list[CameraModel]:
    models: list[CameraModel] = []
    offset = 0
    total_size = len(blob)

    while offset < total_size:
        if total_size - offset < 44:
            raise ValueError("Calibration blob is truncated")

        header = struct.unpack_from("<11i", blob, offset)
        model_type = header[3]
        if model_type != 0:
            raise NotImplementedError(
                "This helper currently supports RGB-D databases with mono CameraModel calibration only."
            )

        width = header[4]
        height = header[5]
        k_len, d_len, r_len, p_len, l_len = header[6:11]
        chunk_size = 44 + 8 * (k_len + d_len + r_len + p_len) + 4 * l_len
        if offset + chunk_size > total_size:
            raise ValueError("Calibration blob size does not match header")

        cursor = offset + 44
        K = None
        P = None
        if k_len:
            K = np.frombuffer(blob, dtype="<f8", count=k_len, offset=cursor).copy().reshape(3, 3)
            cursor += 8 * k_len
        if d_len:
            cursor += 8 * d_len
        if r_len:
            cursor += 8 * r_len
        if p_len:
            P = np.frombuffer(blob, dtype="<f8", count=p_len, offset=cursor).copy().reshape(3, 4)
            cursor += 8 * p_len

        local_transform = np.eye(4, dtype=np.float32)
        if l_len:
            local_transform = transform_from_12(
                np.frombuffer(blob, dtype="<f4", count=l_len, offset=cursor).astype(np.float32)
            )
            cursor += 4 * l_len

        fx = float(P[0, 0] if P is not None else K[0, 0])
        fy = float(P[1, 1] if P is not None else K[1, 1])
        cx = float(P[0, 2] if P is not None else K[0, 2])
        cy = float(P[1, 2] if P is not None else K[1, 2])

        models.append(
            CameraModel(
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                width=width,
                height=height,
                local_transform=local_transform,
            )
        )
        offset += chunk_size

    return models


def load_optimized_poses(conn: sqlite3.Connection) -> dict[int, np.ndarray]:
    row = conn.execute("SELECT opt_ids, opt_poses FROM Admin LIMIT 1").fetchone()
    if row is None or row[0] is None or row[1] is None:
        return {}

    ids = uncompress_mat(row[0]).reshape(-1)
    poses = uncompress_mat(row[1]).reshape(-1, 12)
    if ids.shape[0] != poses.shape[0]:
        raise ValueError("Optimized pose blob size mismatch")

    return {
        int(node_id): transform_from_12(pose.astype(np.float32))
        for node_id, pose in zip(ids.tolist(), poses)
    }


def apply_transform(points: np.ndarray, transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    return points @ rotation.T + translation


def build_cloud(
    rgb: np.ndarray,
    depth: np.ndarray,
    model: CameraModel,
    image_decimation: int,
    min_depth: float,
    max_depth: float,
) -> tuple[np.ndarray, np.ndarray]:
    if depth.dtype == np.uint16:
        depth_m = depth.astype(np.float32) * 0.001
    elif depth.dtype == np.float32:
        depth_m = depth
    else:
        raise ValueError(f"Unsupported depth dtype: {depth.dtype}")

    if rgb.shape[:2] != depth.shape[:2]:
        rgb = cv2.resize(rgb, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_LINEAR)

    v_coords = np.arange(0, depth.shape[0], image_decimation, dtype=np.float32)
    u_coords = np.arange(0, depth.shape[1], image_decimation, dtype=np.float32)
    uu, vv = np.meshgrid(u_coords, v_coords)

    sampled_depth = depth_m[::image_decimation, ::image_decimation]
    sampled_rgb = rgb[::image_decimation, ::image_decimation]

    valid = np.isfinite(sampled_depth) & (sampled_depth > min_depth)
    if max_depth > 0:
        valid &= sampled_depth < max_depth

    if not np.any(valid):
        return np.empty((0, 3), dtype=np.float32), np.empty((0, 3), dtype=np.uint8)

    z = sampled_depth[valid]
    x = (uu[valid] - model.cx) * z / model.fx
    y = (vv[valid] - model.cy) * z / model.fy
    points = np.stack((x, y, z), axis=1).astype(np.float32)
    colors = sampled_rgb[valid].reshape(-1, 3).astype(np.uint8)
    return points, colors


def voxel_downsample(points: np.ndarray, colors: np.ndarray, voxel_size: float) -> tuple[np.ndarray, np.ndarray]:
    if voxel_size <= 0 or points.shape[0] == 0:
        return points, colors

    voxel_indices = np.floor(points / voxel_size).astype(np.int32)
    _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
    unique_indices.sort()
    return points[unique_indices], colors[unique_indices]


def limit_points(points: np.ndarray, colors: np.ndarray, max_points: int) -> tuple[np.ndarray, np.ndarray]:
    if max_points <= 0 or points.shape[0] <= max_points:
        return points, colors

    stride = int(np.ceil(points.shape[0] / max_points))
    return points[::stride], colors[::stride]
