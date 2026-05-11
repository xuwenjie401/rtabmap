#!/usr/bin/env python3
#
# Usage:
#   Default behavior does not downsample the reconstructed map, and also logs
#   time-varying camera poses plus RGB images for playback in Rerun.
#   conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rtabmap_db_rerun_viewer.py \
#     --db-path /home/wjxu22/Datasets/outputs/rtab/pointcloud.db \
#     --spawn
#
#   Add downsampling only when needed.
#   conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rtabmap_db_rerun_viewer.py \
#     --db-path /home/wjxu22/Datasets/outputs/rtab/pointcloud.db \
#     --spawn \
#     --image-decimation 2 \
#     --voxel-size 0.03 \
#     --max-points 800000

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sqlite3
import struct
import sys
import zlib

import cv2
import numpy as np
import rerun as rr


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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read an RTAB-Map RGB-D database and visualize the global RGB pointcloud in Rerun."
    )
    parser.add_argument(
        "--db-path",
        type=Path,
        default=None,
        help="RTAB-Map .db path. If omitted, the script searches in the default output directory.",
    )
    parser.add_argument(
        "--search-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory searched when --db-path is omitted.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_DIR,
        help="Directory used for default .rrd output.",
    )
    parser.add_argument(
        "--image-decimation",
        type=int,
        default=1,
        help="Pixel stride used when projecting depth to 3D. Default 1 means no image decimation.",
    )
    parser.add_argument(
        "--min-depth",
        type=float,
        default=0.2,
        help="Minimum valid depth in meters.",
    )
    parser.add_argument(
        "--max-depth",
        type=float,
        default=5.0,
        help="Maximum valid depth in meters. Use 0 to disable.",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.0,
        help="Voxel size in meters for final downsampling. Default 0 disables voxel downsampling.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=0,
        help="Maximum number of final map points after downsampling. Default 0 disables this limit.",
    )
    parser.add_argument(
        "--max-nodes",
        type=int,
        default=0,
        help="Maximum number of database nodes to process. Use 0 to process all.",
    )
    parser.add_argument(
        "--node-step",
        type=int,
        default=1,
        help="Only process every Nth node.",
    )
    parser.add_argument(
        "--spawn",
        action="store_true",
        help="Spawn a local Rerun viewer window.",
    )
    parser.add_argument(
        "--save-rrd",
        type=Path,
        default=None,
        help="Save the recording to an .rrd file instead of spawning a viewer.",
    )
    args = parser.parse_args()

    if args.image_decimation < 1:
        parser.error("--image-decimation must be >= 1")
    if args.node_step < 1:
        parser.error("--node-step must be >= 1")
    if args.spawn and args.save_rrd is not None:
        parser.error("--spawn and --save-rrd are mutually exclusive")

    return args


def resolve_db_path(db_path: Path | None, search_dir: Path) -> Path:
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
                "This viewer currently supports RGB-D databases with mono CameraModel calibration only."
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
        rgb = cv2.resize(
            rgb,
            (depth.shape[1], depth.shape[0]),
            interpolation=cv2.INTER_LINEAR,
        )

    v_coords = np.arange(0, depth.shape[0], image_decimation, dtype=np.float32)
    u_coords = np.arange(0, depth.shape[1], image_decimation, dtype=np.float32)
    uu, vv = np.meshgrid(u_coords, v_coords)

    sampled_depth = depth_m[::image_decimation, ::image_decimation]
    sampled_rgb = rgb[::image_decimation, ::image_decimation]

    valid = np.isfinite(sampled_depth) & (sampled_depth > min_depth)
    if max_depth > 0:
        valid &= sampled_depth < max_depth

    if not np.any(valid):
        return (
            np.empty((0, 3), dtype=np.float32),
            np.empty((0, 3), dtype=np.uint8),
        )

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


def configure_rerun(db_path: Path, args: argparse.Namespace) -> Path | None:
    rr.init("rtabmap_db_rerun_viewer", spawn=args.spawn)
    if args.spawn:
        return None

    save_path = args.save_rrd
    if save_path is None:
        save_path = args.output_dir / f"{db_path.stem}.rrd"
    save_path = save_path.expanduser().resolve()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    rr.save(save_path)
    return save_path


def log_camera_frame(
    stamp: float,
    frame_index: int,
    camera_pose: np.ndarray,
    model: CameraModel,
    rgb: np.ndarray,
) -> None:
    rr.set_time_seconds("stamp", stamp)
    rr.set_time_sequence("frame", frame_index)

    rr.log(
        "world/camera",
        rr.Transform3D(
            translation=camera_pose[:3, 3],
            mat3x3=camera_pose[:3, :3],
            axis_length=0.1,
        ),
    )
    rr.log(
        "world/camera",
        rr.Pinhole(
            resolution=[rgb.shape[1], rgb.shape[0]],
            focal_length=[model.fx, model.fy],
            principal_point=[model.cx, model.cy],
            camera_xyz=rr.ViewCoordinates.RDF,
        ),
    )
    rr.log("world/camera/rgb", rr.Image(rgb))


def main() -> int:
    args = parse_args()
    args.output_dir = args.output_dir.expanduser().resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    db_path = resolve_db_path(args.db_path, args.search_dir)
    rrd_path = configure_rerun(db_path, args)

    print(f"Reading database: {db_path}")
    if rrd_path is not None:
        print(f"Saving Rerun recording to: {rrd_path}")

    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    conn.row_factory = sqlite3.Row

    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    rr.log("world/camera", rr.ViewCoordinates.RDF, static=True)

    optimized_poses = load_optimized_poses(conn)
    if optimized_poses:
        print(f"Loaded {len(optimized_poses)} optimized poses from Admin.opt_poses")
    else:
        print("No saved optimized poses found, falling back to Node.pose")

    rows = conn.execute(
        """
        SELECT
            Node.id AS id,
            Node.stamp AS stamp,
            Node.pose AS pose,
            Data.image AS image,
            Data.depth AS depth,
            Data.calibration AS calibration
        FROM Node
        JOIN Data ON Node.id = Data.id
        ORDER BY Node.stamp ASC
        """
    ).fetchall()

    if not rows:
        print("Database does not contain any Node/Data rows.", file=sys.stderr)
        return 1

    point_chunks: list[np.ndarray] = []
    color_chunks: list[np.ndarray] = []
    camera_positions: list[np.ndarray] = []
    processed = 0
    skipped = 0

    for row_index, row in enumerate(rows):
        if args.max_nodes > 0 and processed >= args.max_nodes:
            break
        if row_index % args.node_step != 0:
            continue

        node_id = int(row["id"])
        pose = optimized_poses.get(node_id)
        if pose is None:
            pose = parse_pose_blob(row["pose"])
        elif pose is not None and optimized_poses:
            pass

        if optimized_poses and node_id not in optimized_poses:
            skipped += 1
            continue
        if pose is None:
            skipped += 1
            continue
        if row["image"] is None or row["depth"] is None or row["calibration"] is None:
            skipped += 1
            continue

        try:
            camera_models = parse_camera_models(row["calibration"])
            if not camera_models:
                skipped += 1
                continue
            model = camera_models[0]
            rgb = decode_image(row["image"])
            depth = decode_depth(row["depth"])
            local_points, local_colors = build_cloud(
                rgb,
                depth,
                model,
                args.image_decimation,
                args.min_depth,
                args.max_depth,
            )
        except NotImplementedError as exc:
            print(f"Node {node_id}: {exc}", file=sys.stderr)
            return 2
        except Exception as exc:  # noqa: BLE001
            print(f"Node {node_id}: failed to decode sensor data: {exc}", file=sys.stderr)
            skipped += 1
            continue

        if local_points.shape[0] == 0:
            skipped += 1
            continue

        camera_points = apply_transform(local_points, model.local_transform)
        world_points = apply_transform(camera_points, pose)
        point_chunks.append(world_points.astype(np.float32))
        color_chunks.append(local_colors.astype(np.uint8))

        camera_pose = pose @ model.local_transform
        camera_positions.append(camera_pose[:3, 3].astype(np.float32))
        log_camera_frame(
            stamp=float(row["stamp"]),
            frame_index=processed,
            camera_pose=camera_pose,
            model=model,
            rgb=rgb,
        )

        processed += 1
        print(
            f"[{processed}] node={node_id} stamp={float(row['stamp']):.6f} points={world_points.shape[0]}"
        )

    if not point_chunks:
        print("No valid RGB-D nodes were converted into pointclouds.", file=sys.stderr)
        return 1

    points = np.concatenate(point_chunks, axis=0)
    colors = np.concatenate(color_chunks, axis=0)
    raw_point_count = points.shape[0]

    if args.voxel_size > 0:
        points, colors = voxel_downsample(points, colors, args.voxel_size)
    if args.max_points > 0:
        points, colors = limit_points(points, colors, args.max_points)

    trajectory = (
        np.stack(camera_positions, axis=0).astype(np.float32)
        if camera_positions
        else np.empty((0, 3), dtype=np.float32)
    )

    rr.reset_time()
    rr.log("world/map", rr.Points3D(points, colors=colors), static=True)
    if trajectory.shape[0] > 0:
        rr.log(
            "world/cameras",
            rr.Points3D(
                trajectory,
                colors=np.tile(np.array([[255, 170, 0]], dtype=np.uint8), (trajectory.shape[0], 1)),
                radii=np.full((trajectory.shape[0],), 0.03, dtype=np.float32),
            ),
            static=True,
        )
    if trajectory.shape[0] > 1:
        rr.log(
            "world/trajectory",
            rr.LineStrips3D(
                [trajectory],
                colors=np.array([[255, 170, 0]], dtype=np.uint8),
            ),
            static=True,
        )

    print(f"Processed nodes: {processed}")
    print(f"Skipped nodes: {skipped}")
    print(f"Raw merged points: {raw_point_count}")
    print(f"Final logged points: {points.shape[0]}")
    if rrd_path is not None:
        print(f"Rerun recording saved to: {rrd_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
