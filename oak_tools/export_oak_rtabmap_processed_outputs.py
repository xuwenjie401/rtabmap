#!/usr/bin/env python3
from __future__ import annotations

import argparse
from bisect import bisect_left
import csv
from dataclasses import dataclass
import json
from pathlib import Path
import shutil
import sqlite3
import struct
import subprocess
import sys
from typing import Any

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from rtabmap_msgs.msg import MapData
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_msgs.msg import TFMessage

from extracted_dataset_common import (
    invert_transform,
    matrix_from_translation_quaternion,
    ns_to_seconds,
    quaternion_from_matrix,
    stamp_to_ns,
    write_json,
)


DEFAULT_RAW_BAG = Path("/home/wjxu22/Datasets/rosbags/oak_capture_stereo_imu_rgb_20260430_024609")
DEFAULT_PROCESSED_BAG = Path(
    "/home/wjxu22/Datasets/rosbags/processed/oak_rtabmap_processed_20260430_032909"
)
DEFAULT_OUTPUT_DIR = Path("/home/wjxu22/Datasets/outputs/rtab/oak_stereo_imu_processed_export")


@dataclass
class TimedTransform:
    stamp_ns: int
    matrix: np.ndarray
    reference_frame: str
    target_frame: str


@dataclass
class RgbStamp:
    index: int
    stamp_ns: int
    frame_id: str
    bag_stamp_ns: int
    image_path: str | None = None


@dataclass
class TopicSummary:
    processed_messages: int = 0
    rgb_stamps: int = 0
    odom_samples: int = 0
    correction_samples: int = 0
    cloud_topics: dict[str, int] | None = None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Export RTAB-Map processed bag products: latest map cloud plus "
            "per-RGB timestamp poses in map frame."
        )
    )
    parser.add_argument("--raw-bag", type=Path, default=DEFAULT_RAW_BAG)
    parser.add_argument("--processed-bag", type=Path, default=DEFAULT_PROCESSED_BAG)
    parser.add_argument(
        "--cloud-bag",
        type=Path,
        default=None,
        help=(
            "Optional bag used only for PointCloud2 topics. Pose, graph and TF "
            "are still read from --processed-bag. This is useful when a high-res "
            "map_assembler bag contains cloud topics but not /rtabmap/mapData."
        ),
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--rgb-topic", default="/oak/rgb/image_raw")
    parser.add_argument("--rgb-camera-info-topic", default="/oak/rgb/camera_info")
    parser.add_argument("--odom-topic", default="/rtabmap/odom")
    parser.add_argument("--map-data-topic", default="/rtabmap/mapData")
    parser.add_argument("--tf-static-topic", default="/tf_static")
    parser.add_argument("--base-frame", default="oak")
    parser.add_argument("--rgb-frame", default="oak_rgb_camera_optical_frame")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--cloud-topic", default="/rtabmap/cloud_map")
    parser.add_argument(
        "--extra-cloud-topic",
        action="append",
        default=["/rtabmap/cloud_obstacles", "/rtabmap/cloud_ground"],
        help="Additional PointCloud2 topic to export. May be repeated.",
    )
    parser.add_argument("--max-odom-gap-sec", type=float, default=0.5)
    parser.add_argument("--max-correction-gap-sec", type=float, default=5.0)
    parser.add_argument("--max-correction-extrapolation-sec", type=float, default=2.0)
    parser.add_argument(
        "--export-rgb-images",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Export RGB images from the raw bag and link them from the pose table.",
    )
    parser.add_argument(
        "--image-format",
        choices=("jpg", "png"),
        default="jpg",
        help="Saved RGB image format. Use png for lossless export; jpg is smaller.",
    )
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--png-compression", type=int, default=3)
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if not (1 <= args.jpeg_quality <= 100):
        parser.error("--jpeg-quality must be in [1, 100]")
    if not (0 <= args.png_compression <= 9):
        parser.error("--png-compression must be in [0, 9]")
    return args


def prepare_output_dir(path: Path, overwrite: bool) -> None:
    path = path.expanduser().resolve()
    if path.exists():
        if not overwrite:
            raise FileExistsError(f"Output directory already exists: {path}")
        shutil.rmtree(path)
    (path / "pointcloud").mkdir(parents=True, exist_ok=True)
    (path / "poses").mkdir(parents=True, exist_ok=True)
    (path / "images").mkdir(parents=True, exist_ok=True)
    (path / "camera").mkdir(parents=True, exist_ok=True)
    (path / ".cache").mkdir(parents=True, exist_ok=True)


def resolve_bag_sqlite_files(bag_path: Path) -> list[Path]:
    bag_path = bag_path.expanduser().resolve()
    if bag_path.is_file():
        if bag_path.suffix == ".db3":
            return [bag_path]
        if bag_path.name.endswith(".db3.zstd"):
            return [bag_path]
        if bag_path.name == "metadata.yaml":
            bag_path = bag_path.parent
        else:
            raise FileNotFoundError(f"Unsupported bag file: {bag_path}")
    if not bag_path.is_dir():
        raise FileNotFoundError(f"Bag path not found: {bag_path}")
    db3_files = sorted(bag_path.glob("*.db3"))
    zstd_files = sorted(bag_path.glob("*.db3.zstd"))
    if db3_files:
        return db3_files
    if zstd_files:
        return zstd_files
    raise FileNotFoundError(f"No .db3 or .db3.zstd files found under {bag_path}")


def decompress_zstd_if_needed(db_file: Path, cache_dir: Path) -> Path:
    if not db_file.name.endswith(".zstd"):
        return db_file

    zstd = shutil.which("zstd")
    if zstd is None:
        raise FileNotFoundError("zstd executable not found in PATH, cannot read compressed bag.")

    output = cache_dir / db_file.name.removesuffix(".zstd")
    if output.exists() and output.stat().st_mtime >= db_file.stat().st_mtime:
        return output

    print(f"Decompressing processed bag db: {db_file} -> {output}")
    subprocess.run([zstd, "-d", "-f", str(db_file), "-o", str(output)], check=True)
    return output


def make_reader(sqlite_uri: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(sqlite_uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def set_topic_filter(reader: rosbag2_py.SequentialReader, topics: list[str]) -> None:
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))


def align(offset: int, alignment: int) -> int:
    return (offset + alignment - 1) & ~(alignment - 1)


def parse_image_header_from_cdr_prefix(prefix: bytes) -> tuple[int, str]:
    if len(prefix) < 20:
        raise ValueError("Serialized image prefix too small.")

    little_endian = prefix[:2] == b"\x00\x01"
    fmt = "<" if little_endian else ">"
    offset = 4

    offset = align(offset, 4)
    sec = struct.unpack_from(fmt + "i", prefix, offset)[0]
    offset += 4
    offset = align(offset, 4)
    nanosec = struct.unpack_from(fmt + "I", prefix, offset)[0]
    offset += 4
    offset = align(offset, 4)
    frame_len = struct.unpack_from(fmt + "I", prefix, offset)[0]
    offset += 4
    if offset + frame_len > len(prefix):
        raise ValueError("Serialized image prefix does not contain full frame_id.")
    frame_bytes = prefix[offset : offset + frame_len]
    if frame_bytes.endswith(b"\x00"):
        frame_bytes = frame_bytes[:-1]
    frame_id = frame_bytes.decode("utf-8")
    return sec * 1_000_000_000 + nanosec, frame_id


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if encoding in ("bgr8", "rgb8", "8uc3"):
        channels = 3
        rows = data.reshape(height, step)[:, : width * channels]
        image = rows.reshape(height, width, channels)
        if encoding == "rgb8":
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        return image.copy()

    if encoding in ("bgra8", "rgba8", "8uc4"):
        channels = 4
        rows = data.reshape(height, step)[:, : width * channels]
        image = rows.reshape(height, width, channels)
        if encoding == "rgba8":
            return cv2.cvtColor(image, cv2.COLOR_RGBA2BGRA)
        return image.copy()

    if encoding in ("mono8", "8uc1"):
        rows = data.reshape(height, step)[:, :width]
        return rows.reshape(height, width).copy()

    raise ValueError(f"Unsupported RGB image encoding: {msg.encoding}")


def save_image_msg(
    msg: Image,
    output_path: Path,
    image_format: str,
    jpeg_quality: int,
    png_compression: int,
) -> None:
    bgr = image_msg_to_bgr(msg)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if image_format == "jpg":
        ok = cv2.imwrite(str(output_path), bgr, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    else:
        ok = cv2.imwrite(str(output_path), bgr, [cv2.IMWRITE_PNG_COMPRESSION, png_compression])
    if not ok:
        raise ValueError(f"Failed to save image: {output_path}")


def camera_info_to_dict(msg: CameraInfo, topic: str) -> dict[str, Any]:
    return {
        "topic": topic,
        "stamp_ns": stamp_to_ns(msg.header.stamp),
        "frame_id": msg.header.frame_id,
        "width": int(msg.width),
        "height": int(msg.height),
        "distortion_model": msg.distortion_model,
        "d": [float(v) for v in msg.d],
        "k": [float(v) for v in msg.k],
        "r": [float(v) for v in msg.r],
        "p": [float(v) for v in msg.p],
        "fx": float(msg.p[0] if msg.p[0] != 0.0 else msg.k[0]),
        "fy": float(msg.p[5] if msg.p[5] != 0.0 else msg.k[4]),
        "cx": float(msg.p[2] if msg.p[2] != 0.0 else msg.k[2]),
        "cy": float(msg.p[6] if msg.p[6] != 0.0 else msg.k[5]),
    }


def read_first_camera_info(raw_bag: Path, camera_info_topic: str) -> dict[str, Any] | None:
    sqlite_files = resolve_bag_sqlite_files(raw_bag)
    for db_file in sqlite_files:
        if db_file.name.endswith(".zstd"):
            continue
        conn = sqlite3.connect(f"file:{db_file}?mode=ro", uri=True)
        topic_row = conn.execute("SELECT id FROM topics WHERE name=?", (camera_info_topic,)).fetchone()
        if topic_row is None:
            conn.close()
            continue
        row = conn.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp ASC LIMIT 1",
            (int(topic_row[0]),),
        ).fetchone()
        conn.close()
        if row is None:
            continue
        return camera_info_to_dict(deserialize_message(row[0], CameraInfo), camera_info_topic)
    return None


def read_rgb_stamps_and_images(
    raw_bag: Path,
    rgb_topic: str,
    output_dir: Path,
    export_images: bool,
    image_format: str,
    jpeg_quality: int,
    png_compression: int,
) -> list[RgbStamp]:
    rgb_stamps: list[RgbStamp] = []
    sqlite_files = resolve_bag_sqlite_files(raw_bag)
    index = 0
    extension = ".jpg" if image_format == "jpg" else ".png"

    for db_file in sqlite_files:
        if db_file.name.endswith(".zstd"):
            raise ValueError("Compressed raw bag reading is not implemented for RGB stamp extraction.")
        conn = sqlite3.connect(f"file:{db_file}?mode=ro", uri=True)
        topic_row = conn.execute("SELECT id FROM topics WHERE name=?", (rgb_topic,)).fetchone()
        if topic_row is None:
            conn.close()
            continue
        topic_id = int(topic_row[0])
        select_expr = "timestamp, data" if export_images else "timestamp, substr(data, 1, 256)"
        for bag_stamp_ns, payload in conn.execute(
            f"""
            SELECT {select_expr}
            FROM messages
            WHERE topic_id=?
            ORDER BY timestamp ASC
            """,
            (topic_id,),
        ):
            image_path = None
            if export_images:
                msg = deserialize_message(payload, Image)
                stamp_ns = stamp_to_ns(msg.header.stamp)
                frame_id = msg.header.frame_id
                image_rel_path = Path("images") / f"rgb_{index:06d}{extension}"
                save_image_msg(
                    msg,
                    output_dir / image_rel_path,
                    image_format,
                    jpeg_quality,
                    png_compression,
                )
                image_path = str(image_rel_path)
            else:
                stamp_ns, frame_id = parse_image_header_from_cdr_prefix(payload)
            rgb_stamps.append(
                RgbStamp(
                    index=index,
                    stamp_ns=stamp_ns,
                    frame_id=frame_id,
                    bag_stamp_ns=int(bag_stamp_ns),
                    image_path=image_path,
                )
            )
            index += 1
        conn.close()

    if not rgb_stamps:
        raise ValueError(f"No RGB image stamps found on topic {rgb_topic} in {raw_bag}")
    return rgb_stamps


def matrix_from_pose_msg(pose: Any) -> np.ndarray:
    return matrix_from_translation_quaternion(
        [pose.position.x, pose.position.y, pose.position.z],
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
    )


def matrix_from_transform_msg(transform: Any) -> np.ndarray:
    return matrix_from_translation_quaternion(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w],
    )


def slerp(q0: np.ndarray, q1: np.ndarray, alpha: float) -> np.ndarray:
    q0 = np.asarray(q0, dtype=np.float64)
    q1 = np.asarray(q1, dtype=np.float64)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(max(dot, -1.0), 1.0)
    if dot > 0.9995:
        q = q0 + alpha * (q1 - q0)
        return q / np.linalg.norm(q)
    theta_0 = np.arccos(dot)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * alpha
    s0 = np.sin(theta_0 - theta) / sin_theta_0
    s1 = np.sin(theta) / sin_theta_0
    return s0 * q0 + s1 * q1


def interpolate_matrix(a: np.ndarray, b: np.ndarray, alpha: float) -> np.ndarray:
    qa = quaternion_from_matrix(a[:3, :3])
    qb = quaternion_from_matrix(b[:3, :3])
    q = slerp(qa, qb, alpha)
    t = (1.0 - alpha) * a[:3, 3] + alpha * b[:3, 3]
    return matrix_from_translation_quaternion(t, q)


def interpolate_timed_transform(
    samples: list[TimedTransform],
    stamp_ns: int,
    max_gap_ns: int,
    allow_extrapolation_ns: int = 0,
) -> tuple[np.ndarray | None, int | None, str]:
    if not samples:
        return None, None, "missing"
    stamps = [sample.stamp_ns for sample in samples]
    index = bisect_left(stamps, stamp_ns)

    if index == 0:
        dt = stamp_ns - stamps[0]
        if dt < 0 and abs(dt) <= allow_extrapolation_ns:
            return samples[0].matrix, dt, "extrapolated_before"
        if dt == 0:
            return samples[0].matrix, 0, "exact"
        return None, dt, "before_range"
    if index == len(samples):
        dt = stamp_ns - stamps[-1]
        if dt >= 0 and dt <= allow_extrapolation_ns:
            return samples[-1].matrix, dt, "extrapolated_after"
        return None, dt, "after_range"

    before = samples[index - 1]
    after = samples[index]
    span = after.stamp_ns - before.stamp_ns
    if span <= 0:
        return before.matrix, 0, "duplicate"
    gap = max(stamp_ns - before.stamp_ns, after.stamp_ns - stamp_ns)
    if gap > max_gap_ns:
        return None, int(gap), "gap_too_large"
    alpha = (stamp_ns - before.stamp_ns) / span
    return interpolate_matrix(before.matrix, after.matrix, alpha), int(gap), "interpolated"


def pointcloud_to_arrays(msg: PointCloud2) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
    structured = point_cloud2.read_points(msg, skip_nans=False)
    xyz = np.stack([structured["x"], structured["y"], structured["z"]], axis=-1).astype(np.float32)
    valid = np.isfinite(xyz).all(axis=1)
    xyz = xyz[valid]

    rgb = None
    intensity = None
    if "rgb" in structured.dtype.names or "rgba" in structured.dtype.names:
        field_name = "rgb" if "rgb" in structured.dtype.names else "rgba"
        packed = structured[field_name][valid]
        if packed.dtype == np.float32:
            packed = packed.view(np.uint32)
        else:
            packed = packed.astype(np.uint32, copy=False)
        rgb = np.stack(
            [
                ((packed >> 16) & 0xFF).astype(np.uint8),
                ((packed >> 8) & 0xFF).astype(np.uint8),
                (packed & 0xFF).astype(np.uint8),
            ],
            axis=-1,
        )
    elif {"r", "g", "b"}.issubset(structured.dtype.names):
        rgb = np.stack(
            [
                structured["r"][valid].astype(np.uint8),
                structured["g"][valid].astype(np.uint8),
                structured["b"][valid].astype(np.uint8),
            ],
            axis=-1,
        )
    elif "intensity" in structured.dtype.names:
        intensity = structured["intensity"][valid].astype(np.float32)

    return xyz, rgb, intensity


def write_cloud(path: Path, xyz: np.ndarray, rgb: np.ndarray | None, intensity: np.ndarray | None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if rgb is not None:
        vertex = np.empty(
            xyz.shape[0],
            dtype=[
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("red", "u1"),
                ("green", "u1"),
                ("blue", "u1"),
            ],
        )
        vertex["x"] = xyz[:, 0]
        vertex["y"] = xyz[:, 1]
        vertex["z"] = xyz[:, 2]
        vertex["red"] = rgb[:, 0]
        vertex["green"] = rgb[:, 1]
        vertex["blue"] = rgb[:, 2]
        fields = [
            "property float x",
            "property float y",
            "property float z",
            "property uchar red",
            "property uchar green",
            "property uchar blue",
        ]
    elif intensity is not None:
        vertex = np.empty(
            xyz.shape[0],
            dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")],
        )
        vertex["x"] = xyz[:, 0]
        vertex["y"] = xyz[:, 1]
        vertex["z"] = xyz[:, 2]
        vertex["intensity"] = intensity
        fields = ["property float x", "property float y", "property float z", "property float intensity"]
    else:
        vertex = np.empty(xyz.shape[0], dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
        vertex["x"] = xyz[:, 0]
        vertex["y"] = xyz[:, 1]
        vertex["z"] = xyz[:, 2]
        fields = ["property float x", "property float y", "property float z"]

    header = "\n".join(
        ["ply", "format binary_little_endian 1.0", f"element vertex {xyz.shape[0]}", *fields, "end_header\n"]
    ).encode("ascii")
    with path.open("wb") as file:
        file.write(header)
        vertex.tofile(file)


def save_cloud_products(output_dir: Path, topic: str, msg: PointCloud2) -> dict[str, Any]:
    name = topic.strip("/").replace("/", "_")
    xyz, rgb, intensity = pointcloud_to_arrays(msg)
    ply_path = output_dir / "pointcloud" / f"{name}_latest.ply"
    npz_path = output_dir / "pointcloud" / f"{name}_latest.npz"
    write_cloud(ply_path, xyz, rgb, intensity)
    np.savez_compressed(
        npz_path,
        xyz=xyz,
        rgb=np.empty((0, 3), dtype=np.uint8) if rgb is None else rgb,
        intensity=np.empty((0,), dtype=np.float32) if intensity is None else intensity,
    )
    return {
        "topic": topic,
        "frame_id": msg.header.frame_id,
        "stamp_ns": stamp_to_ns(msg.header.stamp),
        "points": int(xyz.shape[0]),
        "has_rgb": rgb is not None,
        "has_intensity": intensity is not None,
        "ply_path": str(ply_path),
        "npz_path": str(npz_path),
    }


def cloud_topics_from_args(args: argparse.Namespace) -> list[str]:
    return [args.cloud_topic, *args.extra_cloud_topic]


def collect_cloud_data(
    cloud_sqlite: Path,
    args: argparse.Namespace,
) -> tuple[dict[str, PointCloud2], dict[str, int]]:
    reader = make_reader(cloud_sqlite)
    cloud_topics = cloud_topics_from_args(args)
    set_topic_filter(reader, cloud_topics)

    latest_clouds: dict[str, PointCloud2] = {}
    cloud_counts = {topic: 0 for topic in cloud_topics}

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic in cloud_counts:
            msg = deserialize_message(serialized, PointCloud2)
            latest_clouds[topic] = msg
            cloud_counts[topic] += 1

    return latest_clouds, cloud_counts


def collect_processed_data(
    processed_sqlite: Path,
    args: argparse.Namespace,
    include_clouds: bool = True,
) -> tuple[
    list[TimedTransform],
    MapData,
    dict[int, int],
    dict[tuple[str, str], np.ndarray],
    dict[str, PointCloud2],
    dict[str, int],
]:
    reader = make_reader(processed_sqlite)
    cloud_topics = cloud_topics_from_args(args) if include_clouds else []
    topics = [args.odom_topic, args.map_data_topic, args.tf_static_topic, *cloud_topics]
    set_topic_filter(reader, topics)

    odom_samples: list[TimedTransform] = []
    last_map_data = None
    node_stamps: dict[int, int] = {}
    static_transforms: dict[tuple[str, str], np.ndarray] = {}
    latest_clouds: dict[str, PointCloud2] = {}
    cloud_counts = {topic: 0 for topic in cloud_topics}

    while reader.has_next():
        topic, serialized, _ = reader.read_next()
        if topic == args.odom_topic:
            msg = deserialize_message(serialized, Odometry)
            odom_samples.append(
                TimedTransform(
                    stamp_ns=stamp_to_ns(msg.header.stamp),
                    matrix=matrix_from_pose_msg(msg.pose.pose),
                    reference_frame=msg.header.frame_id,
                    target_frame=msg.child_frame_id,
                )
            )
        elif topic == args.map_data_topic:
            last_map_data = deserialize_message(serialized, MapData)
            for node in last_map_data.nodes:
                node_stamps[int(node.id)] = int(round(float(node.stamp) * 1_000_000_000))
        elif topic == args.tf_static_topic:
            msg = deserialize_message(serialized, TFMessage)
            for transform in msg.transforms:
                static_transforms[(transform.header.frame_id, transform.child_frame_id)] = matrix_from_transform_msg(
                    transform.transform
                )
        elif topic in cloud_counts:
            msg = deserialize_message(serialized, PointCloud2)
            latest_clouds[topic] = msg
            cloud_counts[topic] += 1

    if not odom_samples:
        raise ValueError(f"No odometry samples found on {args.odom_topic}")
    if last_map_data is None:
        raise ValueError(f"No MapData found on {args.map_data_topic}")
    if not node_stamps:
        raise ValueError(f"No node stamps found in {args.map_data_topic}")
    return odom_samples, last_map_data, node_stamps, static_transforms, latest_clouds, cloud_counts


def lookup_static_transform(
    transforms: dict[tuple[str, str], np.ndarray],
    source_frame: str,
    target_frame: str,
) -> np.ndarray:
    if source_frame == target_frame:
        return np.eye(4, dtype=np.float64)

    adjacency: dict[str, list[tuple[str, np.ndarray]]] = {}
    for (parent, child), matrix in transforms.items():
        adjacency.setdefault(parent, []).append((child, matrix))
        adjacency.setdefault(child, []).append((parent, invert_transform(matrix)))

    queue: list[tuple[str, np.ndarray]] = [(source_frame, np.eye(4, dtype=np.float64))]
    visited = {source_frame}
    while queue:
        frame, matrix = queue.pop(0)
        for next_frame, edge in adjacency.get(frame, []):
            if next_frame in visited:
                continue
            next_matrix = matrix @ edge
            if next_frame == target_frame:
                return next_matrix
            visited.add(next_frame)
            queue.append((next_frame, next_matrix))
    raise ValueError(f"No static TF chain found from {source_frame} to {target_frame}")


def build_correction_samples(
    map_data: MapData,
    node_stamps: dict[int, int],
    odom_samples: list[TimedTransform],
    max_odom_gap_ns: int,
) -> list[TimedTransform]:
    samples: list[TimedTransform] = []

    for node_id, pose in zip(map_data.graph.poses_id, map_data.graph.poses):
        node_id = int(node_id)
        stamp_ns = node_stamps.get(node_id)
        if stamp_ns is None:
            continue
        odom_matrix, _, status = interpolate_timed_transform(odom_samples, stamp_ns, max_odom_gap_ns)
        if odom_matrix is None:
            continue
        map_to_base = matrix_from_pose_msg(pose)
        correction = map_to_base @ invert_transform(odom_matrix)
        samples.append(
            TimedTransform(
                stamp_ns=stamp_ns,
                matrix=correction,
                reference_frame=map_data.graph.header.frame_id or "map",
                target_frame=odom_samples[0].reference_frame,
            )
        )

    samples.sort(key=lambda sample: sample.stamp_ns)
    if not samples:
        raise ValueError("No correction samples could be built from final MapData and odometry.")
    return samples


def matrix_columns(prefix: str, matrix: np.ndarray) -> dict[str, float]:
    qx, qy, qz, qw = quaternion_from_matrix(matrix[:3, :3])
    return {
        f"{prefix}_tx": float(matrix[0, 3]),
        f"{prefix}_ty": float(matrix[1, 3]),
        f"{prefix}_tz": float(matrix[2, 3]),
        f"{prefix}_qx": float(qx),
        f"{prefix}_qy": float(qy),
        f"{prefix}_qz": float(qz),
        f"{prefix}_qw": float(qw),
    }


def write_pose_outputs(
    output_dir: Path,
    rgb_stamps: list[RgbStamp],
    odom_samples: list[TimedTransform],
    correction_samples: list[TimedTransform],
    t_base_rgb: np.ndarray,
    args: argparse.Namespace,
) -> dict[str, Any]:
    max_odom_gap_ns = int(args.max_odom_gap_sec * 1_000_000_000)
    max_correction_gap_ns = int(args.max_correction_gap_sec * 1_000_000_000)
    max_correction_extrapolation_ns = int(args.max_correction_extrapolation_sec * 1_000_000_000)
    csv_path = output_dir / "poses" / "rgb_poses.csv"
    jsonl_path = output_dir / "poses" / "rgb_poses.jsonl"

    fieldnames = [
        "rgb_index",
        "rgb_stamp_ns",
        "rgb_stamp_sec",
        "rgb_frame_id",
        "image_path",
        "valid",
        "status",
        "odom_status",
        "odom_dt_ns",
        "correction_status",
        "correction_dt_ns",
        "map_frame",
        "base_frame",
        "rgb_optical_frame",
        *matrix_columns("map_base", np.eye(4)).keys(),
        *matrix_columns("map_rgb_optical", np.eye(4)).keys(),
    ]

    valid_count = 0
    invalid_count = 0
    images_manifest: list[dict[str, Any]] = []
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file, jsonl_path.open(
        "w", encoding="utf-8"
    ) as jsonl_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for rgb in rgb_stamps:
            odom_matrix, odom_dt_ns, odom_status = interpolate_timed_transform(
                odom_samples, rgb.stamp_ns, max_odom_gap_ns
            )
            correction_matrix, correction_dt_ns, correction_status = interpolate_timed_transform(
                correction_samples,
                rgb.stamp_ns,
                max_correction_gap_ns,
                allow_extrapolation_ns=max_correction_extrapolation_ns,
            )

            valid = odom_matrix is not None and correction_matrix is not None
            row: dict[str, Any] = {
                "rgb_index": rgb.index,
                "rgb_stamp_ns": rgb.stamp_ns,
                "rgb_stamp_sec": f"{ns_to_seconds(rgb.stamp_ns):.9f}",
                "rgb_frame_id": rgb.frame_id,
                "image_path": "" if rgb.image_path is None else rgb.image_path,
                "valid": int(valid),
                "status": "ok" if valid else "missing_pose",
                "odom_status": odom_status,
                "odom_dt_ns": "" if odom_dt_ns is None else int(odom_dt_ns),
                "correction_status": correction_status,
                "correction_dt_ns": "" if correction_dt_ns is None else int(correction_dt_ns),
                "map_frame": args.map_frame,
                "base_frame": args.base_frame,
                "rgb_optical_frame": args.rgb_frame,
            }

            if valid:
                valid_count += 1
                t_map_base = correction_matrix @ odom_matrix
                t_map_rgb = t_map_base @ t_base_rgb
                row.update(matrix_columns("map_base", t_map_base))
                row.update(matrix_columns("map_rgb_optical", t_map_rgb))
                q_map_rgb = quaternion_from_matrix(t_map_rgb[:3, :3])
                pose_entry = {
                    "reference_frame": args.map_frame,
                    "target_frame": args.rgb_frame,
                    "translation_xyz": t_map_rgb[:3, 3].astype(float).tolist(),
                    "quaternion_xyzw": q_map_rgb.astype(float).tolist(),
                    "T_map_rgb_optical": t_map_rgb.astype(float).reshape(-1).tolist(),
                    "T_map_base": t_map_base.astype(float).reshape(-1).tolist(),
                }
                json_payload = {
                    **row,
                    **pose_entry,
                }
            else:
                invalid_count += 1
                for key in matrix_columns("map_base", np.eye(4)).keys():
                    row[key] = ""
                for key in matrix_columns("map_rgb_optical", np.eye(4)).keys():
                    row[key] = ""
                json_payload = row
                pose_entry = None

            writer.writerow(row)
            jsonl_file.write(json.dumps(json_payload, ensure_ascii=True) + "\n")
            images_manifest.append(
                {
                    "index": rgb.index,
                    "timestamp_ns": rgb.stamp_ns,
                    "timestamp_sec": ns_to_seconds(rgb.stamp_ns),
                    "frame_id": rgb.frame_id,
                    "path": rgb.image_path,
                    "valid_pose": bool(valid),
                    "pose": pose_entry,
                }
            )

    manifest_path = output_dir / "images_manifest.json"
    write_json(manifest_path, images_manifest)
    return {
        "csv_path": str(csv_path),
        "jsonl_path": str(jsonl_path),
        "images_manifest_path": str(manifest_path),
        "rgb_total": len(rgb_stamps),
        "valid_poses": valid_count,
        "invalid_poses": invalid_count,
    }


def main() -> int:
    args = parse_args()
    output_dir = args.output_dir.expanduser().resolve()
    prepare_output_dir(output_dir, args.overwrite)

    processed_files = resolve_bag_sqlite_files(args.processed_bag)
    if len(processed_files) != 1:
        raise ValueError("This exporter currently expects one processed bag sqlite file.")
    processed_sqlite = decompress_zstd_if_needed(processed_files[0], output_dir / ".cache")

    cloud_bag = args.cloud_bag if args.cloud_bag is not None else args.processed_bag
    cloud_files = resolve_bag_sqlite_files(cloud_bag)
    if len(cloud_files) != 1:
        raise ValueError("This exporter currently expects one cloud bag sqlite file.")
    cloud_sqlite = decompress_zstd_if_needed(cloud_files[0], output_dir / ".cache")
    split_cloud_bag = cloud_sqlite.resolve() != processed_sqlite.resolve()

    print(f"Reading raw RGB stamps from: {args.raw_bag}")
    rgb_stamps = read_rgb_stamps_and_images(
        args.raw_bag,
        args.rgb_topic,
        output_dir,
        args.export_rgb_images,
        args.image_format,
        args.jpeg_quality,
        args.png_compression,
    )
    print(
        f"Loaded RGB stamps: {len(rgb_stamps)}"
        + (" and exported images" if args.export_rgb_images else "")
    )
    rgb_camera_info = read_first_camera_info(args.raw_bag, args.rgb_camera_info_topic)
    if rgb_camera_info is not None:
        write_json(output_dir / "camera" / "rgb_camera_info.json", rgb_camera_info)

    print(f"Reading processed RTAB-Map data from: {processed_sqlite}")
    odom_samples, map_data, node_stamps, static_transforms, latest_clouds, cloud_counts = collect_processed_data(
        processed_sqlite, args, include_clouds=not split_cloud_bag
    )
    if split_cloud_bag:
        print(f"Reading PointCloud2 topics from: {cloud_sqlite}")
        latest_clouds, cloud_counts = collect_cloud_data(cloud_sqlite, args)
    print(f"Loaded odom samples: {len(odom_samples)}")
    print(
        f"Loaded final mapData graph poses: {len(map_data.graph.poses_id)}, "
        f"known node stamps: {len(node_stamps)}"
    )

    t_base_rgb = lookup_static_transform(static_transforms, args.base_frame, args.rgb_frame)
    max_odom_gap_ns = int(args.max_odom_gap_sec * 1_000_000_000)
    correction_samples = build_correction_samples(map_data, node_stamps, odom_samples, max_odom_gap_ns)
    print(f"Built final graph correction samples: {len(correction_samples)}")

    cloud_exports = []
    for topic, msg in sorted(latest_clouds.items()):
        cloud_exports.append(save_cloud_products(output_dir, topic, msg))
        print(f"Exported latest {topic}: {cloud_exports[-1]['points']} points")

    pose_summary = write_pose_outputs(
        output_dir,
        rgb_stamps,
        odom_samples,
        correction_samples,
        t_base_rgb,
        args,
    )
    print(
        "Wrote RGB pose table: "
        f"{pose_summary['valid_poses']}/{pose_summary['rgb_total']} valid poses"
    )

    metadata = {
        "raw_bag": str(args.raw_bag.expanduser().resolve()),
        "processed_bag": str(args.processed_bag.expanduser().resolve()),
        "cloud_bag": str(cloud_bag.expanduser().resolve()),
        "processed_sqlite_used": str(processed_sqlite),
        "cloud_sqlite_used": str(cloud_sqlite),
        "rgb_topic": args.rgb_topic,
        "rgb_camera_info_topic": args.rgb_camera_info_topic,
        "rgb_camera_info": rgb_camera_info,
        "export_rgb_images": args.export_rgb_images,
        "image_format": args.image_format,
        "jpeg_quality": args.jpeg_quality if args.image_format == "jpg" else None,
        "png_compression": args.png_compression if args.image_format == "png" else None,
        "odom_topic": args.odom_topic,
        "map_data_topic": args.map_data_topic,
        "base_frame": args.base_frame,
        "rgb_frame": args.rgb_frame,
        "map_frame": args.map_frame,
        "max_odom_gap_sec": args.max_odom_gap_sec,
        "max_correction_gap_sec": args.max_correction_gap_sec,
        "max_correction_extrapolation_sec": args.max_correction_extrapolation_sec,
        "T_base_rgb_optical": t_base_rgb.astype(float).reshape(-1).tolist(),
        "rgb_stamps": len(rgb_stamps),
        "odom_samples": len(odom_samples),
        "known_node_stamps": len(node_stamps),
        "map_graph_poses": len(map_data.graph.poses_id),
        "correction_samples": len(correction_samples),
        "cloud_topic_counts": cloud_counts,
        "cloud_exports": cloud_exports,
        "pose_summary": pose_summary,
    }
    write_json(output_dir / "metadata.json", metadata)
    print(f"Wrote metadata: {output_dir / 'metadata.json'}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        raise
