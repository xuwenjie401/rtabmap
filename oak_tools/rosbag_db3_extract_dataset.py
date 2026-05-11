#!/usr/bin/env python3.10
#
# Usage:
#   Extract images, pointclouds and pose-index manifests from a rosbag2 sqlite3 bag.
#   python3.10 /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rosbag_db3_extract_dataset.py \
#     --bag-path /path/to/bag_dir_or_db3 \
#     --image-topic /oak/rgb/image_raw \
#     --pointcloud-topic /oak/rgb_map \
#     --fixed-frame map
#
#   If a direct pose topic is preferred over TF, pass it explicitly.
#   python3.10 /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rosbag_db3_extract_dataset.py \
#     --bag-path /path/to/bag_dir_or_db3 \
#     --image-topic /oak/rgb/image_raw \
#     --pose-topic /oak/vio/odometry

from __future__ import annotations

import argparse
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import json
from pathlib import Path
import shutil
from typing import Any

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2

from extracted_dataset_common import (
    DEFAULT_DATASET_ROOT,
    invert_transform,
    make_pose_entry,
    matrix_from_translation_quaternion,
    stamp_to_ns,
    write_json,
)


IMAGE_MESSAGE_TYPES = {
    "sensor_msgs/msg/Image",
    "sensor_msgs/msg/CompressedImage",
}
POINTCLOUD_MESSAGE_TYPES = {
    "sensor_msgs/msg/PointCloud2",
}
POSE_MESSAGE_TYPES = {
    "nav_msgs/msg/Odometry",
    "geometry_msgs/msg/PoseStamped",
    "geometry_msgs/msg/PoseWithCovarianceStamped",
}
TF_MESSAGE_TYPES = {
    "tf2_msgs/msg/TFMessage",
}


@dataclass
class PoseSample:
    stamp_ns: int
    matrix: np.ndarray
    reference_frame: str
    target_frame: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Extract images, pointclouds and pose-index manifests from a rosbag2 sqlite3 bag."
    )
    parser.add_argument("--bag-path", type=Path, required=True, help="Bag directory, metadata.yaml or .db3 path.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Output dataset directory. Default is /home/wjxu22/Datasets/outputs/rtab/extracted/<bag_name>.",
    )
    parser.add_argument("--image-topic", type=str, default=None, help="Image topic to extract.")
    parser.add_argument("--pointcloud-topic", type=str, default=None, help="PointCloud2 topic to extract.")
    parser.add_argument(
        "--pose-topic",
        type=str,
        default=None,
        help="Optional direct pose topic. Supported: Odometry, PoseStamped, PoseWithCovarianceStamped.",
    )
    parser.add_argument(
        "--pose-source",
        choices=("auto", "pose", "tf", "none"),
        default="auto",
        help="Pose source used for image/pointcloud manifests.",
    )
    parser.add_argument(
        "--fixed-frame",
        type=str,
        default=None,
        help="Reference frame used when pose source is TF.",
    )
    parser.add_argument("--tf-topic", type=str, default="/tf", help="Dynamic TF topic.")
    parser.add_argument("--tf-static-topic", type=str, default="/tf_static", help="Static TF topic.")
    parser.add_argument(
        "--pose-max-dt-sec",
        type=float,
        default=0.1,
        help="Maximum allowed nearest-neighbor delta when using a direct pose topic.",
    )
    parser.add_argument(
        "--image-target-frame",
        type=str,
        default=None,
        help="Override image target frame for TF lookup. Default uses image header.frame_id.",
    )
    parser.add_argument(
        "--pointcloud-target-frame",
        type=str,
        default=None,
        help="Override pointcloud target frame for TF lookup. Default uses cloud header.frame_id.",
    )
    parser.add_argument("--max-images", type=int, default=0, help="Limit extracted image count. 0 means all.")
    parser.add_argument("--max-pointclouds", type=int, default=0, help="Limit extracted pointcloud count. 0 means all.")
    parser.add_argument("--image-step", type=int, default=1, help="Only extract every Nth image message.")
    parser.add_argument("--pointcloud-step", type=int, default=1, help="Only extract every Nth pointcloud message.")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output directory if it exists.")
    parser.add_argument("--list-topics", action="store_true", help="List bag topics and exit.")

    args = parser.parse_args()
    if args.image_step < 1:
        parser.error("--image-step must be >= 1")
    if args.pointcloud_step < 1:
        parser.error("--pointcloud-step must be >= 1")
    if args.pose_max_dt_sec < 0.0:
        parser.error("--pose-max-dt-sec must be >= 0")
    return args


def resolve_bag_uri(bag_path: Path) -> Path:
    bag_path = bag_path.expanduser().resolve()
    if bag_path.is_file():
        if bag_path.name == "metadata.yaml":
            return bag_path.parent
        if bag_path.suffix == ".db3":
            return bag_path.parent
        raise FileNotFoundError(f"Unsupported bag path: {bag_path}")
    if bag_path.is_dir():
        metadata = bag_path / "metadata.yaml"
        if metadata.is_file():
            if not any(bag_path.glob("*.db3")):
                raise FileNotFoundError(f"No .db3 files found under bag directory: {bag_path}")
            return bag_path
    raise FileNotFoundError(f"Bag not found or metadata.yaml missing: {bag_path}")


def make_reader(uri: Path) -> rosbag2_py.SequentialReader:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def topic_type_map(uri: Path) -> dict[str, str]:
    reader = make_reader(uri)
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def print_topics(topic_types: dict[str, str]) -> None:
    for topic_name in sorted(topic_types):
        print(f"{topic_name}: {topic_types[topic_name]}")


def choose_topic(
    explicit_topic: str | None,
    topic_types: dict[str, str],
    supported_types: set[str],
    label: str,
) -> str | None:
    if explicit_topic is not None:
        if explicit_topic not in topic_types:
            raise ValueError(f"{label} topic not found in bag: {explicit_topic}")
        topic_type = topic_types[explicit_topic]
        if topic_type not in supported_types:
            raise ValueError(f"{label} topic has unsupported type {topic_type}: {explicit_topic}")
        return explicit_topic

    candidates = [name for name, msg_type in topic_types.items() if msg_type in supported_types]
    if len(candidates) == 1:
        return candidates[0]
    if len(candidates) == 0:
        return None
    raise ValueError(f"Multiple {label} topics found, please pass --{label.replace(' ', '-')} explicitly: {candidates}")


def infer_pose_source(args: argparse.Namespace, pose_topic: str | None, topic_types: dict[str, str]) -> str:
    if args.pose_source != "auto":
        return args.pose_source
    if args.fixed_frame and args.tf_topic in topic_types:
        return "tf"
    if pose_topic is not None:
        return "pose"
    return "none"


def deserialize_topic_message(topic_type: str, serialized: bytes) -> Any:
    return deserialize_message(serialized, get_message(topic_type))


def image_stamp_ns(msg: Any, bag_stamp_ns: int) -> int:
    if hasattr(msg, "header") and (msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0):
        return stamp_to_ns(msg.header.stamp)
    return bag_stamp_ns


def pose_sample_from_message(topic_type: str, msg: Any) -> PoseSample:
    if topic_type == "nav_msgs/msg/Odometry":
        translation = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        reference_frame = msg.header.frame_id
        target_frame = msg.child_frame_id if msg.child_frame_id else "pose"
        return PoseSample(
            stamp_ns=stamp_to_ns(msg.header.stamp),
            matrix=matrix_from_translation_quaternion(translation, quaternion),
            reference_frame=reference_frame,
            target_frame=target_frame,
        )
    if topic_type == "geometry_msgs/msg/PoseStamped":
        translation = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        quaternion = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        return PoseSample(
            stamp_ns=stamp_to_ns(msg.header.stamp),
            matrix=matrix_from_translation_quaternion(translation, quaternion),
            reference_frame=msg.header.frame_id,
            target_frame="pose",
        )
    if topic_type == "geometry_msgs/msg/PoseWithCovarianceStamped":
        translation = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        return PoseSample(
            stamp_ns=stamp_to_ns(msg.header.stamp),
            matrix=matrix_from_translation_quaternion(translation, quaternion),
            reference_frame=msg.header.frame_id,
            target_frame="pose",
        )
    raise ValueError(f"Unsupported pose topic type: {topic_type}")


def collect_pose_samples(uri: Path, pose_topic: str, topic_type: str) -> list[PoseSample]:
    reader = make_reader(uri)
    samples: list[PoseSample] = []
    while reader.has_next():
        topic_name, serialized, _ = reader.read_next()
        if topic_name != pose_topic:
            continue
        msg = deserialize_topic_message(topic_type, serialized)
        samples.append(pose_sample_from_message(topic_type, msg))
    samples.sort(key=lambda sample: sample.stamp_ns)
    return samples


def nearest_pose(
    samples: list[PoseSample],
    sample_stamps_ns: list[int],
    stamp_ns: int,
    max_dt_ns: int,
) -> tuple[PoseSample | None, int | None]:
    if not samples:
        return None, None
    idx = bisect_left(sample_stamps_ns, stamp_ns)

    candidates: list[tuple[int, PoseSample]] = []
    if idx < len(samples):
        candidates.append((abs(samples[idx].stamp_ns - stamp_ns), samples[idx]))
    if idx > 0:
        candidates.append((abs(samples[idx - 1].stamp_ns - stamp_ns), samples[idx - 1]))
    if not candidates:
        return None, None

    dt_ns, sample = min(candidates, key=lambda item: item[0])
    if max_dt_ns > 0 and dt_ns > max_dt_ns:
        return None, dt_ns
    return sample, dt_ns


def transform_from_msg(transform_msg: Any) -> np.ndarray:
    translation = [
        transform_msg.translation.x,
        transform_msg.translation.y,
        transform_msg.translation.z,
    ]
    quaternion = [
        transform_msg.rotation.x,
        transform_msg.rotation.y,
        transform_msg.rotation.z,
        transform_msg.rotation.w,
    ]
    return matrix_from_translation_quaternion(translation, quaternion)


def lookup_tf_transform(
    fixed_frame: str,
    target_frame: str,
    static_edges: dict[tuple[str, str], np.ndarray],
    dynamic_edges: dict[tuple[str, str], tuple[int, np.ndarray]],
) -> tuple[np.ndarray | None, int | None]:
    if fixed_frame == target_frame:
        return np.eye(4, dtype=np.float64), 0

    adjacency: dict[str, list[tuple[str, np.ndarray, int | None]]] = {}

    def add_edge(parent: str, child: str, matrix: np.ndarray, age_ns: int | None) -> None:
        adjacency.setdefault(parent, []).append((child, matrix, age_ns))
        adjacency.setdefault(child, []).append((parent, invert_transform(matrix), age_ns))

    for (parent, child), matrix in static_edges.items():
        add_edge(parent, child, matrix, 0)
    for (parent, child), (stamp_ns, matrix) in dynamic_edges.items():
        add_edge(parent, child, matrix, stamp_ns)

    queue: deque[tuple[str, np.ndarray, int]] = deque([(fixed_frame, np.eye(4, dtype=np.float64), 0)])
    visited = {fixed_frame}

    while queue:
        frame, accumulated, max_age_ns = queue.popleft()
        for neighbor, edge_matrix, edge_age_ns in adjacency.get(frame, []):
            if neighbor in visited:
                continue
            next_age_ns = max(max_age_ns, 0 if edge_age_ns is None else edge_age_ns)
            next_transform = accumulated @ edge_matrix
            if neighbor == target_frame:
                return next_transform, next_age_ns
            visited.add(neighbor)
            queue.append((neighbor, next_transform, next_age_ns))
    return None, None


def deduce_image_extension(compressed_format: str) -> str:
    lowered = compressed_format.lower()
    if "png" in lowered:
        return ".png"
    if "jpg" in lowered or "jpeg" in lowered:
        return ".jpg"
    return ".bin"


def image_array_from_message(msg: Any) -> np.ndarray:
    encoding = msg.encoding.lower()
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if encoding in ("rgb8",):
        return data.reshape(msg.height, msg.width, 3)
    if encoding in ("bgr8",):
        return data.reshape(msg.height, msg.width, 3)[:, :, ::-1]
    if encoding in ("rgba8",):
        return data.reshape(msg.height, msg.width, 4)
    if encoding in ("bgra8",):
        return data.reshape(msg.height, msg.width, 4)[:, :, [2, 1, 0, 3]]
    if encoding in ("mono8", "8uc1"):
        return data.reshape(msg.height, msg.width)
    if encoding in ("mono16", "16uc1"):
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def write_image_message(output_path: Path, topic_type: str, msg: Any) -> tuple[int, int, str]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if topic_type == "sensor_msgs/msg/CompressedImage":
        output_path.write_bytes(bytes(msg.data))
        image = cv2.imread(str(output_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            raise ValueError(f"Failed to decode saved compressed image: {output_path}")
        return int(image.shape[1]), int(image.shape[0]), msg.format

    image_array = image_array_from_message(msg)
    if image_array.ndim == 3 and image_array.shape[2] == 3:
        success = cv2.imwrite(str(output_path), cv2.cvtColor(image_array, cv2.COLOR_RGB2BGR))
    elif image_array.ndim == 3 and image_array.shape[2] == 4:
        success = cv2.imwrite(str(output_path), cv2.cvtColor(image_array, cv2.COLOR_RGBA2BGRA))
    else:
        success = cv2.imwrite(str(output_path), image_array)
    if not success:
        raise ValueError(f"Failed to write image: {output_path}")
    return int(msg.width), int(msg.height), msg.encoding


def pointcloud_to_arrays(msg: Any) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
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
        r = ((packed >> 16) & 0xFF).astype(np.uint8)
        g = ((packed >> 8) & 0xFF).astype(np.uint8)
        b = (packed & 0xFF).astype(np.uint8)
        rgb = np.stack([r, g, b], axis=-1)
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


def save_pointcloud_npz(output_path: Path, xyz: np.ndarray, rgb: np.ndarray | None, intensity: np.ndarray | None) -> dict[str, Any]:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    payload: dict[str, Any] = {"xyz": xyz.astype(np.float32)}
    fields = ["xyz"]
    if rgb is not None:
        payload["rgb"] = rgb.astype(np.uint8)
        fields.append("rgb")
    if intensity is not None:
        payload["intensity"] = intensity.astype(np.float32)
        fields.append("intensity")
    np.savez(output_path, **payload)
    return {"point_count": int(xyz.shape[0]), "fields": fields}


def prepare_output_dir(path: Path, overwrite: bool) -> None:
    if path.exists():
        if not overwrite:
            raise FileExistsError(f"Output directory already exists: {path}")
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def main() -> int:
    args = parse_args()
    bag_uri = resolve_bag_uri(args.bag_path)
    topic_types = topic_type_map(bag_uri)

    if args.list_topics:
        print_topics(topic_types)
        return 0

    image_topic = choose_topic(args.image_topic, topic_types, IMAGE_MESSAGE_TYPES, "image")
    pointcloud_topic = choose_topic(args.pointcloud_topic, topic_types, POINTCLOUD_MESSAGE_TYPES, "pointcloud")
    pose_topic = choose_topic(args.pose_topic, topic_types, POSE_MESSAGE_TYPES, "pose") if args.pose_topic or any(t in POSE_MESSAGE_TYPES for t in topic_types.values()) else None

    if image_topic is None and pointcloud_topic is None:
        raise ValueError("No image or pointcloud topic selected.")

    pose_source = infer_pose_source(args, pose_topic, topic_types)
    if pose_source == "tf" and not args.fixed_frame:
        raise ValueError("--fixed-frame is required when pose source is tf.")

    bag_name = bag_uri.name
    output_dir = args.output_dir.expanduser().resolve() if args.output_dir else (DEFAULT_DATASET_ROOT / bag_name)
    prepare_output_dir(output_dir, args.overwrite)
    images_dir = output_dir / "images"
    pointclouds_dir = output_dir / "pointclouds"

    print(f"Bag: {bag_uri}")
    print(f"Output: {output_dir}")
    print(f"Image topic: {image_topic}")
    print(f"Pointcloud topic: {pointcloud_topic}")
    print(f"Pose source: {pose_source}")

    pose_samples: list[PoseSample] = []
    pose_sample_stamps_ns: list[int] = []
    max_pose_dt_ns = int(args.pose_max_dt_sec * 1_000_000_000.0)
    if pose_source == "pose" and pose_topic is not None:
        pose_samples = collect_pose_samples(bag_uri, pose_topic, topic_types[pose_topic])
        pose_sample_stamps_ns = [sample.stamp_ns for sample in pose_samples]
        print(f"Collected {len(pose_samples)} pose samples from {pose_topic}")

    reader = make_reader(bag_uri)
    static_tf_edges: dict[tuple[str, str], np.ndarray] = {}
    dynamic_tf_edges: dict[tuple[str, str], tuple[int, np.ndarray]] = {}

    images_manifest: list[dict[str, Any]] = []
    pointclouds_manifest: list[dict[str, Any]] = []
    image_seen = 0
    pointcloud_seen = 0

    while reader.has_next():
        topic_name, serialized, bag_stamp_ns = reader.read_next()
        topic_type = topic_types.get(topic_name)
        if topic_type in TF_MESSAGE_TYPES and pose_source == "tf":
            tf_msg = deserialize_topic_message(topic_type, serialized)
            for transform_stamped in tf_msg.transforms:
                parent = transform_stamped.header.frame_id
                child = transform_stamped.child_frame_id
                matrix = transform_from_msg(transform_stamped.transform)
                stamp_ns = stamp_to_ns(transform_stamped.header.stamp)
                if topic_name == args.tf_static_topic:
                    static_tf_edges[(parent, child)] = matrix
                else:
                    dynamic_tf_edges[(parent, child)] = (stamp_ns, matrix)
            continue

        if topic_name == image_topic:
            if args.max_images > 0 and len(images_manifest) >= args.max_images:
                continue
            if image_seen % args.image_step != 0:
                image_seen += 1
                continue

            msg = deserialize_topic_message(topic_type, serialized)
            stamp_ns = image_stamp_ns(msg, bag_stamp_ns)
            frame_id = msg.header.frame_id if hasattr(msg, "header") else ""
            pose_entry = None

            if pose_source == "pose" and pose_samples:
                sample, dt_ns = nearest_pose(
                    pose_samples,
                    pose_sample_stamps_ns,
                    stamp_ns,
                    max_pose_dt_ns,
                )
                if sample is not None:
                    pose_entry = make_pose_entry(
                        sample.matrix,
                        sample.reference_frame,
                        sample.target_frame,
                        source="pose_topic",
                        dt_ns=dt_ns,
                    )
            elif pose_source == "tf" and args.fixed_frame:
                target_frame = args.image_target_frame or frame_id
                if target_frame:
                    matrix, max_edge_stamp_ns = lookup_tf_transform(
                        args.fixed_frame,
                        target_frame,
                        static_tf_edges,
                        dynamic_tf_edges,
                    )
                    if matrix is not None:
                        age_ns = None if max_edge_stamp_ns is None else max(0, stamp_ns - max_edge_stamp_ns)
                        pose_entry = make_pose_entry(
                            matrix,
                            args.fixed_frame,
                            target_frame,
                            source="tf",
                            age_ns=age_ns,
                        )

            image_ext = ".png"
            if topic_type == "sensor_msgs/msg/CompressedImage":
                image_ext = deduce_image_extension(msg.format)
            image_path = images_dir / f"{len(images_manifest):06d}{image_ext}"
            width, height, encoding = write_image_message(image_path, topic_type, msg)

            images_manifest.append(
                {
                    "index": len(images_manifest),
                    "timestamp_ns": int(stamp_ns),
                    "topic": image_topic,
                    "path": str(image_path.relative_to(output_dir)),
                    "frame_id": frame_id,
                    "width": width,
                    "height": height,
                    "encoding": encoding,
                    "pose": pose_entry,
                }
            )
            image_seen += 1
            if len(images_manifest) % 50 == 0:
                print(f"Extracted images: {len(images_manifest)}")
            continue

        if topic_name == pointcloud_topic:
            if args.max_pointclouds > 0 and len(pointclouds_manifest) >= args.max_pointclouds:
                continue
            if pointcloud_seen % args.pointcloud_step != 0:
                pointcloud_seen += 1
                continue

            msg = deserialize_topic_message(topic_type, serialized)
            stamp_ns = image_stamp_ns(msg, bag_stamp_ns)
            frame_id = msg.header.frame_id
            pose_entry = None

            if pose_source == "pose" and pose_samples:
                sample, dt_ns = nearest_pose(
                    pose_samples,
                    pose_sample_stamps_ns,
                    stamp_ns,
                    max_pose_dt_ns,
                )
                if sample is not None:
                    pose_entry = make_pose_entry(
                        sample.matrix,
                        sample.reference_frame,
                        sample.target_frame,
                        source="pose_topic",
                        dt_ns=dt_ns,
                    )
            elif pose_source == "tf" and args.fixed_frame:
                target_frame = args.pointcloud_target_frame or frame_id
                if target_frame:
                    matrix, max_edge_stamp_ns = lookup_tf_transform(
                        args.fixed_frame,
                        target_frame,
                        static_tf_edges,
                        dynamic_tf_edges,
                    )
                    if matrix is not None:
                        age_ns = None if max_edge_stamp_ns is None else max(0, stamp_ns - max_edge_stamp_ns)
                        pose_entry = make_pose_entry(
                            matrix,
                            args.fixed_frame,
                            target_frame,
                            source="tf",
                            age_ns=age_ns,
                        )

            xyz, rgb, intensity = pointcloud_to_arrays(msg)
            cloud_path = pointclouds_dir / f"{len(pointclouds_manifest):06d}.npz"
            cloud_info = save_pointcloud_npz(cloud_path, xyz, rgb, intensity)

            pointclouds_manifest.append(
                {
                    "index": len(pointclouds_manifest),
                    "timestamp_ns": int(stamp_ns),
                    "topic": pointcloud_topic,
                    "path": str(cloud_path.relative_to(output_dir)),
                    "frame_id": frame_id,
                    "pose": pose_entry,
                    **cloud_info,
                }
            )
            pointcloud_seen += 1
            if len(pointclouds_manifest) % 20 == 0:
                print(f"Extracted pointclouds: {len(pointclouds_manifest)}")

    dataset_info = {
        "version": 1,
        "source_bag": str(bag_uri),
        "image_topic": image_topic,
        "pointcloud_topic": pointcloud_topic,
        "pose_topic": pose_topic,
        "pose_source": pose_source,
        "fixed_frame": args.fixed_frame,
        "counts": {
            "images": len(images_manifest),
            "pointclouds": len(pointclouds_manifest),
        },
        "files": {
            "images_manifest": "images_manifest.json",
            "pointclouds_manifest": "pointclouds_manifest.json",
        },
    }

    write_json(output_dir / "dataset_info.json", dataset_info)
    write_json(output_dir / "images_manifest.json", images_manifest)
    write_json(output_dir / "pointclouds_manifest.json", pointclouds_manifest)

    print(json.dumps(dataset_info["counts"], ensure_ascii=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
