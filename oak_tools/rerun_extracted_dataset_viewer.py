#!/usr/bin/env python3
#
# Usage:
#   Load data extracted by rosbag_db3_extract_dataset.py.
#   conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rerun_extracted_dataset_viewer.py \
#     --dataset-dir /home/wjxu22/Datasets/outputs/rtab/extracted/my_bag \
#     --spawn
#
#   Save an .rrd instead of opening a window.
#   conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/rerun_extracted_dataset_viewer.py \
#     --dataset-dir /home/wjxu22/Datasets/outputs/rtab/extracted/my_bag \
#     --save-rrd /home/wjxu22/Datasets/outputs/rtab/extracted/my_bag.rrd

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rerun as rr

from extracted_dataset_common import DEFAULT_DATASET_ROOT, ns_to_seconds, pose_matrix_from_entry


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Load a dataset extracted from rosbag_db3_extract_dataset.py and visualize it in Rerun."
    )
    parser.add_argument("--dataset-dir", type=Path, required=True, help="Extracted dataset directory.")
    parser.add_argument("--spawn", action="store_true", help="Spawn a local Rerun viewer window.")
    parser.add_argument("--save-rrd", type=Path, default=None, help="Save a .rrd recording instead of spawning.")
    parser.add_argument("--image-step", type=int, default=1, help="Only load every Nth extracted image.")
    parser.add_argument("--pointcloud-step", type=int, default=1, help="Only load every Nth extracted pointcloud.")
    parser.add_argument("--max-images", type=int, default=0, help="Limit image count. 0 means all.")
    parser.add_argument("--max-pointclouds", type=int, default=0, help="Limit pointcloud count. 0 means all.")
    args = parser.parse_args()
    if args.image_step < 1:
        parser.error("--image-step must be >= 1")
    if args.pointcloud_step < 1:
        parser.error("--pointcloud-step must be >= 1")
    if args.spawn and args.save_rrd is not None:
        parser.error("--spawn and --save-rrd are mutually exclusive")
    return args


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def configure_rerun(dataset_dir: Path, args: argparse.Namespace) -> Path | None:
    rr.init("rerun_extracted_dataset_viewer", spawn=args.spawn)
    if args.spawn:
        return None

    save_path = args.save_rrd
    if save_path is None:
        save_path = dataset_dir.with_suffix(".rrd")
    save_path = save_path.expanduser().resolve()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    rr.save(save_path)
    return save_path


def load_image_rgb(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"Failed to read image: {path}")
    if image.ndim == 2:
        return image
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2RGBA)
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def colorize_intensity(intensity: np.ndarray) -> np.ndarray:
    if intensity.size == 0:
        return np.empty((0, 3), dtype=np.uint8)
    normalized = intensity.astype(np.float32)
    min_value = float(np.min(normalized))
    max_value = float(np.max(normalized))
    if max_value > min_value:
        normalized = (normalized - min_value) / (max_value - min_value)
    else:
        normalized = np.zeros_like(normalized)
    gray = (normalized * 255.0).clip(0, 255).astype(np.uint8)
    return np.stack([gray, gray, gray], axis=-1)


def log_pose(entity_path: str, pose_entry: dict[str, Any]) -> None:
    matrix = pose_matrix_from_entry(pose_entry)
    rr.log(
        entity_path,
        rr.Transform3D(
            translation=matrix[:3, 3],
            mat3x3=matrix[:3, :3],
            axis_length=0.1,
        ),
    )


def log_pinhole_if_available(entity_path: str, image_item: dict[str, Any]) -> None:
    camera_model = image_item.get("camera_model")
    if not camera_model:
        return
    rr.log(
        entity_path,
        rr.Pinhole(
            resolution=[camera_model["width"], camera_model["height"]],
            focal_length=[camera_model["fx"], camera_model["fy"]],
            principal_point=[camera_model["cx"], camera_model["cy"]],
            camera_xyz=rr.ViewCoordinates.RDF,
        ),
    )


def main() -> int:
    args = parse_args()
    dataset_dir = args.dataset_dir.expanduser().resolve()
    if not dataset_dir.is_dir():
        raise FileNotFoundError(f"Dataset directory not found: {dataset_dir}")

    dataset_info = read_json(dataset_dir / "dataset_info.json")
    images_manifest = read_json(dataset_dir / dataset_info["files"]["images_manifest"])
    pointclouds_manifest = read_json(dataset_dir / dataset_info["files"]["pointclouds_manifest"])

    rrd_path = configure_rerun(dataset_dir, args)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    trajectory_positions: list[np.ndarray] = []
    loaded_images = 0
    loaded_pointclouds = 0

    for item in images_manifest:
        if args.max_images > 0 and loaded_images >= args.max_images:
            break
        if item["index"] % args.image_step != 0:
            continue

        stamp_seconds = ns_to_seconds(int(item["timestamp_ns"]))
        rr.set_time_seconds("stamp", stamp_seconds)
        rr.set_time_sequence("image_index", int(item["index"]))

        image_path = dataset_dir / item["path"]
        image = load_image_rgb(image_path)
        if item["pose"] is not None:
            log_pose("world/camera", item["pose"])
            pose_matrix = pose_matrix_from_entry(item["pose"])
            trajectory_positions.append(pose_matrix[:3, 3].astype(np.float32))
        log_pinhole_if_available("world/camera", item)
        rr.log("world/camera/rgb", rr.Image(image))
        loaded_images += 1

    for item in pointclouds_manifest:
        if args.max_pointclouds > 0 and loaded_pointclouds >= args.max_pointclouds:
            break
        if item["index"] % args.pointcloud_step != 0:
            continue

        stamp_seconds = ns_to_seconds(int(item["timestamp_ns"]))
        rr.set_time_seconds("stamp", stamp_seconds)
        rr.set_time_sequence("cloud_index", int(item["index"]))

        cloud = np.load(dataset_dir / item["path"])
        colors = None
        if "rgb" in cloud:
            colors = cloud["rgb"]
        elif "intensity" in cloud:
            colors = colorize_intensity(cloud["intensity"])

        if item.get("static"):
            rr.reset_time()
            rr.log(
                item.get("entity_path", "world/map"),
                rr.Points3D(cloud["xyz"], colors=colors),
                static=True,
            )
            loaded_pointclouds += 1
            continue

        if item["pose"] is not None:
            log_pose("world/cloud_sensor", item["pose"])
            rr.log("world/cloud_sensor/points", rr.Points3D(cloud["xyz"], colors=colors))
        else:
            rr.log("world/clouds", rr.Points3D(cloud["xyz"], colors=colors))
        loaded_pointclouds += 1

    if trajectory_positions:
        rr.reset_time()
        trajectory = np.stack(trajectory_positions, axis=0)
        rr.log(
            "world/trajectory",
            rr.LineStrips3D([trajectory], colors=np.array([[255, 170, 0]], dtype=np.uint8)),
            static=True,
        )
        rr.log(
            "world/camera_positions",
            rr.Points3D(
                trajectory,
                colors=np.tile(np.array([[255, 170, 0]], dtype=np.uint8), (trajectory.shape[0], 1)),
                radii=np.full((trajectory.shape[0],), 0.03, dtype=np.float32),
            ),
            static=True,
        )

    print(f"Loaded images: {loaded_images}")
    print(f"Loaded pointclouds: {loaded_pointclouds}")
    if rrd_path is not None:
        print(f"Rerun recording saved to: {rrd_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
