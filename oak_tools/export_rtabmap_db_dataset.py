#!/usr/bin/env python3
#
# Usage:
#   Export global pointcloud plus RGB images with poses from an RTAB-Map .db.
#   conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/export_rtabmap_db_dataset.py \
#     --db-path /home/wjxu22/Datasets/outputs/rtab/xwj_room.db \
#     --output-dir /home/wjxu22/Datasets/outputs/rtab/xwj_room_export
#
#   Add explicit pointcloud downsampling only when needed.
#   conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/export_rtabmap_db_dataset.py \
#     --db-path /home/wjxu22/Datasets/outputs/rtab/xwj_room.db \
#     --output-dir /home/wjxu22/Datasets/outputs/rtab/xwj_room_export \
#     --image-decimation 2 \
#     --voxel-size 0.03 \
#     --max-points 1000000

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import sqlite3
import sys
from typing import Any

try:
    import cv2
except ModuleNotFoundError as exc:
    raise SystemExit(
        "OpenCV (cv2) is required. Run this tool with your conda environment, for example: "
        "conda run -n jarvis python /home/wjxu22/SLAMBot/rtab_ws/src/rtabmap/oak_tools/export_rtabmap_db_dataset.py ..."
    ) from exc
import numpy as np

from extracted_dataset_common import make_pose_entry, write_json
from rtabmap_db_common import (
    DEFAULT_OUTPUT_DIR,
    apply_transform,
    build_cloud,
    decode_depth,
    decode_image,
    limit_points,
    load_optimized_poses,
    parse_camera_models,
    parse_pose_blob,
    resolve_db_path,
    voxel_downsample,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export RGB images with poses and a global colored pointcloud from an RTAB-Map database."
    )
    parser.add_argument("--db-path", type=Path, required=True, help="RTAB-Map .db path.")
    parser.add_argument("--output-dir", type=Path, required=True, help="Directory to write exported data into.")
    parser.add_argument(
        "--image-format",
        choices=("png", "jpg"),
        default="png",
        help="Format used for exported RGB images. Default png avoids extra loss.",
    )
    parser.add_argument(
        "--jpeg-quality",
        type=int,
        default=100,
        help="JPEG quality used only when --image-format=jpg.",
    )
    parser.add_argument(
        "--image-decimation",
        type=int,
        default=1,
        help="Pixel stride used when reconstructing the exported global pointcloud. Default 1 means no decimation.",
    )
    parser.add_argument("--min-depth", type=float, default=0.2, help="Minimum valid depth in meters.")
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
        help="Voxel size for final exported pointcloud. Default 0 disables voxel downsampling.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=0,
        help="Maximum final point count for exported pointcloud. Default 0 disables this limit.",
    )
    parser.add_argument("--max-nodes", type=int, default=0, help="Limit number of database nodes to export. 0 means all.")
    parser.add_argument("--node-step", type=int, default=1, help="Only export every Nth node.")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite output directory if it already exists.")
    args = parser.parse_args()

    if args.image_decimation < 1:
        parser.error("--image-decimation must be >= 1")
    if args.node_step < 1:
        parser.error("--node-step must be >= 1")
    if not (1 <= args.jpeg_quality <= 100):
        parser.error("--jpeg-quality must be in [1, 100]")
    return args


def prepare_output_dir(path: Path, overwrite: bool) -> None:
    if path.exists():
        if not overwrite:
            raise FileExistsError(f"Output directory already exists: {path}")
        shutil.rmtree(path)
    path.mkdir(parents=True, exist_ok=True)


def save_rgb_image(path: Path, rgb: np.ndarray, image_format: str, jpeg_quality: int) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    if image_format == "png":
        ok = cv2.imwrite(str(path), bgr)
    else:
        ok = cv2.imwrite(str(path), bgr, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
    if not ok:
        raise ValueError(f"Failed to write image: {path}")


def write_ply(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    vertex_count = points.shape[0]
    vertex_array = np.empty(
        vertex_count,
        dtype=[
            ("x", "<f4"),
            ("y", "<f4"),
            ("z", "<f4"),
            ("red", "u1"),
            ("green", "u1"),
            ("blue", "u1"),
        ],
    )
    vertex_array["x"] = points[:, 0]
    vertex_array["y"] = points[:, 1]
    vertex_array["z"] = points[:, 2]
    vertex_array["red"] = colors[:, 0]
    vertex_array["green"] = colors[:, 1]
    vertex_array["blue"] = colors[:, 2]

    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {vertex_count}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n"
    ).encode("ascii")

    with path.open("wb") as file:
        file.write(header)
        file.write(vertex_array.tobytes())


def camera_model_entry(model: Any) -> dict[str, Any]:
    return {
        "fx": float(model.fx),
        "fy": float(model.fy),
        "cx": float(model.cx),
        "cy": float(model.cy),
        "width": int(model.width),
        "height": int(model.height),
        "local_transform_3x4": model.local_transform[:3, :].astype(float).reshape(-1).tolist(),
    }


def main() -> int:
    args = parse_args()
    db_path = resolve_db_path(args.db_path, DEFAULT_OUTPUT_DIR)
    output_dir = args.output_dir.expanduser().resolve()
    prepare_output_dir(output_dir, args.overwrite)
    rgb_dir = output_dir / "rgb"
    cloud_dir = output_dir / "pointcloud"

    print(f"Reading database: {db_path}")
    print(f"Writing export to: {output_dir}")

    conn = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True)
    conn.row_factory = sqlite3.Row

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

    image_manifest: list[dict[str, Any]] = []
    point_chunks: list[np.ndarray] = []
    color_chunks: list[np.ndarray] = []
    skipped = 0
    processed = 0
    image_ext = ".png" if args.image_format == "png" else ".jpg"

    for row_index, row in enumerate(rows):
        if args.max_nodes > 0 and processed >= args.max_nodes:
            break
        if row_index % args.node_step != 0:
            continue

        node_id = int(row["id"])
        pose = optimized_poses.get(node_id)
        if pose is None:
            pose = parse_pose_blob(row["pose"])
        if optimized_poses and node_id not in optimized_poses:
            skipped += 1
            continue
        if pose is None or row["image"] is None or row["depth"] is None or row["calibration"] is None:
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

        rgb_path = rgb_dir / f"{processed:06d}{image_ext}"
        save_rgb_image(rgb_path, rgb, args.image_format, args.jpeg_quality)

        camera_pose = pose @ model.local_transform
        stamp_seconds = float(row["stamp"])
        timestamp_ns = int(round(stamp_seconds * 1_000_000_000.0))
        image_manifest.append(
            {
                "index": processed,
                "node_id": node_id,
                "timestamp_ns": timestamp_ns,
                "timestamp_seconds": stamp_seconds,
                "topic": "rtabmap_db_rgb",
                "path": str(rgb_path.relative_to(output_dir)),
                "frame_id": "camera_optical_frame",
                "pose": make_pose_entry(
                    camera_pose,
                    reference_frame="optimized_map",
                    target_frame="camera_optical_frame",
                    source="rtabmap_db",
                ),
                "node_pose": make_pose_entry(
                    pose,
                    reference_frame="optimized_map",
                    target_frame="base_frame",
                    source="rtabmap_db",
                ),
                "camera_model": camera_model_entry(model),
                "image_shape": [int(rgb.shape[0]), int(rgb.shape[1]), int(rgb.shape[2])],
            }
        )

        camera_points = apply_transform(local_points, model.local_transform)
        world_points = apply_transform(camera_points, pose)
        point_chunks.append(world_points.astype(np.float32))
        color_chunks.append(local_colors.astype(np.uint8))

        processed += 1
        if processed % 10 == 0:
            print(f"Processed nodes: {processed}")

    if not point_chunks:
        print("No valid RGB-D nodes were exported.", file=sys.stderr)
        return 1

    points = np.concatenate(point_chunks, axis=0)
    colors = np.concatenate(color_chunks, axis=0)
    raw_point_count = points.shape[0]

    if args.voxel_size > 0:
        points, colors = voxel_downsample(points, colors, args.voxel_size)
    if args.max_points > 0:
        points, colors = limit_points(points, colors, args.max_points)

    cloud_npz_path = cloud_dir / "map_cloud.npz"
    cloud_ply_path = cloud_dir / "map_cloud.ply"
    cloud_dir.mkdir(parents=True, exist_ok=True)
    np.savez(cloud_npz_path, xyz=points.astype(np.float32), rgb=colors.astype(np.uint8))
    write_ply(cloud_ply_path, points.astype(np.float32), colors.astype(np.uint8))

    pointcloud_manifest = [
        {
            "index": 0,
            "timestamp_ns": 0,
            "topic": "rtabmap_db_global_map",
            "path": str(cloud_npz_path.relative_to(output_dir)),
            "ply_path": str(cloud_ply_path.relative_to(output_dir)),
            "frame_id": "optimized_map",
            "pose": None,
            "point_count": int(points.shape[0]),
            "fields": ["xyz", "rgb"],
            "static": True,
            "entity_path": "world/map",
        }
    ]

    export_info = {
        "version": 1,
        "source_db": str(db_path),
        "counts": {
            "exported_images": len(image_manifest),
            "raw_point_count": int(raw_point_count),
            "final_point_count": int(points.shape[0]),
            "skipped_nodes": int(skipped),
        },
        "pointcloud_files": {
            "npz": str(cloud_npz_path.relative_to(output_dir)),
            "ply": str(cloud_ply_path.relative_to(output_dir)),
        },
        "image_format": args.image_format,
        "pointcloud_reconstruction": {
            "image_decimation": int(args.image_decimation),
            "min_depth": float(args.min_depth),
            "max_depth": float(args.max_depth),
            "voxel_size": float(args.voxel_size),
            "max_points": int(args.max_points),
        },
        "files": {
            "images_manifest": "images_manifest.json",
            "pointclouds_manifest": "pointclouds_manifest.json",
        },
    }

    write_json(output_dir / "images_manifest.json", image_manifest)
    write_json(output_dir / "pointclouds_manifest.json", pointcloud_manifest)
    write_json(output_dir / "export_info.json", export_info)
    write_json(output_dir / "dataset_info.json", export_info)

    print(f"Exported images: {len(image_manifest)}")
    print(f"Raw merged points: {raw_point_count}")
    print(f"Final exported points: {points.shape[0]}")
    print(f"Pointcloud NPZ: {cloud_npz_path}")
    print(f"Pointcloud PLY: {cloud_ply_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
