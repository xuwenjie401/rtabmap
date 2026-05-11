#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import rerun as rr
import rerun.blueprint as rrb


DEFAULT_EXPORT_DIR = Path("/home/wjxu22/Datasets/outputs/rtab/oak_stereo_imu_processed_export")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Visualize exported OAK RTAB-Map pointcloud and posed RGB images in Rerun."
    )
    parser.add_argument("--export-dir", type=Path, default=DEFAULT_EXPORT_DIR)
    parser.add_argument("--spawn", action="store_true", help="Open a local Rerun viewer window.")
    parser.add_argument("--save-rrd", type=Path, default=None, help="Save a Rerun .rrd file instead of spawning.")
    parser.add_argument("--image-step", type=int, default=50, help="Visualize every Nth valid RGB image.")
    parser.add_argument("--max-images", type=int, default=0, help="Maximum images to log. 0 means no limit.")
    parser.add_argument(
        "--image-plane-distance",
        type=float,
        default=0.12,
        help="Distance in meters at which each RGB image plane is drawn from its camera center.",
    )
    parser.add_argument(
        "--trajectory-point-radius",
        type=float,
        default=0.008,
        help="Radius in meters for RGB trajectory pose samples in the 3D map.",
    )
    parser.add_argument(
        "--camera-axis-length",
        type=float,
        default=0.05,
        help="Axis length in meters for the currently selected RGB camera pose.",
    )
    parser.add_argument(
        "--cloud",
        choices=("map", "obstacles", "ground"),
        default="map",
        help="Which exported pointcloud to visualize.",
    )
    args = parser.parse_args()
    if args.image_step < 1:
        parser.error("--image-step must be >= 1")
    if args.image_plane_distance <= 0.0:
        parser.error("--image-plane-distance must be > 0")
    if args.trajectory_point_radius <= 0.0:
        parser.error("--trajectory-point-radius must be > 0")
    if args.camera_axis_length <= 0.0:
        parser.error("--camera-axis-length must be > 0")
    if args.spawn and args.save_rrd is not None:
        parser.error("--spawn and --save-rrd are mutually exclusive")
    return args


def read_json(path: Path) -> Any:
    with path.open("r", encoding="utf-8") as file:
        return json.load(file)


def load_image_rgb(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"Failed to read image: {path}")
    if image.ndim == 2:
        return image
    if image.shape[2] == 4:
        return cv2.cvtColor(image, cv2.COLOR_BGRA2RGBA)
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def matrix_from_row(row: dict[str, str], prefix: str) -> np.ndarray:
    tx = float(row[f"{prefix}_tx"])
    ty = float(row[f"{prefix}_ty"])
    tz = float(row[f"{prefix}_tz"])
    qx = float(row[f"{prefix}_qx"])
    qy = float(row[f"{prefix}_qy"])
    qz = float(row[f"{prefix}_qz"])
    qw = float(row[f"{prefix}_qw"])

    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    matrix = np.eye(4, dtype=np.float64)
    matrix[0, 0] = 1.0 - 2.0 * (yy + zz)
    matrix[0, 1] = 2.0 * (xy - wz)
    matrix[0, 2] = 2.0 * (xz + wy)
    matrix[1, 0] = 2.0 * (xy + wz)
    matrix[1, 1] = 1.0 - 2.0 * (xx + zz)
    matrix[1, 2] = 2.0 * (yz - wx)
    matrix[2, 0] = 2.0 * (xz - wy)
    matrix[2, 1] = 2.0 * (yz + wx)
    matrix[2, 2] = 1.0 - 2.0 * (xx + yy)
    matrix[:3, 3] = [tx, ty, tz]
    return matrix


def cloud_path(export_dir: Path, cloud_name: str, metadata: dict[str, Any]) -> Path:
    suffix = {
        "map": "rtabmap_cloud_map_latest.npz",
        "obstacles": "rtabmap_cloud_obstacles_latest.npz",
        "ground": "rtabmap_cloud_ground_latest.npz",
    }[cloud_name]
    default_path = export_dir / "pointcloud" / suffix
    if default_path.is_file():
        return default_path

    topic_suffix = {
        "map": "/cloud_map",
        "obstacles": "/cloud_obstacles",
        "ground": "/cloud_ground",
    }[cloud_name]
    for item in metadata.get("cloud_exports", []):
        topic = item.get("topic", "")
        if not topic.endswith(topic_suffix):
            continue
        npz_path = Path(item["npz_path"])
        if not npz_path.is_absolute():
            npz_path = export_dir / npz_path
        if npz_path.is_file():
            return npz_path

    ply_suffix = suffix.removesuffix(".npz") + ".ply"
    default_ply_path = export_dir / "pointcloud" / ply_suffix
    if default_ply_path.is_file():
        return default_ply_path

    for item in metadata.get("cloud_exports", []):
        topic = item.get("topic", "")
        if not topic.endswith(topic_suffix):
            continue
        ply_path = Path(item["ply_path"])
        if not ply_path.is_absolute():
            ply_path = export_dir / ply_path
        if ply_path.is_file():
            return ply_path

    raise FileNotFoundError(f"No exported {cloud_name} cloud found under {export_dir}")


def ply_dtype(type_name: str, endian: str) -> np.dtype:
    type_map = {
        "char": "i1",
        "int8": "i1",
        "uchar": "u1",
        "uint8": "u1",
        "short": "i2",
        "int16": "i2",
        "ushort": "u2",
        "uint16": "u2",
        "int": "i4",
        "int32": "i4",
        "uint": "u4",
        "uint32": "u4",
        "float": "f4",
        "float32": "f4",
        "double": "f8",
        "float64": "f8",
    }
    if type_name not in type_map:
        raise ValueError(f"Unsupported PLY property type: {type_name}")
    code = type_map[type_name]
    if code.endswith("1"):
        return np.dtype(code)
    return np.dtype(endian + code)


def load_ply_cloud(path: Path) -> dict[str, np.ndarray]:
    with path.open("rb") as file:
        first_line = file.readline().decode("utf-8", errors="replace").strip()
        if first_line != "ply":
            raise ValueError(f"Not a PLY file: {path}")

        format_name = None
        vertex_count = None
        active_element = None
        vertex_properties: list[tuple[str, str]] = []

        while True:
            line = file.readline()
            if not line:
                raise ValueError(f"PLY header is missing end_header: {path}")
            text = line.decode("utf-8", errors="replace").strip()
            if text == "end_header":
                break
            parts = text.split()
            if not parts:
                continue
            if parts[0] == "format":
                format_name = parts[1]
            elif parts[0] == "element":
                active_element = parts[1]
                if active_element == "vertex":
                    vertex_count = int(parts[2])
            elif parts[0] == "property" and active_element == "vertex":
                if parts[1] == "list":
                    raise ValueError("PLY list properties on vertex elements are not supported.")
                vertex_properties.append((parts[2], parts[1]))

        if format_name is None or vertex_count is None:
            raise ValueError(f"PLY header is missing format or vertex count: {path}")

        names = [name for name, _ in vertex_properties]
        required = {"x", "y", "z"}
        if not required.issubset(names):
            raise ValueError(f"PLY vertex properties must include x/y/z: {path}")

        if format_name == "ascii":
            table = np.loadtxt(file, dtype=np.float64, max_rows=vertex_count)
            if table.ndim == 1:
                table = table.reshape(1, -1)
            columns = {name: index for index, name in enumerate(names)}
            xyz = table[:, [columns["x"], columns["y"], columns["z"]]].astype(np.float32)
            rgb = None
            if {"red", "green", "blue"}.issubset(columns):
                rgb = table[:, [columns["red"], columns["green"], columns["blue"]]].clip(0, 255).astype(np.uint8)
            return {"xyz": xyz, "rgb": np.empty((0, 3), dtype=np.uint8) if rgb is None else rgb}

        if format_name == "binary_little_endian":
            endian = "<"
        elif format_name == "binary_big_endian":
            endian = ">"
        else:
            raise ValueError(f"Unsupported PLY format: {format_name}")

        dtype = np.dtype([(name, ply_dtype(type_name, endian)) for name, type_name in vertex_properties])
        vertex = np.fromfile(file, dtype=dtype, count=vertex_count)
        if vertex.shape[0] != vertex_count:
            raise ValueError(f"PLY ended early: expected {vertex_count} vertices, got {vertex.shape[0]}")

    xyz = np.stack([vertex["x"], vertex["y"], vertex["z"]], axis=-1).astype(np.float32)
    cloud: dict[str, np.ndarray] = {"xyz": xyz}
    if {"red", "green", "blue"}.issubset(vertex.dtype.names or ()):
        cloud["rgb"] = np.stack([vertex["red"], vertex["green"], vertex["blue"]], axis=-1).astype(np.uint8)
    return cloud


def load_cloud(path: Path) -> dict[str, np.ndarray]:
    if path.suffix == ".npz":
        with np.load(path) as data:
            return {name: data[name] for name in data.files}
    if path.suffix == ".ply":
        return load_ply_cloud(path)
    raise ValueError(f"Unsupported cloud file: {path}")


def colorize_intensity(intensity: np.ndarray) -> np.ndarray | None:
    if intensity.size == 0:
        return None
    values = intensity.astype(np.float32)
    lo = float(np.min(values))
    hi = float(np.max(values))
    if hi > lo:
        values = (values - lo) / (hi - lo)
    else:
        values = np.zeros_like(values)
    gray = (255.0 * values).clip(0, 255).astype(np.uint8)
    return np.stack([gray, gray, gray], axis=-1)


def configure_rerun(export_dir: Path, args: argparse.Namespace) -> Path | None:
    rr.init("oak_rtabmap_processed_export_viewer", spawn=args.spawn)
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial2DView(
                    origin="/world/current_rgb/image",
                    name="Current RGB",
                ),
                rrb.Spatial3DView(
                    origin="/world",
                    contents=[
                        "/world/pointcloud",
                        "/world/rgb_trajectory",
                        "/world/rgb_pose_samples",
                        "/world/current_rgb/**",
                    ],
                    name="Pointcloud Map",
                ),
                column_shares=[1, 2],
            ),
            rrb.SelectionPanel(expanded=True),
            rrb.TimePanel(expanded=True),
            auto_views=False,
        )
    )
    if args.spawn:
        return None
    save_path = args.save_rrd or export_dir / "oak_rtabmap_processed_export.rrd"
    save_path = save_path.expanduser().resolve()
    save_path.parent.mkdir(parents=True, exist_ok=True)
    rr.save(save_path)
    return save_path


def main() -> int:
    args = parse_args()
    export_dir = args.export_dir.expanduser().resolve()
    metadata = read_json(export_dir / "metadata.json")

    rrd_path = configure_rerun(export_dir, args)
    rr.log("world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)

    selected_cloud_path = cloud_path(export_dir, args.cloud, metadata)
    cloud = load_cloud(selected_cloud_path)
    colors = None
    if "rgb" in cloud and cloud["rgb"].shape[0] == cloud["xyz"].shape[0]:
        colors = cloud["rgb"]
    elif "intensity" in cloud:
        colors = colorize_intensity(cloud["intensity"])
    rr.log("world/pointcloud", rr.Points3D(cloud["xyz"], colors=colors), static=True)

    rgb_camera_info = metadata.get("rgb_camera_info")
    pose_csv = export_dir / "poses" / "rgb_poses.csv"
    valid_rows: list[dict[str, str]] = []
    with pose_csv.open("r", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            if row["valid"] == "1" and row["image_path"]:
                valid_rows.append(row)

    trajectory = np.stack(
        [matrix_from_row(row, "map_rgb_optical")[:3, 3].astype(np.float32) for row in valid_rows],
        axis=0,
    )
    rr.log(
        "world/rgb_trajectory",
        rr.LineStrips3D([trajectory], colors=np.array([[255, 170, 0]], dtype=np.uint8)),
        static=True,
    )
    rr.log(
        "world/rgb_pose_samples",
        rr.Points3D(
            trajectory,
            colors=np.tile(np.array([[255, 170, 0]], dtype=np.uint8), (trajectory.shape[0], 1)),
            radii=np.full((trajectory.shape[0],), args.trajectory_point_radius, dtype=np.float32),
        ),
        static=True,
    )

    current_camera_entity = "world/current_rgb"
    current_image_entity = f"{current_camera_entity}/image"
    if rgb_camera_info is not None:
        rr.log(
            current_image_entity,
            rr.Pinhole(
                resolution=[rgb_camera_info["width"], rgb_camera_info["height"]],
                focal_length=[rgb_camera_info["fx"], rgb_camera_info["fy"]],
                principal_point=[rgb_camera_info["cx"], rgb_camera_info["cy"]],
                camera_xyz=rr.ViewCoordinates.RDF,
                image_plane_distance=args.image_plane_distance,
            ),
            static=True,
        )

    loaded_images = 0
    for row in valid_rows:
        image_index = int(row["rgb_index"])
        if image_index % args.image_step != 0:
            continue
        if args.max_images > 0 and loaded_images >= args.max_images:
            break

        matrix = matrix_from_row(row, "map_rgb_optical")
        image = load_image_rgb(export_dir / row["image_path"])

        rr.set_time_sequence("rgb_index", image_index)
        rr.log(
            current_camera_entity,
            rr.Transform3D(
                translation=matrix[:3, 3],
                mat3x3=matrix[:3, :3],
                axis_length=args.camera_axis_length,
            ),
        )
        rr.log(current_image_entity, rr.Image(image))
        loaded_images += 1
    rr.disable_timeline("rgb_index")

    print(f"Export dir: {export_dir}")
    print(f"Pointcloud file: {selected_cloud_path}")
    print(f"Pointcloud points: {cloud['xyz'].shape[0]}")
    print(f"Valid posed RGB images: {len(valid_rows)}")
    print(f"Loaded images into Rerun: {loaded_images}")
    if rrd_path is not None:
        print(f"Saved Rerun recording: {rrd_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
