#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Subscribe to a PointCloud2 topic and save the latest cloud to PLY on shutdown."
    )
    parser.add_argument("--topic", required=True, help="PointCloud2 topic to subscribe to.")
    parser.add_argument("--output", type=Path, required=True, help="Output PLY file path.")
    return parser.parse_args()


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


def write_binary_ply(
    output_path: Path,
    xyz: np.ndarray,
    rgb: np.ndarray | None,
    intensity: np.ndarray | None,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)

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
        header = [
            "ply",
            "format binary_little_endian 1.0",
            f"element vertex {xyz.shape[0]}",
            "property float x",
            "property float y",
            "property float z",
            "property uchar red",
            "property uchar green",
            "property uchar blue",
            "end_header\n",
        ]
    elif intensity is not None:
        vertex = np.empty(
            xyz.shape[0],
            dtype=[
                ("x", "<f4"),
                ("y", "<f4"),
                ("z", "<f4"),
                ("intensity", "<f4"),
            ],
        )
        vertex["x"] = xyz[:, 0]
        vertex["y"] = xyz[:, 1]
        vertex["z"] = xyz[:, 2]
        vertex["intensity"] = intensity
        header = [
            "ply",
            "format binary_little_endian 1.0",
            f"element vertex {xyz.shape[0]}",
            "property float x",
            "property float y",
            "property float z",
            "property float intensity",
            "end_header\n",
        ]
    else:
        vertex = np.empty(
            xyz.shape[0],
            dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")],
        )
        vertex["x"] = xyz[:, 0]
        vertex["y"] = xyz[:, 1]
        vertex["z"] = xyz[:, 2]
        header = [
            "ply",
            "format binary_little_endian 1.0",
            f"element vertex {xyz.shape[0]}",
            "property float x",
            "property float y",
            "property float z",
            "end_header\n",
        ]

    with output_path.open("wb") as f:
        f.write("\n".join(header).encode("ascii"))
        vertex.tofile(f)


class PointCloudSnapshotSaver(Node):
    def __init__(self, topic: str, output_path: Path) -> None:
        super().__init__("pointcloud_snapshot_saver")
        self._topic = topic
        self._output_path = output_path.expanduser().resolve()
        self._latest_msg: PointCloud2 | None = None
        self._received_count = 0
        self._saved = False
        self.create_subscription(PointCloud2, topic, self._cloud_callback, 1)
        self.get_logger().info(
            f"Listening on {topic}, will save the latest cloud to {self._output_path} on shutdown."
        )

    def _cloud_callback(self, msg: PointCloud2) -> None:
        self._latest_msg = msg
        self._received_count += 1
        if self._received_count == 1:
            approx_points = int(msg.width) * int(msg.height)
            self.get_logger().info(
                f"Received first cloud on {self._topic} (frame={msg.header.frame_id}, approx_points={approx_points})."
            )

    def save_snapshot(self, reason: str) -> None:
        if self._saved:
            return
        self._saved = True

        if self._latest_msg is None:
            self.get_logger().warning(
                f"Shutdown reason={reason}, but no cloud was received on {self._topic}; nothing saved."
            )
            return

        try:
            xyz, rgb, intensity = pointcloud_to_arrays(self._latest_msg)
            write_binary_ply(self._output_path, xyz, rgb, intensity)
            metadata_path = self._output_path.parent / f"{self._output_path.stem}.json"
            metadata = {
                "topic": self._topic,
                "frame_id": self._latest_msg.header.frame_id,
                "stamp_sec": int(self._latest_msg.header.stamp.sec),
                "stamp_nanosec": int(self._latest_msg.header.stamp.nanosec),
                "received_messages": self._received_count,
                "saved_points": int(xyz.shape[0]),
                "has_rgb": bool(rgb is not None),
                "has_intensity": bool(intensity is not None),
                "ply_path": str(self._output_path),
            }
            metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
            self.get_logger().info(
                f"Saved latest cloud ({xyz.shape[0]} points) to {self._output_path} and {metadata_path}."
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed saving pointcloud snapshot: {exc}")


def main() -> int:
    args = parse_args()
    rclpy.init(args=None)
    node = PointCloudSnapshotSaver(args.topic, args.output)

    def _handle_signal(signum: int, _frame: object) -> None:
        node.get_logger().info(f"Received signal {signum}, shutting down.")
        if rclpy.ok():
            rclpy.shutdown()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_snapshot("shutdown")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
