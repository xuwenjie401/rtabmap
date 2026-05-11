#!/usr/bin/env python3
import argparse
import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo


class OakStereoCameraInfoFix(Node):
    def __init__(self, args):
        super().__init__("oak_stereo_camera_info_fix")

        self._baseline = abs(args.baseline)
        self._fallback_baseline = abs(args.fallback_baseline)

        sub_qos = QoSProfile(depth=10)
        sub_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        pub_qos = QoSProfile(depth=10)
        pub_qos.reliability = QoSReliabilityPolicy.RELIABLE

        self._left_pub = self.create_publisher(CameraInfo, args.left_out, pub_qos)
        self._right_pub = self.create_publisher(CameraInfo, args.right_out, pub_qos)

        self.create_subscription(CameraInfo, args.left_in, self._left_cb, sub_qos)
        self.create_subscription(CameraInfo, args.right_in, self._right_cb, sub_qos)

        self.get_logger().info(
            "Fixing OAK stereo camera_info: "
            f"{args.left_in}->{args.left_out}, {args.right_in}->{args.right_out}, "
            f"baseline={self._baseline or self._fallback_baseline:.6f} m"
        )

    def _update_baseline_from_msg(self, msg):
        p = msg.p
        if self._baseline == 0.0 and len(p) > 3 and p[0] != 0.0 and p[3] != 0.0:
            self._baseline = abs(p[3] / p[0])

    def _effective_baseline(self):
        return self._baseline if self._baseline > 0.0 else self._fallback_baseline

    def _left_cb(self, msg):
        self._update_baseline_from_msg(msg)
        out = copy.deepcopy(msg)
        p = list(out.p)
        p[3] = 0.0
        out.p = p
        self._left_pub.publish(out)

    def _right_cb(self, msg):
        self._update_baseline_from_msg(msg)
        out = copy.deepcopy(msg)
        p = list(out.p)
        fx = p[0] if p[0] != 0.0 else out.k[0]
        p[3] = -abs(fx) * self._effective_baseline()
        out.p = p
        self._right_pub.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--left-in", default="/oak/left/camera_info")
    parser.add_argument("--right-in", default="/oak/right/camera_info")
    parser.add_argument("--left-out", default="/oak/left/camera_info_rtabmap")
    parser.add_argument("--right-out", default="/oak/right/camera_info_rtabmap")
    parser.add_argument("--baseline", type=float, default=0.074568)
    parser.add_argument("--fallback-baseline", type=float, default=0.074568)
    args = parser.parse_args()

    rclpy.init()
    node = OakStereoCameraInfoFix(args)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
