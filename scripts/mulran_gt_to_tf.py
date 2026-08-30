#!/usr/bin/env python3
"""
Bridges a mulran_radar_to_bag.py bag's /gt (nav_msgs/Odometry, ground-truth
base_link pose) into a live TF tree radar_simulator can consume, plus a
static extrinsic for the radar sensor itself -- see MIGRATION.md, "MulRan
dataset work", runbook step 4 ("drive TF from the ground truth").

Publishes:
  map -> base_link   (dynamic, one broadcast per /gt message)
  base_link -> radar (static, from a MulRan calib_base2radar.txt file)

Usage (alongside `ros2 bag play <bag>` and radar_simulator/radar_simulator_gpu
with -p map_frame:=map -p sensor_frame:=radar -p sync_topic:=/Navtech/Polar):
  ros2 run radarays_ros mulran_gt_to_tf --calib <calib_base2radar.txt>
"""

import argparse
import re
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


def parse_calib_rpy(path: str):
    """Parses MulRan's calib_base2*.txt format (one 'key: value (unit)' per
    line for x/y/z in meters and r/p/y in degrees) into (x, y, z, roll,
    pitch, yaw) with angles in radians. Same format/parsing rule as
    mulran_lidar_to_cloud.py's parse_calib() -- duplicated rather than
    imported since these are standalone installed scripts, not a shared
    importable module."""
    values = {}
    with open(path, "r") as f:
        for line in f:
            match = re.match(r"\s*([xyzrpy]):\s*(-?[\d.]+)", line)
            if match:
                key, value = match.group(1), float(match.group(2))
                if key == "y" and "y" in values:
                    values["yaw"] = value
                else:
                    values[key] = value

    return (
        values["x"], values["y"], values["z"],
        np.radians(values["r"]), np.radians(values["p"]), np.radians(values["yaw"]),
    )


def quat_from_rpy(roll: float, pitch: float, yaw: float):
    cr, sr = np.cos(roll / 2.0), np.sin(roll / 2.0)
    cp, sp = np.cos(pitch / 2.0), np.sin(pitch / 2.0)
    cy, sy = np.cos(yaw / 2.0), np.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class GtToTf(Node):
    def __init__(self, calib_path: str, map_frame: str, base_frame: str, sensor_frame: str):
        super().__init__("mulran_gt_to_tf")
        self.map_frame = map_frame
        self.base_frame = base_frame

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_extrinsic(calib_path, base_frame, sensor_frame)

        self.sub = self.create_subscription(Odometry, "/gt", self._on_gt, 10)

    def _publish_static_extrinsic(self, calib_path: str, base_frame: str, sensor_frame: str):
        x, y, z, roll, pitch, yaw = parse_calib_rpy(calib_path)
        qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)

        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = base_frame
        msg.child_frame_id = sensor_frame
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = z
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        self.static_broadcaster.sendTransform(msg)
        self.get_logger().info(
            f"Published static {base_frame} -> {sensor_frame}: "
            f"xyz=({x:.3f},{y:.3f},{z:.3f}) rpy_deg=({np.degrees(roll):.3f},{np.degrees(pitch):.3f},{np.degrees(yaw):.3f})"
        )

    def _on_gt(self, msg: Odometry):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--calib", required=True, help="Path to calib_base2radar.txt (or calib_base2outer.txt)")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--sensor-frame", default="radar")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])

    rclpy.init()
    node = GtToTf(args.calib, args.map_frame, args.base_frame, args.sensor_frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
