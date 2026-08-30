#!/usr/bin/env python3
"""
Converts a MulRan dataset sequence's radar-polar images and ground-truth
poses into a ros2 bag, for comparing radarays_ros's simulated /radar/image
against real recorded radar data.

MulRan (https://sites.google.com/view/mulran-pr) isn't distributed as a
rosbag -- it's raw per-sensor files, normally replayed with the ROS1/Qt GUI
tool file_player_mulran (https://github.com/RPM-Robotics-Lab/file_player_mulran).
That tool also handles LiDAR/IMU/GPS and live playback controls, none of
which radar-vs-simulation comparison needs. This script instead does only
the one relevant piece of file_player_mulran's own ROSThread::SaveRosbag()
(radar-polar PNGs + global_pose.csv ground truth), reimplemented directly
in ROS 2 -- see MIGRATION_HANDOFF.md, "MulRan dataset work", for why.

Expected input layout (a MulRan sequence root directory):
  <sequence_root>/sensor_data/radar/polar/<stamp_ns>.png   (mono8 polar images)
  <sequence_root>/global_pose.csv                          (ground truth poses)

The free "ParkingLot" sample sequence (no registration required, unlike
the full DCC/KAIST/Riverside/Sejong sequences) uses a flatter
<sequence_root>/polar/<stamp_ns>.png layout instead -- both are tried.

global_pose.csv format (KITTI-style, one line per pose): 13 comma-separated
fields -- stamp_ns, then a row-major 3x4 [R|t] matrix (r00,r01,r02,tx,
r10,r11,r12,ty,r20,r21,r22,tz). This matches the field layout
file_player_mulran's own SaveRosbag() parses.

The raw translation is in UTM coordinates (hundreds-of-thousands/millions
range) -- recentered here around the sequence's first pose before writing
to /gt, so downstream consumers (Gazebo/rmagine, both float32-based) don't
lose precision for no benefit. mulran_lidar_to_cloud.py recenters its
reconstructed mesh the same way, from the same file's first row, so the
two agree without needing to share any state.

Output: a ros2 bag (sqlite3) with:
  /Navtech/Polar  sensor_msgs/msg/Image   (mono8, one per radar-polar PNG)
  /gt             nav_msgs/msg/Odometry   (one per global_pose.csv row)

Usage:
  ros2 run radarays_ros mulran_radar_to_bag --input <sequence_root> --output <bag_dir>
"""

import argparse
import csv
import os
import sys

import cv2
import numpy as np

import rclpy.time
from rclpy.serialization import serialize_message
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

import rosbag2_py


def make_image_msg(png_path: str, stamp_ns: int) -> Image:
    img = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise RuntimeError(f"Failed to read radar-polar image: {png_path}")

    msg = Image()
    msg.header = Header()
    msg.header.stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
    msg.header.frame_id = "radar_polar"
    msg.height = img.shape[0]
    msg.width = img.shape[1]
    msg.encoding = "mono8"
    msg.is_bigendian = 0
    msg.step = img.shape[1]
    msg.data = img.tobytes()
    return msg


def make_odometry_msg(row: "list[str]") -> Odometry:
    # row: [stamp, r00,r01,r02,tx, r10,r11,r12,ty, r20,r21,r22,tz]
    if len(row) != 13:
        raise ValueError(f"Expected 13 fields in global_pose.csv row, got {len(row)}: {row}")

    stamp_ns = int(row[0])
    vals = [float(v) for v in row[1:]]
    R = np.array(vals[0:3] + vals[4:7] + vals[8:11]).reshape(3, 3)
    t = np.array([vals[3], vals[7], vals[11]])

    # Rotation matrix -> quaternion (Shepperd's method).
    trace = np.trace(R)
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    msg = Odometry()
    msg.header = Header()
    msg.header.stamp = rclpy.time.Time(nanoseconds=stamp_ns).to_msg()
    msg.header.frame_id = "world"
    msg.child_frame_id = "gt"
    msg.pose.pose.position.x = float(t[0])
    msg.pose.pose.position.y = float(t[1])
    msg.pose.pose.position.z = float(t[2])
    msg.pose.pose.orientation.x = float(qx)
    msg.pose.pose.orientation.y = float(qy)
    msg.pose.pose.orientation.z = float(qz)
    msg.pose.pose.orientation.w = float(qw)
    return msg, stamp_ns


def read_global_pose_csv(path: str):
    rows = []
    with open(path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            row = [c.strip() for c in row if c.strip() != ""]
            if len(row) != 13:
                continue
            rows.append(row)

    # MulRan's global_pose.csv stores raw UTM coordinates
    # (hundreds-of-thousands/millions range) -- recentered around the
    # sequence's first pose so downstream consumers (Gazebo/rmagine, both
    # float32-based) don't eat precision loss for no benefit. Must match
    # mulran_lidar_to_cloud.py's own recentering exactly -- both derive it
    # from this same file's first row, so they agree without sharing state.
    if rows:
        origin = (float(rows[0][4]), float(rows[0][8]), float(rows[0][12]))
        for row in rows:
            row[4] = str(float(row[4]) - origin[0])
            row[8] = str(float(row[8]) - origin[1])
            row[12] = str(float(row[12]) - origin[2])

    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="MulRan sequence root directory")
    parser.add_argument("--output", required=True, help="Output ros2 bag directory (must not exist yet)")
    args = parser.parse_args()

    radar_dir_candidates = [
        os.path.join(args.input, "sensor_data", "radar", "polar"),
        os.path.join(args.input, "polar"),
    ]
    radar_dir = next((d for d in radar_dir_candidates if os.path.isdir(d)), None)
    gt_path = os.path.join(args.input, "global_pose.csv")

    if radar_dir is None:
        print(f"ERROR: no radar-polar directory found, tried: {radar_dir_candidates}", file=sys.stderr)
        return 1
    if not os.path.isfile(gt_path):
        print(f"ERROR: global_pose.csv not found: {gt_path}", file=sys.stderr)
        return 1

    radar_files = sorted(
        (f for f in os.listdir(radar_dir) if f.endswith(".png")),
        key=lambda f: int(os.path.splitext(f)[0]),
    )
    if not radar_files:
        print(f"ERROR: no .png files found in {radar_dir}", file=sys.stderr)
        return 1

    gt_rows = read_global_pose_csv(gt_path)
    if not gt_rows:
        print(f"ERROR: no valid pose rows found in {gt_path}", file=sys.stderr)
        return 1

    print(f"Found {len(radar_files)} radar-polar images, {len(gt_rows)} ground-truth poses.")

    writer = rosbag2_py.SequentialWriter()
    storage_options = rosbag2_py.StorageOptions(uri=args.output, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    writer.open(storage_options, converter_options)

    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=0, name="/Navtech/Polar", type="sensor_msgs/msg/Image", serialization_format="cdr"
        )
    )
    writer.create_topic(
        rosbag2_py.TopicMetadata(
            id=1, name="/gt", type="nav_msgs/msg/Odometry", serialization_format="cdr"
        )
    )

    n_radar_written = 0
    for fname in radar_files:
        stamp_ns = int(os.path.splitext(fname)[0])
        try:
            msg = make_image_msg(os.path.join(radar_dir, fname), stamp_ns)
        except RuntimeError as e:
            print(f"WARNING: {e}", file=sys.stderr)
            continue
        writer.write("/Navtech/Polar", serialize_message(msg), stamp_ns)
        n_radar_written += 1

    n_gt_written = 0
    for row in gt_rows:
        msg, stamp_ns = make_odometry_msg(row)
        writer.write("/gt", serialize_message(msg), stamp_ns)
        n_gt_written += 1

    print(f"Wrote {n_radar_written} /Navtech/Polar messages, {n_gt_written} /gt messages to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
