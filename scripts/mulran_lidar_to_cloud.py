#!/usr/bin/env python3
"""
Accumulates a MulRan dataset sequence's Ouster LiDAR scans into a single,
world-frame point cloud, for mesh reconstruction via lvr2_reconstruct (see
MIGRATION.md, "MulRan dataset work" -- this is step 2 of the validation
runbook, "reconstruct a mesh from the LiDAR scans").

Each Ouster .bin scan (MulRan's raw format: N x 4 float32, x/y/z/intensity,
no header) is in the sensor's own local frame. To build one consistent
map, each scan is transformed sensor -> base -> world using:
  - a fixed base->ouster extrinsic (MulRan's calib_base2outer.txt format,
    x/y/z in meters + roll/pitch/yaw in degrees -- see parse_calib())
  - that scan's own base->world pose, nearest-neighbor-matched by
    timestamp against global_pose.csv (MulRan's own devkit does the same
    nearest-neighbor time matching -- see calib/base2ouster/example_matlab
    /make_submap_using_pose/util/findNnPoseUsingTime.m in the calib zip)

Expected input layout (a MulRan sequence root directory):
  <sequence_root>/Ouster/<stamp_ns>.bin  (or <sequence_root>/sensor_data/Ouster/...)
  <sequence_root>/global_pose.csv

A full sequence can be thousands of 65536-point scans -- accumulating all
of them is usually neither necessary nor fast to reconstruct from, so only
every --stride'th scan (by chronological order, not by count) is used by
default.

Output: a plain ASCII .xyz point cloud (one "x y z" per line), directly
consumable by lvr2_reconstruct (--inputFile ... , ASCII .xyz is one of its
supported formats).

Usage:
  ros2 run radarays_ros mulran_lidar_to_cloud \\
    --input <sequence_root> --calib <calib_base2outer.txt> \\
    --output <cloud.xyz> [--stride 20] [--max-scans 100]
"""

import argparse
import csv
import os
import re
import sys

import numpy as np


def parse_calib(path: str) -> np.ndarray:
    """Parses MulRan's calib_base2*.txt format (one 'key: value (unit)' per
    line for x/y/z in meters and r/p/y in degrees) into a 4x4 base->sensor
    transform matrix."""
    values = {}
    with open(path, "r") as f:
        for line in f:
            match = re.match(r"\s*([xyzrpy]):\s*(-?[\d.]+)", line)
            if match:
                key, value = match.group(1), float(match.group(2))
                # r/p/y and x/y/z each appear once; "y" is ambiguous
                # between the y-position and yaw lines, disambiguated by
                # which has already been seen (y-position always comes
                # before yaw in every real calib file in this dataset).
                if key == "y" and "y" in values:
                    values["yaw"] = value
                else:
                    values[key] = value

    x, y, z = values["x"], values["y"], values["z"]
    roll, pitch, yaw = (
        np.radians(values["r"]), np.radians(values["p"]), np.radians(values["yaw"])
    )

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    # Standard roll-pitch-yaw (Z * Y * X) rotation matrix.
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])

    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = [x, y, z]
    return T


def read_global_pose_csv(path: str):
    """Returns a sorted list of (stamp_ns, 4x4 base->world transform).

    MulRan's global_pose.csv stores raw UTM coordinates (easting/northing
    in the hundreds-of-thousands/millions range) -- fine for the dataset's
    own purposes, but that magnitude eats into float32 precision once fed
    through Gazebo/rmagine (both float32-based), and produces enormous,
    unreadable coordinates in the reconstructed mesh for no benefit.
    Recentered here around the sequence's first pose, so "world" becomes
    "wherever the sequence started" -- must match mulran_radar_to_bag.py's
    own recentering exactly (both read the same file's first row as the
    origin, so they agree without needing to share state)."""
    poses = []
    with open(path, "r") as f:
        reader = csv.reader(f)
        for row in reader:
            row = [c.strip() for c in row if c.strip() != ""]
            if len(row) != 13:
                continue
            stamp_ns = int(row[0])
            vals = [float(v) for v in row[1:]]
            T = np.eye(4)
            T[0, 0:3] = vals[0:3]
            T[0, 3] = vals[3]
            T[1, 0:3] = vals[4:7]
            T[1, 3] = vals[7]
            T[2, 0:3] = vals[8:11]
            T[2, 3] = vals[11]
            poses.append((stamp_ns, T))
    poses.sort(key=lambda p: p[0])

    origin = poses[0][1][0:3, 3].copy()
    for _, T in poses:
        T[0:3, 3] -= origin

    return poses


def nearest_pose(stamp_ns: int, sorted_poses) -> np.ndarray:
    stamps = [p[0] for p in sorted_poses]
    idx = np.searchsorted(stamps, stamp_ns)
    candidates = [i for i in (idx - 1, idx) if 0 <= i < len(sorted_poses)]
    best = min(candidates, key=lambda i: abs(sorted_poses[i][0] - stamp_ns))
    return sorted_poses[best][1]


def find_ouster_dir(sequence_root: str) -> str:
    for candidate in (
        os.path.join(sequence_root, "Ouster"),
        os.path.join(sequence_root, "sensor_data", "Ouster"),
    ):
        if os.path.isdir(candidate):
            return candidate
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input", required=True, help="MulRan sequence root directory")
    parser.add_argument("--calib", required=True, help="Path to calib_base2outer.txt (or equivalent)")
    parser.add_argument("--output", required=True, help="Output .xyz point cloud path")
    parser.add_argument("--stride", type=int, default=20, help="Use every Nth scan (default 20)")
    parser.add_argument("--max-scans", type=int, default=None, help="Cap on number of scans used, after striding")
    args = parser.parse_args()

    ouster_dir = find_ouster_dir(args.input)
    if ouster_dir is None:
        print(f"ERROR: no Ouster scan directory found under {args.input}", file=sys.stderr)
        return 1

    gt_path = os.path.join(args.input, "global_pose.csv")
    if not os.path.isfile(gt_path):
        print(f"ERROR: global_pose.csv not found: {gt_path}", file=sys.stderr)
        return 1

    T_base_ouster = parse_calib(args.calib)
    poses = read_global_pose_csv(gt_path)
    if not poses:
        print(f"ERROR: no valid pose rows found in {gt_path}", file=sys.stderr)
        return 1

    scan_files = sorted(
        (f for f in os.listdir(ouster_dir) if f.endswith(".bin")),
        key=lambda f: int(os.path.splitext(f)[0]),
    )
    if not scan_files:
        print(f"ERROR: no .bin files found in {ouster_dir}", file=sys.stderr)
        return 1

    selected = scan_files[:: args.stride]
    if args.max_scans is not None:
        selected = selected[: args.max_scans]

    print(f"Found {len(scan_files)} Ouster scans, using {len(selected)} (stride={args.stride}).")

    all_points = []
    for fname in selected:
        stamp_ns = int(os.path.splitext(fname)[0])
        raw = np.fromfile(os.path.join(ouster_dir, fname), dtype=np.float32)
        if raw.size % 4 != 0:
            print(f"WARNING: {fname} size not a multiple of 4 floats, skipping", file=sys.stderr)
            continue
        pts = raw.reshape(-1, 4)[:, 0:3]

        T_world_base = nearest_pose(stamp_ns, poses)
        T_world_ouster = T_world_base @ T_base_ouster

        pts_h = np.hstack([pts, np.ones((pts.shape[0], 1), dtype=np.float32)])
        pts_world = (T_world_ouster @ pts_h.T).T[:, 0:3]
        all_points.append(pts_world)

    if not all_points:
        print("ERROR: no points accumulated", file=sys.stderr)
        return 1

    cloud = np.vstack(all_points)
    print(f"Accumulated {cloud.shape[0]} points from {len(selected)} scans.")

    with open(args.output, "w") as f:
        for p in cloud:
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")

    print(f"Wrote point cloud to {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
