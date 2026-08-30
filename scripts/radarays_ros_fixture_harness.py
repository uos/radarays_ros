#!/usr/bin/env python3
"""Capture/compare regression harness for radar_simulator itself -- the
first automated test radarays_ros has ever had (see MIGRATION.md,
"radar_simulator regression fixture"). Unlike rmagine_gazebo_plugins'/
radarays_gazebo_plugins' fixtures, there is no Gazebo world here at all:
radar_simulator is a standalone ROS 2 node (map_file/materials_file/TF
params, no gz-sim dependency), so this launches it directly as a
subprocess alongside a static_transform_publisher for the sensor pose,
against a small self-contained synthetic mesh
(testdata/wall_test.ply -- a thin wall, near face at x=5) instead of
depending on a real dataset mesh or another package's world file.
"""

import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


FIXTURE_NAMES = {
    "wall": "wall_fixture.json",
}
MESH_NAMES = {
    "wall": "wall_test.ply",
}
MATERIALS_NAMES = {
    "wall": "wall_test_materials.yaml",
}


def share_root() -> Path:
    return Path(get_package_share_directory("radarays_ros"))


def testdata_path(name: str) -> Path:
    return share_root() / "testdata" / name


def fixture_path(world: str) -> Path:
    return testdata_path(FIXTURE_NAMES[world])


def default_output_path(world: str) -> Path:
    return Path("/tmp") / f"radarays_ros_{world}_capture.json"


def default_log_path(world: str) -> Path:
    return Path("/tmp") / f"radarays_ros_{world}_radar_sim.log"


def ensure_ros_log_dir() -> None:
    log_dir = Path("/tmp") / "radarays_ros_fixture_ros_logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_LOG_DIR", str(log_dir))


def iso_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def stamp_to_float(msg) -> Optional[float]:
    if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
        return None
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def rate_from_stamps(stamps: List[float]) -> float:
    if len(stamps) < 2:
        return 0.0
    deltas = [b - a for a, b in zip(stamps[:-1], stamps[1:]) if b > a]
    if not deltas:
        return 0.0
    return 1.0 / (sum(deltas) / len(deltas))


def summarize_image(msg: Image) -> Dict[str, object]:
    """radar_simulator's /radar/image is a full 360-degree rotating-radar
    polar image (width = azimuth bins, height = range cells) -- unlike
    radarays_gazebo_plugins' fixed-FOV sensor (whose "center column = the
    sensor's boresight" assumption is meaningful there), there is no one
    azimuth column that corresponds to "straight at the target" here: a
    wall placed at a single position is only visible from the columns
    whose azimuth actually points at it, and a naive width//2 column can
    easily be a column facing entirely away from it (confirmed empirically
    against wall_test.ply: column 200 was entirely zero). What IS a robust
    invariant for a single static target is the RANGE (row) of its return
    -- summed across all columns, the row(s) at the target's range show a
    real energy peak regardless of which columns see it."""
    values = list(msg.data)
    nonzero_values = [value for value in values if value != 0]

    width = msg.width if msg.width else 1
    height = msg.height
    step = msg.step if msg.step > 0 else width

    row_energy = []
    if width > 0 and height > 0:
        for row in range(height):
            start = row * step
            row_energy.append(sum(values[start:start + width]))

    peak_row = None
    peak_row_value = 0
    if row_energy:
        peak_row = max(range(len(row_energy)), key=lambda r: row_energy[r])
        peak_row_value = row_energy[peak_row]

    return {
        "sensor_config": {
            "frame_id": msg.header.frame_id,
            "width": msg.width,
            "height": msg.height,
            "encoding": msg.encoding,
            "step": msg.step,
        },
        "latest": {
            "nonzero_count": len(nonzero_values),
            "nonzero_ratio": (len(nonzero_values) / len(values)) if values else 0.0,
            "max_value": max(values) if values else 0,
            "row_energy_peak_row": peak_row,
            "row_energy_peak_value": peak_row_value,
        },
        "probe": {
            "nonzero_count": len(nonzero_values),
            "max_value": max(values) if values else 0,
            "row_energy_peak_row": peak_row,
        },
    }


class TopicCollector(Node):
    def __init__(self) -> None:
        super().__init__("radarays_ros_fixture_collector")
        self.image_messages: List[Dict[str, object]] = []
        self.create_subscription(Image, "radar/image", self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg: Image) -> None:
        summary = summarize_image(msg)
        summary["stamp"] = stamp_to_float(msg)
        self.image_messages.append(summary)


def finalize_capture(world: str, collector: TopicCollector, duration_sec: float, log_path: Path) -> Dict[str, object]:
    image_stamps = [msg["stamp"] for msg in collector.image_messages if msg["stamp"] is not None]
    latest_image = collector.image_messages[-1] if collector.image_messages else None

    peak_row_series = [
        int(msg["probe"]["row_energy_peak_row"]) for msg in collector.image_messages
        if msg["probe"]["row_energy_peak_row"] is not None
    ]
    # A raw stddev here is fragile to a single outlier frame -- confirmed
    # empirically: an occasional startup/timing-jitter frame (TF not fully
    # available yet, or a system briefly starved of CPU by something else
    # entirely -- see MIGRATION.md, "radar_simulator regression fixture",
    # for a real instance of the latter) can read a wildly different
    # row and blow out a plain stddev even though every OTHER frame agrees
    # tightly. Report a majority-consensus metric instead: how many frames
    # land within a few rows of the median. A real regression (the peak
    # systematically shifting) fails this; one-off outlier frames don't.
    peak_row_median = statistics.median(peak_row_series) if peak_row_series else None
    peak_row_consensus_ratio = (
        sum(1 for r in peak_row_series if abs(r - peak_row_median) <= 3) / len(peak_row_series)
        if peak_row_series else 0.0
    )
    peak_row_stddev = statistics.pstdev(peak_row_series) if len(peak_row_series) >= 2 else 0.0

    return {
        "world": world,
        "topics": {"image": "radar/image"},
        "capture": {
            "started_at": iso_now(),
            "duration_sec": duration_sec,
            "gz_log_path": str(log_path),  # kept for schema parity with the other two harnesses
        },
        "image": {
            "message_count": len(collector.image_messages),
            "rate_hz_estimate": rate_from_stamps(image_stamps),
            "latest": latest_image["latest"] if latest_image else None,
            "sensor_config": latest_image["sensor_config"] if latest_image else None,
            "series": {
                "row_energy_peak_row": peak_row_series,
                "row_energy_peak_row_stddev": peak_row_stddev,
                "row_energy_peak_row_median": peak_row_median,
                "row_energy_peak_row_consensus_ratio": peak_row_consensus_ratio,
            },
        },
    }


def launch_pipeline(world: str, log_path: Path):
    """Starts a static_transform_publisher (map -> sensor_test, at
    z=1, matching the sensor height wall_test.ply was authored around)
    and radar_simulator against the synthetic mesh, as two subprocesses.
    Returns (tf_process, radar_process)."""
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_file = log_path.open("w", encoding="utf-8")

    tf_process = subprocess.Popen(
        [
            "ros2", "run", "tf2_ros", "static_transform_publisher",
            "--x", "0", "--y", "0", "--z", "1",
            "--frame-id", "map", "--child-frame-id", "sensor_test",
        ],
        stdout=log_file, stderr=subprocess.STDOUT, preexec_fn=os.setsid,
    )

    radar_process = subprocess.Popen(
        [
            "ros2", "run", "radarays_ros", "radar_simulator", "--ros-args",
            "-p", f"map_file:={testdata_path(MESH_NAMES[world])}",
            "-p", "map_frame:=map",
            "-p", "sensor_frame:=sensor_test",
            "-p", f"materials_file:={testdata_path(MATERIALS_NAMES[world])}",
            "-p", "material_id_air:=0",
            "-p", "object_materials:=[1]",
        ],
        stdout=log_file, stderr=subprocess.STDOUT, preexec_fn=os.setsid,
    )
    return tf_process, radar_process


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5.0)


def capture_world(world: str, output_path: Path, duration_sec: float, timeout_sec: float) -> int:
    log_path = default_log_path(world)
    tf_process, radar_process = launch_pipeline(world, log_path)
    ensure_ros_log_dir()
    rclpy.init(args=None)
    collector = TopicCollector()

    try:
        # The first several /radar/image messages are reliably a startup
        # transient (row_energy_peak_row == 0 -- confirmed empirically:
        # under ctest, TF/mesh warm-up took up to 6 frames before settling
        # at the real 117-118 peak; under an isolated manual run it was
        # sometimes just 1 frame). Wait past this before starting the timed
        # capture at all, then discard whatever warm-up messages arrived
        # during the wait -- same shape as the other two harnesses' "wait
        # for scan AND points AND image" multi-condition readiness check,
        # just for this fixture's own warm-up signal instead.
        first_deadline = time.monotonic() + timeout_sec
        while time.monotonic() < first_deadline:
            if radar_process.poll() is not None:
                print(f"radar_simulator exited early with code {radar_process.returncode}. See {log_path}", file=sys.stderr)
                return 1
            rclpy.spin_once(collector, timeout_sec=0.1)
            if collector.image_messages and collector.image_messages[-1]["probe"]["row_energy_peak_row"]:
                break
        else:
            print(f"Timed out waiting for a warmed-up radar/image (peak row stayed 0). See {log_path}", file=sys.stderr)
            return 1

        collector.image_messages.clear()
        capture_end = time.monotonic() + duration_sec
        while time.monotonic() < capture_end:
            if radar_process.poll() is not None:
                print(f"radar_simulator exited during capture with code {radar_process.returncode}. See {log_path}", file=sys.stderr)
                return 1
            rclpy.spin_once(collector, timeout_sec=0.1)

        result = finalize_capture(world, collector, duration_sec, log_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"Wrote capture for {world} to {output_path}")
        print(f"radar_simulator log saved to {log_path}")
        return 0
    finally:
        try:
            collector.destroy_node()
        finally:
            rclpy.shutdown()
            stop_process(radar_process)
            stop_process(tf_process)


def load_json(path: Path) -> Dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def ensure(condition: bool, message: str, failures: List[str]) -> None:
    if not condition:
        failures.append(message)


def compare_capture(world: str, result_path: Path) -> int:
    fixture = load_json(fixture_path(world))
    result = load_json(result_path)
    failures: List[str] = []

    ensure(result.get("world") == world, f"world mismatch: expected {world}, got {result.get('world')}", failures)

    image = result.get("image") or {}
    image_latest = image.get("latest") or {}
    expected_image = fixture["expected"]["image"]

    ensure(image.get("message_count", 0) >= expected_image["min_messages"],
           f"image message_count {image.get('message_count', 0)} < {expected_image['min_messages']}",
           failures)
    # rate_hz_estimate is deliberately not checked here -- radar_simulator's
    # /radar/image messages don't carry a real header stamp (confirmed
    # empirically: rate_from_stamps() reads 0.0 every capture even with 25+
    # messages/5s), unlike the Gazebo-plugin sensors' scan/points messages.
    # message_count above is the real throughput check for this fixture.
    ensure(image_latest.get("nonzero_count", 0) >= expected_image["latest_nonzero_count_min"],
           f"image nonzero_count {image_latest.get('nonzero_count', 0)} < {expected_image['latest_nonzero_count_min']}",
           failures)
    ensure(image_latest.get("max_value", 0) >= expected_image["latest_max_value_min"],
           f"image max_value {image_latest.get('max_value', 0)} < {expected_image['latest_max_value_min']}",
           failures)

    sensor_config = image.get("sensor_config") or {}
    ensure(sensor_config.get("width") == expected_image["width"],
           f"image width {sensor_config.get('width')} != {expected_image['width']}",
           failures)

    # The falsifiable check: the row (range cell) with peak summed energy
    # across all azimuth columns must fall in the window matching the
    # wall's known real-world distance -- not just "some nonzero pixels
    # somewhere", which a broken material/TF/mesh-loading path could still
    # produce without ever actually returning a real echo from the wall's
    # actual range.
    if "row_energy_target_window" in expected_image:
        window = expected_image["row_energy_target_window"]
        peak_row = image_latest.get("row_energy_peak_row")
        ensure(peak_row is not None and window["row_min"] <= peak_row <= window["row_max"],
               f"image row_energy_peak_row {peak_row} not in expected range "
               f"[{window['row_min']}, {window['row_max']}] (no strong return near the wall's known range)",
               failures)

    ensure(image.get("series", {}).get("row_energy_peak_row_consensus_ratio", 0.0) >= expected_image["row_energy_peak_row_consensus_ratio_min"],
           f"row_energy_peak_row consensus ratio {image.get('series', {}).get('row_energy_peak_row_consensus_ratio', 0.0):.3f} "
           f"< {expected_image['row_energy_peak_row_consensus_ratio_min']:.3f} (static scene, expected the vast majority "
           f"of frames to agree on the peak row -- see finalize_capture()'s comment on why this uses a "
           f"majority-consensus check instead of a raw stddev)",
           failures)

    if failures:
        print(f"Comparison against {fixture_path(world)} failed:", file=sys.stderr)
        for failure in failures:
            print(f"  - {failure}", file=sys.stderr)
        return 1

    print(f"{world} capture matches expected fixture {fixture_path(world)}")
    return 0


def parse_args(argv: List[str]) -> argparse.Namespace:
    command_name = Path(argv[0]).name
    parser = argparse.ArgumentParser(prog=command_name)
    parser.add_argument("world", choices=sorted(FIXTURE_NAMES.keys()))
    parser.add_argument("--output", type=Path, help="Output path for capture JSON")
    parser.add_argument("--result", type=Path, help="Result JSON to compare")
    parser.add_argument("--duration", type=float, help="Capture duration in seconds")
    parser.add_argument("--timeout", type=float, default=20.0, help="Timeout waiting for the first image")
    return parser.parse_args(argv[1:])


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    command_name = Path(argv[0]).name

    if "capture" in command_name:
        output = args.output or default_output_path(args.world)
        fixture = load_json(fixture_path(args.world))
        duration = args.duration or fixture["capture_defaults"]["duration_sec"]
        return capture_world(args.world, output, float(duration), float(args.timeout))

    if "compare" in command_name:
        result = args.result or default_output_path(args.world)
        if not result.exists():
            print(f"Result file not found: {result}", file=sys.stderr)
            return 1
        return compare_capture(args.world, result)

    print(f"Unsupported entrypoint name: {command_name}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
