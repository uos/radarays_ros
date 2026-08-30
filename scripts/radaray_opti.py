#!/usr/bin/env python3
"""ROS 2 port of the `dev/opti` branch's automatic radar-material optimizer
(radaray_opti.py), adapted for this migration's actual use case: tuning
material properties against a real MulRan-reconstructed mesh
(lvr2_reconstruct's output -- see MIGRATION.md, "MulRan dataset work").

Two real, deliberate differences from the dev/opti original, not just a
mechanical rospy -> rclpy translation:

1. dev/opti's to_param_vec()/vec_to_params() hardcode materials.data[1]
   ("wall") and materials.data[3] ("glass") as an 8-parameter problem --
   that assumes a richly-labeled indoor Gazebo scene with distinct named
   objects (DoorHallway1Glass-mesh, Bookshelf-mesh, etc, see
   config/mulran_kaist02.yaml's object_materials comments). A plain
   point-cloud reconstruction from lvr2_reconstruct has no such per-object
   labeling -- it's one undifferentiated triangle soup, one object id.
   So this port optimizes exactly one material's 4 properties
   (velocity/ambient/diffuse/specular), selected by --material-index, not
   a hardcoded wall+glass split that doesn't correspond to this data.
2. dev/opti's own grid_search(bounds, N=5) over 8 dimensions is 5**8
   ~= 390,000 simulate-and-compare evaluations -- not something ever
   meant to finish, and its own main() never actually calls it with those
   arguments productively (bounds get overridden ad hoc, the function
   below it references an undefined `res`). Replaced with
   scipy.optimize.differential_evolution, a real global optimizer built
   for exactly this shape of problem (black-box, noisy, no gradient,
   bounded, low-to-moderate dimensionality) -- for 4 parameters this
   actually converges in a practical number of evaluations.

Usage (alongside a running radar_simulator or radar_simulator_gpu with
--serve_action, and something publishing real radar images on the sync
topic -- e.g. `ros2 bag play` on a MulRan bag converted by
mulran_radar_to_bag):
  ros2 run radarays_ros radaray_opti \\
    --server-node-name radar_simulator --sync-topic /Navtech/Polar \\
    --material-index 1 --maxiter 30 --popsize 10 \\
    --output /tmp/optimized_params.yaml

--material-index accepts more than one index (e.g. `--material-index 1 2`)
to tune several materials' 4 properties simultaneously in one
differential_evolution run -- proven against testdata/two_walls_test.dae
(see MIGRATION_HANDOFF.md), a synthetic 2-object scene built specifically
because no real available scene has more than one distinctly-labeled
material to exercise this with.
"""

import argparse
import sys
import time

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.optimize import differential_evolution

from radarays_ros.action import GenRadarImage
from radarays_ros.srv import GetRadarParams
from radarays_ros.msg import RadarParams


# velocity/ambient/diffuse/specular bounds, matching dev/opti's own wall
# material bounds (the only ones that were ever uncommented/active there).
PARAM_NAMES = ["velocity", "ambient", "diffuse", "specular"]
PARAM_BOUNDS = [(0.0, 0.3), (0.0, 1.0), (0.0, 1.0), (0.0, 5000.0)]


def to_param_vec(params: RadarParams, material_indices):
    vecs = []
    for idx in material_indices:
        mat = params.materials.data[idx]
        vecs.append([mat.velocity, mat.ambient, mat.diffuse, mat.specular])
    return np.array(vecs).flatten()


def vec_to_params(params_init: RadarParams, param_vec, material_indices) -> RadarParams:
    params_out = params_init
    for i, idx in enumerate(material_indices):
        mat = params_out.materials.data[idx]
        mat.velocity = float(param_vec[4 * i + 0])
        mat.ambient = float(param_vec[4 * i + 1])
        mat.diffuse = float(param_vec[4 * i + 2])
        mat.specular = float(param_vec[4 * i + 3])
    return params_out


def normalized_mutual_information(a: np.ndarray, b: np.ndarray, bins: int = 64) -> float:
    """Normalized mutual information between two grayscale images, in
    [0, 1] (1 = identical joint distribution). Implemented directly on a
    2D joint histogram instead of depending on sklearn (not installed in
    this environment, and not worth adding as a new system dependency for
    one metric) -- this is the standard histogram-based MI estimator, the
    same approach sklearn's mutual_info_score uses internally on a
    contingency table.

    Resizes b to a's shape if they differ -- radar_simulator's own
    n_cells/n_samples params need to be set to match the real sensor's
    resolution for a meaningful comparison (see
    config/mulran_kaist_dyncfg.yaml), but this is a defensive fallback,
    not the primary way shapes are expected to match."""
    if a.shape != b.shape:
        b = cv2.resize(b, (a.shape[1], a.shape[0]), interpolation=cv2.INTER_LINEAR)
    a = a.astype(np.float64).flatten()
    b = b.astype(np.float64).flatten()
    joint_hist, _, _ = np.histogram2d(a, b, bins=bins)
    joint_prob = joint_hist / joint_hist.sum()
    a_prob = joint_prob.sum(axis=1, keepdims=True)
    b_prob = joint_prob.sum(axis=0, keepdims=True)

    nonzero = joint_prob > 0
    mi = np.sum(joint_prob[nonzero] * np.log(joint_prob[nonzero] / (a_prob @ b_prob)[nonzero]))

    def entropy(p):
        p = p[p > 0]
        return -np.sum(p * np.log(p))

    h_a = entropy(a_prob.flatten())
    h_b = entropy(b_prob.flatten())
    denom = (h_a + h_b) / 2.0
    if denom <= 0.0:
        return 0.0
    return float(mi / denom)


class RadarayOpti(Node):
    def __init__(self, server_node_name: str, sync_topic: str):
        super().__init__("radaray_opti")
        # radar_simulator/radar_simulator_gpu advertise these as plain,
        # un-namespaced names (`/gen_radar_image`, `/get_radar_params`) --
        # they are NOT auto-prefixed with the server node's own name the
        # way ROS1's private-namespace convention would have implied.
        # server_node_name is only used in log messages here, kept as a
        # constructor arg (rather than removed) so a future multi-radar
        # setup running each server under an actual ROS 2 namespace can
        # thread that through without changing this class's shape again.
        self.server_node_name = server_node_name
        self.bridge = CvBridge()
        self._real_image = None
        self.create_subscription(Image, sync_topic, self._on_real_image, 10)
        self.action_client = ActionClient(self, GenRadarImage, "gen_radar_image")
        self.params_client = self.create_client(GetRadarParams, "get_radar_params")

    def _on_real_image(self, msg: Image) -> None:
        if self._real_image is None:
            self._real_image = self.bridge.imgmsg_to_cv2(msg)

    def wait_for_real_image(self, timeout_sec: float) -> np.ndarray:
        self.get_logger().info("Waiting for a real radar image on the sync topic...")
        deadline = time.monotonic() + timeout_sec
        while self._real_image is None:
            if time.monotonic() > deadline:
                raise TimeoutError("Timed out waiting for a real radar image -- is a bag playing?")
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info(f"Got real radar image, shape={self._real_image.shape}")
        return self._real_image

    def fetch_initial_params(self) -> RadarParams:
        if not self.params_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                f"get_radar_params service not available (is {self.server_node_name} running with serve_action:=true?)")
        future = self.params_client.call_async(GetRadarParams.Request())
        rclpy.spin_until_future_complete(self, future)
        return future.result().params

    def simulate(self, params: RadarParams, timeout_sec: float = 10.0) -> np.ndarray:
        # A long optimizer run (thousands of goal/result round-trips) can
        # occasionally log "Ignoring unexpected result response... more
        # than one action server" from rclpy itself -- this is expected,
        # benign upstream behavior (rclpy/action/client.py's execute(),
        # confirmed by reading its source), a late-arriving duplicate
        # response for an already-resolved request under real DDS timing,
        # not a bug in this script. Safe to ignore.
        if not self.action_client.wait_for_server(timeout_sec=timeout_sec):
            raise RuntimeError(
                f"gen_radar_image action server not available (is {self.server_node_name} running with serve_action:=true?)")

        goal = GenRadarImage.Goal(params=params)
        send_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=timeout_sec)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("gen_radar_image goal was not accepted")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_sec)
        result_wrapper = result_future.result()
        if result_wrapper is None:
            raise RuntimeError("gen_radar_image goal timed out waiting for a result")

        return self.bridge.imgmsg_to_cv2(result_wrapper.result.polar_image)


def run_optimizer(
    node: RadarayOpti,
    material_indices,
    maxiter: int,
    popsize: int,
    seed,
    score_log_path: str,
):
    real_image = node.wait_for_real_image(timeout_sec=30.0)
    params_init = node.fetch_initial_params()

    param_names = [f"mat{idx}_{name}" for idx in material_indices for name in PARAM_NAMES]
    param_bounds = PARAM_BOUNDS * len(material_indices)

    node.get_logger().info(f"Optimizing materials.data{material_indices} (velocity/ambient/diffuse/specular each)")
    node.get_logger().info(f"Initial value: {to_param_vec(params_init, material_indices)}")

    # Sanity check with the initial (un-optimized) params before spending
    # the optimizer's budget, so a broken pipeline (materials_file with no
    # object_materials pointing at material_indices, action server down,
    # etc) fails fast with a clear score printed, not silently inside the
    # first differential_evolution population.
    baseline_image = node.simulate(params_init)
    baseline_score = normalized_mutual_information(real_image, baseline_image)
    node.get_logger().info(f"Baseline (initial params) score: {baseline_score:.4f}")

    score_log = open(score_log_path, "a")
    iteration = {"count": 0}

    def objective(param_vec):
        params = vec_to_params(params_init, param_vec, material_indices)
        try:
            sim_image = node.simulate(params)
        except (RuntimeError, TimeoutError) as ex:
            node.get_logger().warning(f"simulate() failed, penalizing: {ex}")
            return 1.0  # worst possible score (NMI is in [0, 1], minimized as negative)

        score = -normalized_mutual_information(real_image, sim_image)
        iteration["count"] += 1
        line = f"iter={iteration['count']} params={param_vec.tolist()} score={-score:.4f}"
        score_log.write(line + "\n")
        score_log.flush()
        if iteration["count"] % 5 == 0:
            node.get_logger().info(line)
        return score

    node.get_logger().info(f"Running differential_evolution: maxiter={maxiter}, popsize={popsize}")
    result = differential_evolution(
        objective,
        param_bounds,
        maxiter=maxiter,
        popsize=popsize,
        seed=seed,
        workers=1,  # each eval drives a live action call through this node -- can't parallelize processes
        polish=False,
        disp=True,
    )
    score_log.close()

    best_score = -result.fun
    node.get_logger().info(f"Done. Best score: {best_score:.4f} (baseline was {baseline_score:.4f})")
    node.get_logger().info(f"Best params: {dict(zip(param_names, result.x.tolist()))}")

    return result, baseline_score, param_names


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--server-node-name", default="radar_simulator")
    parser.add_argument("--sync-topic", default="/Navtech/Polar")
    parser.add_argument("--material-index", type=int, nargs="+", default=[1],
                         help="One or more indices into materials.data to optimize simultaneously "
                              "(default [1], the real/non-air material in "
                              "config/mulran_optimizer_materials.yaml). Each index adds its own "
                              "velocity/ambient/diffuse/specular to the optimization problem, e.g. "
                              "`--material-index 1 2` tunes 8 parameters at once.")
    parser.add_argument("--maxiter", type=int, default=30)
    parser.add_argument("--popsize", type=int, default=10)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--output", default="/tmp/radaray_opti_result.yaml")
    parser.add_argument("--score-log", default="/tmp/radaray_opti_scores.txt")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv if argv is None else argv)[1:])

    rclpy.init()
    node = RadarayOpti(args.server_node_name, args.sync_topic)
    try:
        result, baseline_score, param_names = run_optimizer(
            node, args.material_index, args.maxiter, args.popsize, args.seed, args.score_log)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    with open(args.output, "w") as f:
        f.write(f"# radaray_opti.py result -- materials.data{args.material_index}\n")
        f.write(f"# baseline_score: {baseline_score:.4f}\n")
        f.write(f"# best_score: {-result.fun:.4f}\n")
        for name, value in zip(param_names, result.x.tolist()):
            f.write(f"{name}: {value}\n")
    print(f"Wrote result to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
