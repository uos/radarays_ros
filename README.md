# RadaRays - ROS package

Rotating FMCW radar simulation based on ray-tracing. This package contains all the tools that has been used to simulate radar data and compare it to real sensor data.

## Migration Status

This package is being migrated to ROS 2 Jazzy / `ament_cmake`. See
[MIGRATION.md](MIGRATION.md) for details.

Current state:

- message/service/action interfaces (`RadarMaterial(s)`, `RadarModel`,
  `RadarParams`, `GetRadarParams`, `GenRadarImage`): built and verified;
  `GetRadarParams`/`GenRadarImage` are also now served on both
  `radar_simulator` and `radar_simulator_gpu` -- run either with
  `-p serve_action:=true` (see MIGRATION.md, "`GetRadarParams`/
  `GenRadarImage` served" and "Known Gaps vs the ROS 1 (`noetic`)
  Branch" #5 for a real, pre-existing GPU-only bug this surfaced:
  `radar_simulator_gpu` needs a real `materials_file` configured or it
  crashes, regardless of `serve_action`)
- `radar_simulator` (CPU/Embree only): ported and runtime-verified against
  a real mesh with a real TF chain — `ros2 run radarays_ros radar_simulator
  --ros-args --params-file <params.yaml>` (see MIGRATION.md for the
  parameter shape, including how the materials list moved from a ROS 1
  `XmlRpc` param to a plain YAML file referenced by a `materials_file`
  parameter). Also has its first-ever automated regression test,
  `radarays_ros_fixture_wall` (`colcon test --packages-select
  radarays_ros`) -- a self-contained synthetic-mesh fixture, no Gazebo
  involved. See MIGRATION.md, "`radar_simulator` regression fixture", for
  the real design mistakes this surfaced and fixed (a wrong "boresight
  column" assumption for a full 360-degree sweep, a startup-transient
  false failure, and a leaked-process cross-talk bug in this session's own
  testing, not the code).
- all ~30 `dynamic_reconfigure`/`RadarModel.cfg` fields: live-tunable
  ROS 2 parameters (`ros2 param set /radar_simulator <name> <value>`)
- the pure radar-physics math (`Ray`/`DirectedWave`, fresnel,
  back-reflection shader, cone sampling, denoising, Perlin noise) is a
  separately-built, exported target, `radarays_core` — no ROS dependency,
  just `rmagine::core`. `radarays_gazebo_plugins` depends on it directly
  now instead of maintaining its own copy (see MIGRATION.md, "Two
  independent physics copies")
- `ray_reflection_test` (reflection-path debug visualizer, publishes a
  `visualization_msgs/msg/Marker` LINE_LIST): ported and runtime-verified
  the same way as `radar_simulator`
- `radar_simulator_gpu` (GPU/OptiX): ported and runtime-verified against
  the real GPU and the same mesh/TF as `radar_simulator` — `ros2 run
  radarays_ros radar_simulator_gpu --ros-args --params-file <params.yaml>`.
  Needs `rmagine` built with a working `rmagine::optix` (see
  `rmagine_gazebo_plugins/README.md`, "OptiX / GPU Harmonic port", for the
  SDK/driver ABI notes); builds automatically alongside `radar_simulator`
  when that's available, no separate flag needed
- `radarays_gpu_core` (pure CUDA kernels, no ROS/`Radar` coupling --
  `move_waves`/`signal_shader`/`fresnel_split`/`draw_signals` plus the
  plain-POD `RadarMaterial`) is a separately-built, exported target, same
  split rationale as `radarays_core`. `radarays_gazebo_plugins`'s
  `radarays_optix_sensor_system` depends on it directly (see its own
  MIGRATION.md) for the Gazebo-side GPU radar path
- not ported yet: `mesh_publisher` (already unbuilt pre-migration)
- `launch/mulran_sim.launch.py`: ROS 2 port of the ROS 1 `mulran_sim.launch`
  (RViz/`gui` intentionally left out, see MIGRATION.md); `ros2 launch
  radarays_ros mulran_sim.launch.py meshfile:=... bagfile:=... calib:=...`
  runs the whole MulRan runbook below as one command. Runtime-verified
  against real MulRan data, and surfaced + fixed a real thread-safety bug
  in `RadarCPU`'s per-thread simulator cache that crashed
  `radar_simulator` under sustained real message throughput (see
  MIGRATION.md, "Known Gaps..." #1)
- example parameter YAMLs (`cfg/mulran_kaist_dyncfg*.yaml`): converted from
  ROS 1 `dynamic_reconfigure` pickle-format to ROS 2 `--params-file`
  format, in place
- `radaray_opti` (automatic material-property optimizer, ported from the
  `dev/opti` branch): adapted for a real MulRan-reconstructed mesh (tunes
  one material's 4 properties, not the original's hardcoded wall+glass
  split meant for a differently-labeled scene) and a real global optimizer
  (`scipy.optimize.differential_evolution`, replacing the original's
  infeasible 5⁸-evaluation grid search). Runtime-verified with a real,
  completed 2460-evaluation run (40 generations × population 60) against
  a real reconstructed mesh and real MulRan bag data -- clean staircase
  convergence, baseline score 0.0025 → best 0.0031, held stable for the
  final 17/40 generations. See MIGRATION_HANDOFF.md, "Phase 3: Automatic
  material optimizer", for the full writeup and convergence numbers.
  `ros2 run
  radarays_ros radaray_opti --server-node-name radar_simulator
  --sync-topic /Navtech/Polar --material-index 1 --maxiter 30
  --popsize 10 --output <result.yaml>` alongside a `radar_simulator
  --serve_action:=true` and a real bag playing
- MulRan dataset validation (the "MulRan" section below): run for real
  against the free ParkingLot sample — mesh reconstructed from its real
  LiDAR scans (`lvr2_reconstruct`), real radar images + ground truth
  converted to a bag, `radar_simulator` run against the reconstructed mesh
  with TF driven by the real ground truth, and simulated vs. real radar
  images compared directly. See MIGRATION.md, "Real MulRan validation —
  first run against the ParkingLot sample", for the two real bugs this
  surfaced (an executor-reentrancy crash and a materials-configuration
  pitfall) and the actual comparison numbers. Real material calibration
  and a full registered sequence remain open for more rigor.

Everything in this README past this point describes the ROS 1 / catkin
workflow and hasn't been re-verified against the ROS 2 build (paths,
launch files, and the `mesh_tools` visualization dependency are all still
ROS 1-only).

## Requirements (ROS 1 / catkin, not yet ported)

- ROS noetic
- Rmagine for ray tracing on CPU or NVIDIA GPUs (RTX or non-RTX): https://github.com/uos/rmagine
- (optional) mesh_tools fork for visualizations: [https://github.com/aock/mesh_tools](https://github.com/aock/mesh_tools)

Mainly tested on:
- Ubuntu 20.04
- AMD Ryzen 7 3800X
- NVIDIA GeForce RTX 2070 SUPER

And others: see paper (preprint is coming soon).

## MulRan

Demonstration of simulating rotating FMCW radar data in large-scale triangle meshes from MulRan datasets. We reconstructed a mesh of the sequences DCC, and KAIST, and added a localization frame to each bag file using MICP-L as described here: [https://github.com/aock/micp_experiments/tree/main/micp_mulran](https://github.com/aock/micp_experiments/tree/main/micp_mulran). The final bag files are piped to RadaRays. The simulation given the localization is started by


```console
roslaunch radarays_ros mulran_sim.launch gui:=false
```

The simulated data is published on topic `/radar/image`. To enable the visualization you will need [https://github.com/aock/mesh_tools](https://github.com/aock/mesh_tools). Then start the same launch file:

```console
roslaunch radarays_ros mulran_sim.launch gui:=true
```

The results should look as follows:

![BRDF](dat/kaist02_radarays_papercolor.png)

Left is the real radar polar image from the MulRan datasets. Right is the RadaRays simulated polar image.

Videos: https://youtube.com/playlist?list=PL9wBuzh6ev06rcl8ksSnxRtv-7jAkR_wx

## BRDF

Easily customize the models reflection parameters using the following interactive tool:

```console
rosrun radaray_ros radarays_snell_fresnel_brdf.py
```

![BRDF](dat/radarays_snell_fresnel_brdf.png)

## Citation

Please reference the following papers when using RadaRays in your scientific work.

```bib
@article{mock2025radarays,
  author={Mock, Alexander and Magnusson, Martin and Hertzberg, Joachim},
  journal={IEEE Robotics and Automation Letters}, 
  title={RadaRays: Real-Time Simulation of Rotating FMCW Radar for Mobile Robotics via Hardware-Accelerated Ray Tracing}, 
  year={2025},
  volume={10},
  number={3},
  pages={2470-2477},
  doi={10.1109/LRA.2025.3531689}
}
```

The paper is available on [IEEE Xplore](https://ieeexplore.ieee.org/abstract/document/10845807) and as preprint on [arXiv](https://arxiv.org/abs/2310.03505).

## Radarays Plugins

- Gazebo: [`https://github.com/uos/radarays_gazebo_plugins`](https://github.com/uos/radarays_gazebo_plugins)

## Roadmap

- [ ] Put every non-ROS component to C++ only repository ([`https://github.com/uos/radarays`](https://github.com/uos/radarays)) for better reusage for non-ROS projects.


## Development

There are currently the following active development tracks on individual branches:

1. [dev/flex](https://github.com/uos/radarays_ros/tree/dev/flex)
    - flexible reflection models for rapid prototyping (CPU-only)
    - Video for Cook-Torrance reflection model: https://www.youtube.com/watch?v=PI5j87NQzmk
2. [dev/opti](https://github.com/uos/radarays_ros/tree/dev/opti)
    - automatic material property optimization using the standard RadaRays simulation model

Individual components of these development branches are transferred to the main branch after extensive testing.