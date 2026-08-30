# Quickstart: seeing this actually work

Read `ARCHITECTURE.md` first if you haven't — this assumes you know what
each of the 4 packages does. Everything below assumes you're in the
workspace root with it built and sourced:

```bash
cd ~/Documents/Github/radarays_ws
colcon build --symlink-install
source install/setup.bash
```

(If Embree raytracing fails at runtime with a missing `libiomp5.so`, see
`MIGRATION_HANDOFF.md`'s "Known Environment Issues" — one-time `apt`
fix.)

## 1. Fastest sanity check: run the fixture harnesses (no GUI, ~10s each)

These are the actual regression tests, and also the fastest way to prove
the whole pipeline still works end to end:

```bash
colcon test --packages-select rmagine_gazebo_plugins radarays_gazebo_plugins radarays_ros
colcon test-result --all --verbose
```

All green means: Embree scene mirroring, the CPU radar sensor (static +
2 dynamic scenarios), and the standalone `radar_simulator` are all
producing real, correct, non-degenerate output right now, on this machine.

## 2. Watch a radar image live, in a real Gazebo Harmonic world

```bash
ros2 launch radarays_gazebo_plugins gz_static_radar_cpu.launch.py
```

In another terminal:

```bash
ros2 run rqt_image_view rqt_image_view /radar/image
```

You'll see a Gazebo window with a simple world, and a rotating-radar polar
image (azimuth × range) updating live in `rqt_image_view`.

To see a *moving* object change the radar returns in real time:

```bash
ros2 launch radarays_gazebo_plugins gz_dynamic_radar_cpu_multi.launch.py
```

This world has two independently moving/rotating objects, each with its
own material — watch their returns shift as they move.

To use the GPU/OptiX sensor instead of CPU/Embree, point the same launch
file at the GPU world (needs an NVIDIA GPU + the OptiX SDK, see
`MIGRATION_HANDOFF.md` #18):

```bash
ros2 launch radarays_gazebo_plugins gz_static_radar_cpu.launch.py \
  world:=$(ros2 pkg prefix radarays_gazebo_plugins)/share/radarays_gazebo_plugins/worlds/gz_static_radar_gpu.sdf
```

## 3. Tune the radar live, while it's running

With either world above still running:

```bash
ros2 param list /radarays_embree_sensor_system   # or /radarays_optix_sensor_system on GPU
ros2 param set /radarays_embree_sensor_system energy_max 200.0
```

The next `/radar/image` frame reflects the change immediately — no
restart. There are ~24 of these live-tunable physics parameters (Fresnel
reflection strength, cone sampling spread, Perlin noise clutter, etc).

## 4. Run the radar standalone, no Gazebo at all

This is the path used for dataset validation and the material optimizer —
just a mesh file + a materials YAML + ROS 2:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 1 --frame-id map --child-frame-id sensor_test &

ros2 run radarays_ros radar_simulator --ros-args \
  -p map_file:=$(ros2 pkg prefix radarays_ros)/share/radarays_ros/testdata/wall_test.ply \
  -p map_frame:=map -p sensor_frame:=sensor_test \
  -p materials_file:=$(ros2 pkg prefix radarays_ros)/share/radarays_ros/testdata/wall_test_materials.yaml \
  -p material_id_air:=0 -p object_materials:=[1]
```

```bash
ros2 topic echo /radar/image --field height,width   # or rqt_image_view again
```

This loads a single thin wall at a known range and radiates a real radar
image from it — the simplest possible way to see the physics in isolation,
change one material property, and see the image change, without any
Gazebo/world-file overhead.

## 5. Auto-tune material parameters against a "real" image

`radaray_opti.py` searches for material values that make the simulated
image match a real one (normalized mutual information as the score). To
see it work end to end without needing real MulRan data, use the
synthetic 2-material scene built for exactly this
(`testdata/two_walls_test.dae` / `two_walls_test_materials.yaml` — see
`MIGRATION_HANDOFF.md`'s multi-material optimizer verification writeup for
how this was validated):

```bash
ros2 run radarays_ros radar_simulator --ros-args \
  -p map_file:=$(ros2 pkg prefix radarays_ros)/share/radarays_ros/testdata/two_walls_test.dae \
  -p map_frame:=map -p sensor_frame:=sensor_test \
  -p materials_file:=$(ros2 pkg prefix radarays_ros)/share/radarays_ros/testdata/two_walls_test_materials.yaml \
  -p material_id_air:=0 -p object_materials:=[1,2] -p serve_action:=true
```

Then, with a real (or bag-replayed) image arriving on some topic:

```bash
ros2 run radarays_ros radaray_opti \
  --server-node-name radar_simulator --sync-topic /Navtech/Polar \
  --material-index 1 2 --maxiter 60 --popsize 15 \
  --output /tmp/opti_result.yaml
```

`--material-index` takes one or more indices — each one adds its own
velocity/ambient/diffuse/specular to the search (e.g. `1 2` tunes 8
parameters at once). This is a slow, global, black-box search (it drives
a live simulation for every candidate). Be aware: on this simple synthetic
scene (two flat wall returns, not much visual structure), an 800-eval run
moved the score only slightly past baseline and didn't fully recover the
hidden ground-truth values for all 8 parameters — see
`MIGRATION_HANDOFF.md`'s Phase 6 for the real numbers. A richer scene (or
real recorded data) gives the metric more to work with than raw eval count
does.

## 6. Real recorded data (MulRan)

Full pipeline (mesh reconstruction from lidar + radar replay + ground
truth TF), once you have a MulRan sequence downloaded (registration-gated,
see `src/radarays_ros/MIGRATION.md`):

```bash
ros2 launch radarays_ros mulran_sim.launch.py \
  meshfile:=<reconstructed.ply> bagfile:=<converted_bag> calib:=<calib.txt>
```
