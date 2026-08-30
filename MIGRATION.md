# radarays_ros Migration Notes

For the current cross-package session handoff, read:

- [`../../MIGRATION_HANDOFF.md`](../../MIGRATION_HANDOFF.md) (at the workspace root, two levels up from this package)

## Current State

`radarays_ros` now has a working ROS 2 build and a working CPU radar
simulator node, on top of the interfaces milestone from earlier in this
session:

- `package.xml` and `CMakeLists.txt`: `ament_cmake`
- message/service/action interfaces build as real ROS 2 interfaces
  (`RadarMaterial(s)`, `RadarModel`, `RadarParams`, `GetRadarParams`,
  `GenRadarImage`) — the `.msg`/`.srv`/`.action` files needed **no content
  changes**, the legacy `pkg/Type` syntax is what `rosidl_generate_interfaces`
  expects too
- `radarays`/`radarays_cpu` libraries and the `radar_simulator` executable
  build and **run**, verified against a real mesh
  (`radarays_gazebo_plugins/worlds/avz_no_roof.stl`) with a real TF chain
  and real materials — `/radar/image` publishes a correctly-sized,
  non-degenerate image (confirmed pixel content: 100% nonzero, max value
  near the configured `signal_max`, not all-zero/garbage)
- `mesh_msgs` and `dynamic_reconfigure` dependencies dropped (see prior
  session note below) — the latter is now genuinely replaced: all ~30
  `RadarModel.cfg` fields are live-tunable ROS 2 parameters

### Ported this session

- **`Radar`/`RadarCPU`**: `ros::NodeHandle` -> `rclcpp::Node::SharedPtr`,
  ROS 1 `tf2_ros` -> ROS 2 `tf2_ros` (`Buffer(clock)`,
  `TransformListener(buffer, node)`, `tf2::TransformException` catch is
  unchanged), `ros::Time`/`geometry_msgs::TransformStamped` ->
  `rclcpp::Time`/`geometry_msgs::msg::TransformStamped`. The physics loop
  itself (fresnel, cone sampling, denoising, ambient noise, the
  OMP-threaded per-thread-simulator structure) is a **faithful line-level
  port**, not a redesign — unlike `radarays_gazebo_plugins`'s
  `radar_algorithms.hpp`, which is a from-scratch rewrite. Both now exist,
  independently, for different purposes (see "Two independent physics
  copies" below).
- **`dynamic_reconfigure::Server<RadarModelConfig>` -> ROS 2 parameters**:
  all ~30 fields from `cfg/RadarModel.cfg` (z_offset, range_min/max,
  beam_width, resolution, n_cells, n_samples, beam_sample_dist(+enum),
  n_reflections, energy_min/max, signal_max, signal_denoising(+enum) and
  its per-kernel width/mode fields, ambient_noise(+enum) and its
  at_signal/energy/perlin fields, scroll_image, multipath_threshold,
  record_multi_reflection, record_multi_path, include_motion) are declared
  ROS 2 parameters with range descriptors, validated the same
  validate-then-apply way as `radarays_gazebo_plugins`.
- **Materials loading redesigned**: ROS 1's `nh->getParam("materials",
  XmlRpc::XmlRpcValue)` read an array of `{velocity, ambient, diffuse,
  specular}` dicts — ROS 2 parameters have no array-of-object type, so
  there is no direct equivalent. Fix: a `materials_file` string parameter
  pointing at a plain YAML file, parsed with `yaml-cpp` (already a system
  dependency via ROS 2 core, no new install needed). `material_id_air` and
  `object_materials` stayed real ROS 2 parameters (scalar int / int
  array — both representable natively).
- **`radar_simulator.cpp`**: ported the path that's actually used. The
  original file's `main()` called `main_publisher` — the action-server
  variant (`main_action_server`) was already commented out and doesn't
  even exist as a function in the file, so it wasn't ported; the
  `GenRadarImage`/`GetRadarParams` interfaces still build (useful if
  something else wants to implement a server against them later) but
  nothing serves them, matching the pre-migration behavior exactly. Also
  dropped: a `pub_pcl` publisher that was advertised but never actually
  published to in the original file (dead code).

### A real bug found by the runtime smoke test

`declare_parameter(name, hardcoded_default)` resolves a `--params-file`/
`-p` override *at declaration time*, but that resolved value was not being
read back into the corresponding `m_cfg_*` member — only
`onSetParameters()` (which only fires on a later, explicit `ros2 param
set`) updated members. Result: passing `n_cells: 512` in a params file was
silently ignored; the image still came out at the hardcoded default height
(3424). Fixed by replaying the resolved parameter values through
`onSetParameters()` once, immediately after declaring everything, in the
`Radar` constructor. Caught by actually running the node against a params
file and checking the real output dimensions — would not have been caught
by a compile check alone.

### ~~Two independent physics copies~~ -- unified

There used to be two separate implementations of the same physics
(fresnel, cone sampling, denoising, Perlin noise): a from-scratch one in
`radarays_gazebo_plugins` (needed while this package was still fully
ROS1, so depending on it was impossible then) and this package's own
`radar_algorithms.h/.cpp`. Not anymore:

- `radar_algorithms.cpp` (the pure math: `Ray`/`DirectedWave`, `fresnel`,
  `back_reflection_shader`, `sample_cone_local`, `make_denoiser_*`, plus
  `radar_math.h`'s `erfinvf` and `image_algorithms.h`'s `perlin_noise`) is
  now built as its own target, `radarays_core` — no ROS dependency at all,
  just `rmagine::core` — exported via `ament_export_targets` so other
  packages can depend on it properly (`find_package(radarays_ros)` +
  `radarays_ros::radarays_core`), the historically-correct dependency
  direction per this package's README ("Radarays Plugins").
- The existing `radarays` library (ROS-coupled: `ros_helper.cpp`,
  `Radar.cpp`) now links `radarays_core` instead of compiling
  `radar_algorithms.cpp` into itself directly — same behavior, just a
  cleaner split.
- `radarays_gazebo_plugins` now depends on `radarays_ros` for this and its
  own `radar_algorithms.hpp` copy is deleted — see its own
  MIGRATION_HANDOFF entry for that side of the change.
- **Not unified, and intentionally so**: `radarays_gazebo_plugins`'s own
  material representation (a plain 4-field struct resolved from SDF tags)
  stayed local rather than switching to this package's ROS-message-based
  `radarays_ros::msg::RadarMaterial` — different sourcing mechanism,
  forcing them together would have been a bigger, riskier change for no
  real benefit. Its `make_ondn_model`-equivalent also stayed local (a few
  lines) since `radar_algorithms.cpp`'s `make_model()` hardcodes
  `range.max` to 1000.0 and `radarays_gazebo_plugins` needs it
  configurable. And `radarays_gazebo_plugins`'s batched-Embree-query
  optimization (see its own MIGRATION_HANDOFF entry) still has no
  counterpart here — this port kept the original per-angle OMP-threaded
  structure faithfully rather than redesigning it, since perf wasn't the
  goal of that milestone.

Verified: all three `radarays_gazebo_plugins` fixtures
(`static_cpu`/`dynamic_cpu`/`dynamic_multi_cpu`) still pass with the exact
same committed thresholds after the switch — the physics is numerically
identical, just sourced from one place instead of two. `radar_simulator`
itself re-verified against the same mesh/TF/materials smoke test as
before, unaffected.

### `ray_reflection_test` ported

Debug/viz tool: shoots one (or a 360-degree fan of) ray(s) from the origin,
recurses through `fresnel()` for `n_reflections` bounces, and publishes the
traversal as a `visualization_msgs/msg/Marker` LINE_LIST. Faithful
line-level port, same style as `Radar`/`RadarCPU`:

- `ros::NodeHandle`/`~private` -> a single `rclcpp::Node` (`ray_reflection_test`)
- `dynamic_reconfigure::Server<RayReflectionConfig>` (`cfg/RayReflection.cfg`)
  -> ROS 2 parameters with range descriptors + `add_on_set_parameters_callback`,
  same validate/apply/replay-on-declare pattern as `Radar::declareReconfigurableParams()`
- `object_materials`/`velocities`/`material_id_air` -> plain ROS 2 array/scalar
  parameters (these were always flat arrays in the ROS 1 version, not an
  array-of-dicts, so — unlike the radar materials — no YAML file detour was
  needed here)
- `tf2_ros` ROS 2 API (`Buffer(clock)`, `TransformListener(buffer, node)`),
  `rclcpp::spin_some()` each loop iteration so the parameter service and TF
  actually get serviced
- added `visualization_msgs` as a real dependency (`package.xml` + `CMakeLists.txt`)
  — the ROS 1 version depended on it implicitly via `roscpp`'s message set

Runtime-verified against the same `avz_no_roof.stl` mesh used for
`radar_simulator`, with a static `map` -> `navtech` TF: the published
`Marker` has real geometry (line count matches `n_reflections + 1`, point
coordinates match the configured `ray_yaw` direction and the mesh's actual
intersection distance), and `spinning: true` produces a smoothly changing
hit range frame to frame — confirms it's really ray-casting against the
mesh, not producing placeholder output.

### `RadarGPU` ported

`RadarGPU.cpp`, `radar_algorithms.cu`/`.cuh`, `image_algorithms.cu`/`.cuh`
ported to ROS 2, and a new `radar_simulator_gpu` node (mirroring
`radar_simulator`, swapped to `RadarGPU`/`OptixMap`). Gated behind
`TARGET rmagine::optix AND CMAKE_CUDA_COMPILER` in `CMakeLists.txt` — the
CPU-only path is completely unaffected if no OptiX SDK is available.

What the port needed beyond mechanical ROS 1 -> ROS 2 substitution
(`ros::NodeHandle` -> `rclcpp::Node::SharedPtr`, `ros::Time` ->
`rclcpp::Time`, `sensor_msgs::ImagePtr` -> `sensor_msgs::msg::Image::SharedPtr`,
`cv_bridge/cv_bridge.h` -> `.hpp`, `m_cfg.x` -> `m_cfg_x` matching
`RadarCPU`'s already-established member names):

- **A genuinely missing type**: `radar_algorithms.cuh` included a
  `<radarays_ros/RadarParams.h>` that doesn't exist in this checkout and
  turned out to be dead — unused by any declaration in that header, so it
  was just dropped. More substantively, `RadarMaterial` (used throughout
  the CUDA kernels as `rm::Memory<RadarMaterial, VRAM_CUDA>`) didn't exist
  as a plain type anywhere either. In ROS 1, `RadarMaterial` was itself the
  message type (ROS 1 messages are plain structs, safely usable as raw GPU
  memory); the ROS 2 message class (`msg::RadarMaterial`) is not
  trivially-copyable, so it can't be memcpy'd to the GPU the same way.
  Fixed by adding a small POD `RadarMaterial` struct (`radar_types.h`,
  same 4 float fields as the message) for the GPU kernels, with an explicit
  field-by-field copy from `msg::RadarMaterial` at the ROS boundary
  (`RadarGPU::simulate()`).
- **Removed a real OpenCV-CUDA dependency that wasn't actually needed**:
  `image_algorithms.cuh` unconditionally included `<opencv2/core/cuda.hpp>`
  for a `fill_perlin_noise(cv::cuda::GpuMat&, ...)` overload that
  `RadarGPU.cpp` never calls (it only uses the `rm::MemView`-based
  overloads). System OpenCV here has no CUDA module built at all, so this
  would have been a hard build blocker for no functional benefit — the
  unused `GpuMat` overload (and its dedicated kernel) was deleted from both
  `image_algorithms.cuh` and `.cu`.
- **`preBuildProgram<ResT>()`**: still templated in this rmagine version
  (confirmed by reading the header, not guessed) — ported unchanged.
- **CUDA language enablement**: rmagine enabling CUDA for its own build
  doesn't carry over to `radarays_ros` (a separate CMake project) — added
  the same `enable_language(CUDA)` guard pattern rmagine itself uses,
  plus `CUDA_ARCHITECTURES all` (CMake >= 3.23) to match `rmagine-cuda`'s
  convention.

**Runtime-verified against the real GPU**, not just a compile check: ran
`radar_simulator_gpu` against the same `avz_no_roof.stl` mesh, TF, and
materials setup as the CPU `radar_simulator` smoke test. 29 consecutive
`simulate()` calls completed successfully at ~5-7ms GPU compute time each
(one harmless initial TF-not-yet-received race, same as any ROS 2 node
started before its first `/tf_static` arrives). Subscribed to `/radar/image`
directly and computed real pixel statistics: 512x400 `mono8`, 99.87%
nonzero, min 0 / max 132 / mean ~14.5 — a physically plausible, non-degenerate
radar image, not placeholder or garbage output.

### `GetRadarParams`/`GenRadarImage` served

The two interfaces built since milestone 1 (see top of this file) but
never served by any node -- matching Classic, where the
`main_action_server` entry point was already commented out/unused before
this migration, so there was no reference implementation to port. New
`serve_action` bool parameter on `radar_simulator`
(`-p serve_action:=true`): when set, skips the continuous-publish/
`sync_topic` loop entirely and instead serves `get_radar_params` (a
`GetRadarParams` service, just returns `radar_->getParams()`) and
`gen_radar_image` (a `GenRadarImage` action: applies `goal->params` via
`setParams()`, calls `simulate()`, returns the image as the action
result).

The two modes are kept mutually exclusive (same node, alternate startup
path) rather than made concurrent, because `Radar`/`RadarCPU` isn't
thread-safe against itself -- concurrent `simulate()`/`setParams()` calls
from two different callback contexts would race on shared members. This
mirrors Classic's own `main_publisher` vs `main_action_server` being
alternate `main()` entry points, never run together.

A real bug surfaced during runtime verification, not just compile-testing:
the first working version called `execute()` directly from the action
server's accepted-goal callback, which is dispatched by `rclcpp::spin(node)`.
But `RadarCPU::simulate()` itself calls `rclcpp::spin_some(m_node)`
whenever `include_motion` is set (the default) -- and rclcpp forbids adding
the same node to two executors at once, even transiently, so the reentrant
`spin_some()` call crashed the node with `std::runtime_error: Node has
already been added to an executor`. Fixed by having the accepted-goal
callback only queue the goal handle; a manual polling loop in `main()`
(`spin_some(node)` then drain the queue then sleep, same shape as the
existing free-running loop) executes the goal outside of any active
executor callback, exactly where the free-running loop already safely
calls `simulate()`.

**Runtime-verified** against the same `avz_no_roof.stl` mesh/TF used for
the other `radar_simulator` smoke tests: `ros2 service call
/get_radar_params radarays_ros/srv/GetRadarParams "{}"` returns the live
params; `ros2 action send_goal /gen_radar_image
radarays_ros/action/GenRadarImage "{params: {...}}"` returns `SUCCEEDED`
with a real 3424x400 `mono8` image; a follow-up `get_radar_params` call
confirms the goal's params (`beam_width`/`n_reflections`) were actually
applied to the shared `Radar` state. Sent two goals back-to-back with no
crash, confirming the queue-drain fix holds under repeated calls.

**Update:** the exact same reentrancy bug turned out to also exist in
`radar_simulator`'s `sync_topic` branch (same root cause: blocking
`rclcpp::spin(node)` while `simulate()` internally calls `spin_some()`) —
flagged as a latent risk here at the time, later confirmed for real once
the real MulRan bag was played through it. Fixed the same way. See "Real
MulRan validation — first run against the ParkingLot sample" below.

### MulRan dataset work

Scoped out what real MulRan-based validation actually requires, rather
than just noting it's blocked. The **full** pipeline the top-level
`README.md`'s "MulRan" section describes (mesh reconstruction via `lvr2`,
localization via MICP-L/`micp_experiments`, bag replay via
`file_player_mulran`, comparison via a `radar_tools` package) turned out to
have real, individually-verified pieces and real, individually-verified
dead ends — findings below, in case a future session wants to build
further on this rather than re-derive it:

- **`lvr2`** (mesh reconstruction, `https://github.com/uos/lvr2`): builds
  clean on this ROS 2 Jazzy workspace as a plain-`cmake`-type ament
  package — only missing system dep was `libgsl-dev`. `lvr2_reconstruct`
  installed and ready. No real platform issue here.
- **`RMCL`/`rmcl_msgs`/`rmcl_ros`** (`https://github.com/uos/rmcl`, the
  successor to MICP-L's original implementation): also builds clean,
  including the GPU/OptiX localization node (`micp_localization_node`).
  Needs `rmagine 2.4`, which this workspace already has. No real platform
  issue here either.
- **`micp_experiments`**'s `micp_mulran` (`https://github.com/aock/micp_experiments`):
  ROS 1/catkin only — no catkin on this system, dead end without a full
  port.
- **`micp_experiments`**'s `micp_mulran2` (nominally ROS 2): fixed two real
  `find_package` ordering bugs to get it to *configure* (rmcl's exported
  targets need `rmagine`'s `cuda`/`optix` components found first;
  `imu_filter_madgwick`'s export needs `tf2_geometry_msgs` found first) —
  but then hit a genuine **API version-skew**, not a platform issue:
  `micp_mulran2_node.cpp` references rmcl's old `Corrector`-based API
  (`O1DnCorrectorEmbree`, `math_batched.h`, `math.h`, `ros_helper.h`) which
  no longer exists in current `rmcl` (refactored into `Updater`/`Pipeline`
  classes, and separate `registration/` correspondence-finding primitives
  like `RCCEmbree`/`CPCEmbree` for the parts that do still exist under new
  names). Confirmed this isn't worth force-porting: `rmcl`'s own
  `docs/MICPL.md` explicitly states `micp_experiments` is "primarily
  compatible with the ROS 1 version... see the older branches or commits
  for reference," and points at `rmcl_ros`'s own generic
  `micp_localization_node` (config via `map_file` + a YAML sensor config)
  plus a separate, actually-maintained `rmcl_examples` repo as the current
  way to do MICP-L. Correctly not pursued further given the author's own
  guidance.
- **`file_player_mulran`** (`https://github.com/RPM-Robotics-Lab/file_player_mulran`,
  replays MulRan's raw per-sensor files as ROS topics): also ROS 1/catkin
  only, and not a simple republisher — a 1563-line Qt5 GUI application
  (play/pause/scrub controls, live playback-rate timing, LiDAR/IMU/GPS
  handling via PCL). Porting all of that for a task that only needs radar
  images and ground-truth poses would be a large, mostly-wasted effort.
  Instead, reimplemented **only the relevant piece** of its own
  `ROSThread::SaveRosbag()` (radar-polar PNGs + `global_pose.csv` ground
  truth -> a bag) directly in ROS 2 as a new small script:
  `scripts/mulran_radar_to_bag.py` (`ros2 run radarays_ros
  mulran_radar_to_bag --input <sequence_root> --output <bag_dir>`), reading
  `<sequence_root>/sensor_data/radar/polar/<stamp_ns>.png` and
  `<sequence_root>/global_pose.csv` (KITTI-style: `stamp,` + row-major
  `[R|t]`, 13 fields/row — same layout `file_player_mulran`'s own code
  parses), writing a standard ros2 bag (`/Navtech/Polar`
  `sensor_msgs/msg/Image`, `/gt` `nav_msgs/msg/Odometry`), replayable with
  plain `ros2 bag play` — no GUI/Qt porting needed at all.

**Verified against synthetic data built to match MulRan's real layout**
(no real MulRan data was available to this session — see below): 3 fake
radar-polar PNGs + a 3-row `global_pose.csv` with a known translating
pose. Confirmed via `ros2 bag info` (correct topic names/types/counts,
correct start/end timestamps) and by deserializing every message back
(correct image dimensions/encoding/byte length; correct position values
matching the synthetic translation; correct quaternion from the identity
rotation matrix). Not yet run against real MulRan data — that's still
blocked on the dataset registration (see next section).

### Real MulRan validation — first run against the ParkingLot sample

The dataset blocker is resolved: the user supplied the free "ParkingLot"
sample sequence (`https://sites.google.com/view/mulran-pr/download`, no
registration needed) plus MulRan's own `calib/` extrinsics zip
(`base2ouster`/`base2radar`), extracted to `data/mulran/` at the workspace
root. This is the first time this pipeline has run against real recorded
data instead of synthetic stand-ins.

Two new tools were built to close the gaps the runbook below used to flag
as "not yet built":

- **`mulran_lidar_to_cloud`** (`scripts/mulran_lidar_to_cloud.py`):
  accumulates a sequence's Ouster `.bin` scans into one world-frame point
  cloud for `lvr2_reconstruct`, transforming each scan sensor -> base ->
  world using a fixed base->ouster extrinsic (parsed from MulRan's
  `calib_base2outer.txt` format) and that scan's own nearest-timestamp
  pose from `global_pose.csv`. Only every Nth scan is used by default
  (`--stride`, default 20) — a full sequence is thousands of 65536-point
  scans, more than needed for a first reconstruction.
- **`mulran_gt_to_tf`** (`scripts/mulran_gt_to_tf.py`): republishes a
  `mulran_radar_to_bag`-produced bag's `/gt` as a live `map -> base_link`
  TF each tick, plus a static `base_link -> radar` extrinsic parsed from
  `calib_base2radar.txt` — the "drive TF from ground truth" step the
  runbook previously had no code for.

Both `mulran_lidar_to_cloud.py` and `mulran_radar_to_bag.py` also needed a
**UTM recentering fix**: `global_pose.csv`'s translations are raw UTM
coordinates (hundreds-of-thousands/millions range), which would silently
eat into float32 precision once fed through Gazebo/rmagine (both
float32-based) for no benefit. Both scripts now subtract the sequence's
first pose's translation before using it, independently deriving the same
origin from the same file's first row so they agree without sharing state.

**Two real, `.bin`-file-format-shaped surprises found only by running
against the actual sample** (not something a synthetic-data test could
have caught):

1. The free ParkingLot sample uses a flatter `<sequence_root>/polar/` and
   `<sequence_root>/Ouster/` layout, not the `sensor_data/radar/polar/`
   nesting the full registered sequences apparently use (per MulRan's own
   devkit conventions, which this session had only read about, not seen
   directly). Fixed by trying both layouts in both conversion scripts.
2. Two real bugs surfaced under real, sustained load that no toy-world
   smoke test had exercised:
   - **The exact same executor-reentrancy crash fixed for
     `serve_action`** (see "`GetRadarParams`/`GenRadarImage` served"
     above) turned out to also exist in `radar_simulator`'s `sync_topic`
     branch — flagged as a latent risk at the time, now confirmed for
     real: `rclcpp::spin(node)` blocking on the same node that
     `RadarCPU::simulate()` internally calls `rclcpp::spin_some()` on
     (when `include_motion` is set, the default) crashed with the same
     `Node has already been added to an executor` error once the real bag
     was played through it. Fixed the same way — the subscription
     callback now only queues the latest sync stamp; a manual polling
     loop drains it between `spin_some()` calls, never from within one.
     (Checked `radar_simulator_gpu.cpp`/`RadarGPU.cpp` too: no internal
     `spin_some()` call exists there at all, so that file's `sync_topic`
     branch was never actually at risk — left unchanged.)
   - **Material index 0 means "air" by convention**
     (`material_id_air` defaults to `0`, and `object_materials` defaults
     to an empty array, which makes every hit fall back to material index
     `0` regardless of the actual geometry). A `materials_file` with a
     single entry at index 0 is therefore indistinguishable from no
     material at all — every ray just "re-enters air" at each surface, so
     the whole image comes back completely black even though everything
     else is working correctly. This isn't a code bug, just an easy-to-miss
     convention with no error or warning when you get it wrong; needs a
     materials file with **at least two entries** (index 0 = air-like
     zeroed-out properties, index 1+ = real materials) plus a matching
     `object_materials` parameter (e.g. `[1]` for a single-geometry mesh)
     to ever produce non-zero output.

**Result, real data end to end**: reconstructed a mesh from 59 Ouster
scans (stride 20, ~3.87M accumulated points) via `lvr2_reconstruct`
(684,319 vertices / 1,159,391 faces, ~27s); converted the real sequence's
477 radar-polar images + 10,093 ground-truth poses into a bag (`ros2 bag
info`: 119.2s duration, matches); ran `radar_simulator` against the
reconstructed mesh, TF driven by `mulran_gt_to_tf` from the real `/gt`
stream, synced to the real `/Navtech/Polar` timestamps. No crash across
the full bag replay (previously would have crashed immediately, see bug
#2 above). Compared real vs. simulated output directly: real
`/Navtech/Polar` was 53.3% nonzero (mean 8.07, max 126); simulated
`/radar/image`, once materials were configured correctly, was 98.9%
nonzero (mean 20.8, max 115) — both clearly non-degenerate and the same
order of magnitude, though not a close match, which is expected: the
material properties used (`ambient: 0.2, diffuse: 0.7, specular: 0.2`)
are illustrative placeholders, not measured asphalt/concrete/vegetation
reflectivity, and only a single generic material was assigned to the
entire reconstructed mesh (no per-surface-type breakdown). Getting closer
than "same order of magnitude" would need real material calibration, not
more code.

#### Runbook: reproducing or extending this

1. **Get a sequence.** The free "ParkingLot" sample (used above) or, for
   more rigor, register at `https://sites.google.com/view/mulran-pr` for a
   full sequence (e.g. `DCC01`) — same tooling either way, since both
   directory layouts are handled (see finding #1 above).

2. **Reconstruct a mesh:**
   ```
   ros2 run radarays_ros mulran_lidar_to_cloud \
     --input <sequence_root> --calib <calib_base2outer.txt> \
     --output cloud.xyz --stride 20
   lvr2_reconstruct --inputFile cloud.xyz --outputDirectory <mesh_dir> --voxelsize 0.5
   ```
   Lower `--stride`/raise scan count and tune `--voxelsize` to the
   sequence's actual point density for a denser reconstruction.

3. **Convert radar + ground truth to a bag:**
   ```
   ros2 run radarays_ros mulran_radar_to_bag \
     --input <sequence_root> --output <bag_dir>
   ```

4. **Drive TF from the ground truth**, in a separate terminal from bag
   playback:
   ```
   ros2 run radarays_ros mulran_gt_to_tf --calib <calib_base2radar.txt> \
     --map-frame map --base-frame base_link --sensor-frame radar
   ```

5. **Run the simulator against the reconstructed mesh**, with a real
   materials file (2+ entries, see finding #2 above) and a matching
   `object_materials`:
   ```
   ros2 run radarays_ros radar_simulator --ros-args \
     -p map_file:=<mesh_dir>/triangle_mesh.ply \
     -p map_frame:=map -p sensor_frame:=radar \
     -p sync_topic:=/Navtech/Polar \
     -p materials_file:=<materials.yaml> -p object_materials:="[1]"
   ros2 bag play <bag_dir>
   ```
   (Swap `radar_simulator` for `radar_simulator_gpu` for the GPU/OptiX
   path — untested against real MulRan data this session, but the same
   parameters apply and it has no equivalent reentrancy risk, see bug #2
   above.)

6. **Compare** `/radar/image` (simulated) against `/Navtech/Polar` (real)
   — pixel statistics (mean/nonzero-ratio, as done above) as a first pass,
   or the actual paper's own comparison method for more rigor (see the
   "MulRan" section of README.md for the published side-by-side example
   image and paper reference).

### `radar_simulator` regression fixture — 2026-07-29

This package had zero registered `colcon test` entries at all, unlike its
two sibling Gazebo-plugin packages (each of which has 3-8 fixture tests).
Closed the gap with the package's first-ever automated test:
`radarays_ros_fixture_wall` (`colcon test --packages-select radarays_ros`).

Unlike the Gazebo-plugin fixtures, there's no `gz sim` involved at all --
`radar_simulator` is a standalone ROS 2 node, so
`scripts/radarays_ros_fixture_harness.py` launches it directly as a
subprocess (alongside a `static_transform_publisher` for the sensor pose)
against a small self-contained synthetic mesh
(`testdata/wall_test.ply` -- a thin wall, near face at x=5, hand-written
as a minimal ASCII PLY so this doesn't depend on a real dataset mesh or
another package's world file).

**A real design mistake found and fixed while building this**: the first
version's "falsifiable" check assumed a `width // 2` "center column"
corresponds to the sensor's boresight -- true for the Gazebo-plugin
sensors (fixed FOV), but `radar_simulator` is a full 360-degree rotating
radar. Checked empirically: the assumed center column was entirely zero
the whole capture (it points away from the wall, not at it). Fixed by
computing the range (row), not azimuth (column), of the target's peak
return, summed across *all* columns -- correct for a full sweep where a
single target is only visible from whichever columns actually point at
it.

**A second real bug found via a failure, not assumed away**: the fixture
initially used a raw stddev on the peak row across the capture window,
and intermittently failed in the full 4-package `colcon test` run (but
not in isolation) with the peak row reading 0 for the first several
frames before settling. Investigated rather than just loosening the
threshold: this is a genuine startup transient (TF/mesh warm-up takes a
variable number of frames depending on system load), not noise. Fixed
properly -- the capture loop now waits until the peak row is off its
degenerate startup value before starting the timed capture window at
all, discarding whatever warm-up frames arrived during the wait (same
shape as the other two harnesses' own multi-condition "wait for scan AND
points AND image" readiness check). Also switched the stability check
itself from raw stddev to a median/majority-consensus ratio (95% of
frames must be within 3 rows of the median) -- more robust to any future
one-off outlier frame without masking a real systematic shift.

**A separate, real process-leak bug found investigating the same
failure**: while debugging the above, `ros2 node list` showed the same
`/radar/image` topic receiving messages with the wrong image height
(3424, this package's default, instead of `radarays_gazebo_plugins`'
fixture's expected 1024) during a run of that *other* package's own
fixture. Root cause: leaked `radar_simulator`/`static_transform_publisher`
processes from earlier manual testing this session, still running and
still publishing to the same topic name on the same `ROS_DOMAIN_ID` (see
MIGRATION_HANDOFF.md's "Phase 3" section on domain sharing) -- killed by
PID for the `ros2 run` wrapper process, not the actual child executable
it spawns, so the real node kept running silently in the background.
Not a bug in any package's code -- a reminder (again) that `ros2 run`'s
own process needs killing by its *child's* PID, not the wrapper's.

## What Is Not Migrated Yet

- ~~`mesh_publisher.cpp` — already unbuilt before this migration (`mesh_msgs`
  has no ROS 2 Jazzy release; not referenced in the ROS 1
  `CMakeLists.txt`'s build list either)~~ — **ported 2026-07-30**, but
  deliberately not as a 1:1 message port. Re-checked first, not just
  assumed still true: upstream `lvr-ros/mesh_msgs` is still
  `catkin`/ROS 1-only (`buildtool_depend: catkin`,
  `message_generation`/`message_runtime`), no ROS 2 branch, no rosdep rule
  for `jazzy` — genuinely still blocked, and its intended RViz1 consumer
  (`rviz_map_plugin`/`mesh_tools`) has no ROS 2 release either, so a
  faithful message-for-message port would be inert without also building
  a new RViz2 plugin. Instead, re-targeted it at `visualization_msgs/
  MarkerArray` (`TRIANGLE_LIST` per mesh) — the same message type
  `ray_reflection_test.cpp` already publishes successfully, rendered
  natively by RViz2 with zero extra plugins or external dependencies.
  Runtime-verified: loads `testdata/two_walls_test.dae`, publishes real
  triangle geometry matching the mesh's actual vertices (confirmed via
  `ros2 topic echo /mesh_markers`), not just "builds without crashing."
- ~~The `GenRadarImage` action and `GetRadarParams` service have real ROS 2
  interfaces but nothing serves them~~ — now served by `radar_simulator
  -p serve_action:=true` (see "`GetRadarParams`/`GenRadarImage` served"
  above).
- ~~Real dataset replay validation against MulRan: tooling is now ready...
  only the dataset itself remains blocked~~ — run for real against the
  free ParkingLot sample, see "Real MulRan validation — first run against
  the ParkingLot sample" above. Real materials calibration (currently a
  single placeholder material for the whole mesh) and a full registered
  sequence (DCC/KAIST/etc, more scale + real IMU/GPS) remain open if more
  rigor is wanted.

## Migration Direction

Remaining, in priority order:

~~1. Real dataset validation (MulRan)~~ — done against the free
   ParkingLot sample, see "Real MulRan validation — first run against the
   ParkingLot sample" above. Real (not placeholder) material calibration
   and a full registered sequence remain open if more rigor is wanted.
~~2. GPU/OptiX radar path in `radarays_gazebo_plugins`~~ — done, see
   `radarays_gazebo_plugins/MIGRATION.md`, "GPU/OptiX radar path".

~~3. `ray_reflection_test.cpp` port~~ — done, see above.

~~4. Consider unifying the two physics copies~~ — done, see above.

~~5. `RadarGPU`/OptiX path~~ — done, see above.

~~6. Serve `GetRadarParams`/`GenRadarImage`~~ — done, see
   "`GetRadarParams`/`GenRadarImage` served" above.

## Known Gaps vs the ROS 1 (`noetic`) Branch (systematic audit)

A dedicated audit comparing the current working tree against
`origin/noetic` (the frozen ROS 1 reference) found the following. Note an
important side-finding first: **nothing in this repo has actually been
committed** — `main` is untouched, and every change described in this
whole file exists only in the uncommitted working tree. Not a code gap,
but worth being aware of before doing anything destructive (`git
checkout`/`git clean`/etc. on this repo would lose all of it).

Concrete gaps found, all still open:

1. ~~**Launch files were never touched at all**~~ -- done for the one that
   matters for this repo's own MulRan runbook: `launch/mulran_sim.launch.py`
   ports `launch/mulran_sim.launch` (the ROS 1 XML is left in place,
   untouched, as a reference -- different filename, no collision). It
   operationalizes this file's own "Runbook: reproducing or extending
   this" as one command: `ros2 bag play --start-paused` (ROS 2 renamed
   ROS 1's `--pause`), `mulran_gt_to_tf` (map -> base_link -> sensor TF),
   and `radar_simulator` with `sync_topic`/`materials_file`/
   `object_materials` (the latter two pulled from the same
   `materials_file` YAML at launch time, so there's one source of truth
   instead of duplicating the list into a second params file). `gui`/RViz
   is deliberately not ported -- no `.rviz2` config exists in this repo and
   the ROS 1 one depends on the `mesh_tools` fork, already out of scope
   (see README.md). `CMakeLists.txt` now installs `launch/`, `cfg/`, and
   `config/` under `share/${PROJECT_NAME}` (previously none of the three
   were installed at all, so `ros2 launch radarays_ros ...` had nothing to
   find regardless of whether a `.launch.py` existed).

   **Runtime-verified** against real MulRan data (the ParkingLot sample
   used throughout this file, re-converted to a bag with
   `mulran_radar_to_bag`) — launched with a stand-in mesh (this session's
   already-verified `avz_no_roof.stl`, since a real MulRan-reconstructed
   mesh from this sample wasn't kept on disk from earlier this session)
   and a matching 2-material file: all 3 processes started
   (`rosbag2_player`, `mulran_gt_to_tf`, `radar_simulator`), TF resolved
   once the bag was resumed, and `/radar/image` began publishing real
   pixel data driven by the actual `/Navtech/Polar` bag messages.

   **This surfaced a second real, previously-undiscovered bug** while
   testing at the ROS 1 launch file's own default rate (1.0) and above:
   `radar_simulator` segfaulted a few messages into real sync_topic
   playback. Root-caused (not just observed) by reading `RadarCPU.cpp`'s
   `#pragma omp parallel for` loop: `m_sims` (an
   `unordered_map<int, OnDnSimulatorEmbree>` keyed by OMP thread id,
   lazily populated with one simulator per thread on first use) was read
   via `.find()` and written via `operator[]` from every thread in the
   parallel region with **no synchronization at all** — only the
   `"Created new simulator..."` log line was inside `#pragma omp
   critical`, not the map access itself. Concurrent `find()`/insert on a
   plain `std::unordered_map` is undefined behavior (an insert can trigger
   a rehash while another thread's `find()` is mid-lookup), so this raced
   and crashed once enough distinct thread ids had shown up under
   sustained real message throughput — reproduced independently of the
   new launch file (identical crash running `radar_simulator`/`ros2 bag
   play` by hand, no `mulran_sim.launch.py` involved) and at multiple
   rates (1.0, 5.0), confirming it's a pre-existing concurrency bug in
   `RadarCPU`, not something the launch port introduced. **Fixed** by
   moving the whole find-or-create block inside one named `#pragma omp
   critical (radarcpu_sims_map)` section instead of just the print.
   **Re-verified**: the exact same launch/bag/rate combination that
   crashed within ~4 seconds before the fix now runs the full 30-second
   test window cleanly, with `radar_simulator` still alive and
   `/radar/image` still publishing at the end. (RadarGPU.cpp has no
   equivalent per-thread lazy-map pattern — checked, not affected.)
2. ~~Example parameter files (`cfg/mulran_kaist_dyncfg*.yaml`) are still
   ROS 1 `dynamic_reconfigure` pickle-format YAML~~ -- done: all 3 files
   (`mulran_kaist_dyncfg.yaml`, `_laserlike.yaml`, `_minimal.yaml`)
   converted in place to ROS 2 `/**:\n  ros__parameters:` format (34, 34,
   and 32 fields respectively). Field names were cross-checked 1:1
   against every `declare_parameter`/`declare_ranged_double` name in
   `Radar.cpp` -- no renames needed, and `beam_width` confirmed to already
   be in degrees in both formats (converted to radians only internally),
   so no unit conversion either. **Runtime-verified**: `ros2 run
   radarays_ros radar_simulator --ros-args -p map_file:=<mesh>
   --params-file cfg/mulran_kaist_dyncfg.yaml` loads cleanly and resizes
   the canvas to 3424 (matching the file's `n_cells`), confirming the
   params are actually parsed and applied, not just schema-valid.
3. ~~`cfg/RadarModel.cfg`/`cfg/RayReflection.cfg` are now dead files~~ --
   done: deleted (still recoverable from git history — both were tracked,
   confirmed via `git status`/`git diff` before deleting; the only diff
   from the committed version was a file-mode change, no content lost).
4. ~~**The CPU/GPU runtime switch on `radar_simulator` is gone.**~~ --
   guarded rather than restored (restoring the actual switch would mean
   merging `RadarCPU`/`RadarGPU` back into one executable, undoing the
   CMakeLists split this migration deliberately made -- out of scope for
   closing a parity gap). `radar_simulator` now declares a `gpu` param
   purely to catch it: if `gpu:=true` is passed, the node logs a clear
   error naming `radar_simulator_gpu` as the correct executable and exits
   non-zero, instead of silently running on the CPU anyway.
   **Runtime-verified**: `-p gpu:=true` produces exactly that error and a
   non-zero exit, confirmed live.
5. ~~`radar_simulator_gpu` never gained the `serve_action` mode~~ -- done.
   Ported the identical `GenRadarImageServer` class/queue-and-drain
   pattern from `radar_simulator.cpp` (RadarGPU's own `simulate()` has no
   internal `rclcpp::spin_some()` call, so the executor-reentrancy bug
   that pattern exists to avoid doesn't actually apply here -- used
   anyway for consistency between the two files and as a safety margin
   if RadarGPU ever grows similar continuation logic later).

   **Runtime-verified**, and this surfaced a real, separate,
   **pre-existing** bug in the process: sending a `GenRadarImage` goal
   with empty `materials` crashed the node with
   `cudaErrorIllegalAddress` during a `cudaFree` inside
   `rmagine::cuda::free`. Confirmed via isolation testing that this has
   *nothing to do with the new action-server code* -- the exact same
   crash reproduces on the plain, unmodified free-running
   `radar_simulator_gpu` (no `serve_action` involved at all) whenever no
   `materials_file` is configured; with a real 2+-entry materials file
   (matching the `material_id_air`-at-index-0 convention documented
   above), both the free-running node and the new `serve_action` mode
   run correctly -- a `GenRadarImage` goal with real materials returned
   `SUCCEEDED`, and a follow-up `get_radar_params` call confirmed the
   goal's params were actually applied. This empty-materials GPU crash
   is a real, open bug (CPU has no equivalent issue -- confirmed
   throughout this whole migration's earlier CPU testing), just outside
   the scope of "add `serve_action` parity" to fix here; noted for
   whoever picks up `RadarGPU.cpp`'s buffer-management code next.

See MIGRATION_HANDOFF.md's cross-package gap list for how this fits
against the other 2 ROS-coupled packages' own gaps and a proposed tackle
order.
