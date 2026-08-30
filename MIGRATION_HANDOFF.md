# Migration Handoff

Date: 2026-04-17

This file is the current handoff for the ROS 2 Jazzy + Gazebo Harmonic migration
work in this workspace. It is intended to answer three questions quickly:

1. What has already been done.
2. What is being done right now.
3. What the important next steps are, both short term and long term.

## Big Picture

The original problem is a migration from:

- ROS 1 + Gazebo Classic

to:

- ROS 2 Jazzy + Gazebo Harmonic (`gz-sim8`)

The critical packages for this effort are:

- `rmagine`
- `rmagine_gazebo_plugins`
- `radarays_gazebo_plugins`
- `radarays_ros`

The migration is not complete yet. The current state is:

- `rmagine_gazebo_plugins` has a working Harmonic Embree prototype and validation harness.
- `radarays_gazebo_plugins` has a real, feature-complete-for-now ROS 2 / Harmonic integration path (image generation, materials, dynamic reconfiguration, static + dynamic scenarios).
- `radarays_ros` has its `ament_cmake` build and ROS 2 message/service/action interfaces working; the node logic (ROS 1 `Radar`/`RadarCPU`/`RadarGPU` classes, `radar_simulator`, `ray_reflection_test`) is not migrated yet.

## Known Environment Issues

### `libiomp5.so` missing breaks the Embree map system at runtime

`rmagine`'s CMake auto-downloads a precompiled Embree 4.4.0 binary release
(from the RenderKit GitHub releases) whenever no local Embree install is
found. That precompiled binary needs `libiomp5.so` (Intel's OpenMP runtime)
at `dlopen` time inside `gz-sim`. This dependency does not show up in a
static `ldd` check — it only surfaces when Gazebo actually loads the plugin.

Symptom:

```
Error while loading the library [.../librmagine_embree_map_system.so]: libiomp5.so: cannot open shared object file
[RmagineEmbreeSensorSystem] No map available for key 'default'.
```

Effect: `/radar/scan`, `/radar/points`, `/radar/image` all get advertised but
never publish real data, because the Embree map never builds.

Fix used on this machine (no `libiomp5` package exists in apt; `libomp5`
ships a different SONAME and does not satisfy the lookup):

```console
sudo ln -sf /lib/x86_64-linux-gnu/libgomp.so.1 /usr/local/lib/libiomp5.so
sudo ldconfig
```

`libgomp` (GNU OpenMP) is ABI-compatible enough for this to work as a
drop-in — confirmed by relaunching `gz_static_radar_cpu.launch.py` and
seeing finite, non-zero ranges on `/radar/scan`. Note: relying on ldconfig's
cache alone was not enough to make the symlink resolve for `gz-sim`'s
`dlopen`; if this recurs, also try exporting
`LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH` before launching.

If setting up this workspace on a new machine, expect to hit this again
unless `libiomp5.so` (or an equivalent) is already present.

## What Has Already Been Done

### 1. `rmagine_gazebo_plugins` Harmonic backend prototype

The package now has native `gz-sim` systems under `include/.../gz` and `src/gz`:

- `rmagine_embree_map_system`
- `rmagine_embree_sensor_system`
- `map_registry`
- `test_box_mover_system`

That means the workspace already has a Harmonic-side implementation for:

- building an Embree map from `gz-sim` visuals
- publishing ROS 2 `LaserScan`
- publishing ROS 2 `PointCloud2`
- refreshing the simulator when the map changes
- validating dynamic obstacle updates in controlled worlds

### 2. `rmagine_gazebo_plugins` validation worlds

The validation worlds are now normalized and have clear roles:

- `src/rmagine_gazebo_plugins/worlds/gz_embree_baseline.sdf`
  - static sensing correctness
- `src/rmagine_gazebo_plugins/worlds/gz_embree_dynamic.sdf`
  - moving obstacle responsiveness
- `src/rmagine_gazebo_plugins/worlds/gz_embree_example.sdf`
  - compatibility alias of `gz_embree_dynamic.sdf`

The dynamic world uses x-only obstacle motion to keep debugging interpretable.

### 3. `rmagine_gazebo_plugins` fixture harness

There is now a stored capture/compare regression flow for the Harmonic backend:

- `capture_embree_fixture baseline`
- `compare_embree_fixture baseline`
- `capture_embree_fixture dynamic`
- `compare_embree_fixture dynamic`

Committed fixtures:

- `src/rmagine_gazebo_plugins/testdata/embree_harmonic/baseline_fixture.json`
- `src/rmagine_gazebo_plugins/testdata/embree_harmonic/dynamic_fixture.json`

Validated measured behavior:

- baseline world:
  - stable finite `/scan`
  - stable finite `/points`
  - center beam matched simple box geometry
- dynamic world:
  - `/scan` changed over time with the moving box
  - `/points` changed over time with the moving box
  - map rebuild / simulator refresh path behaved consistently

### 4. `radarays_gazebo_plugins` build-system migration start

`radarays_gazebo_plugins` is no longer blocked at the package/build level.

Changes already made:

- `package.xml` converted to ROS 2 / `ament_cmake`
- `CMakeLists.txt` converted to a ROS 2 / Harmonic-first path
- Classic source files remain in-repo but are not part of the new ROS 2 build

### 5. First real Harmonic `radarays` integration scenario

The first non-toy `radarays` Harmonic scenario now exists:

- world:
  - `src/radarays_gazebo_plugins/worlds/gz_static_radar_cpu.sdf`
- launch:
  - `src/radarays_gazebo_plugins/launch/gz_static_radar_cpu.launch.py`

This is intentionally thin:

- `radarays_gazebo_plugins` provides the world and launch path
- `rmagine_gazebo_plugins` provides the delegated map/sensor runtime

Outputs validated in this scenario:

- `/radar/scan`
- `/radar/points`

### 6. `radarays_gazebo_plugins` regression harness

The first `radarays` Harmonic scenario now also has a regression fixture:

- `capture_radarays_fixture static_cpu`
- `compare_radarays_fixture static_cpu`

Committed fixture:

- `src/radarays_gazebo_plugins/testdata/radarays_harmonic/static_cpu_fixture.json`

Measured output:

- `/tmp/radarays_harmonic_static_cpu_capture.json`

This confirms the first real `radarays` Harmonic scenario is reproducible.

The committed `static_cpu` fixture is now intended to cover:

- `/radar/scan`
- `/radar/points`
- `/radar/image`

### 7. First ROS 2 `/radar/image` bridge

There is now a first radar-specific ROS 2 image topic in the Harmonic path:

- `/radar/image`

Current implementation:

- node:
  - `src/radarays_gazebo_plugins/scripts/radarays_scan_image_node.py`
- launched from:
  - `gz_static_radar_cpu.launch.py`

Important boundary:

- this is a migration bridge
- it derives the image from `/radar/scan`
- it is not yet equivalent to the old Classic radar-image simulation

Validated runtime checks:

- `/radar/image` exists
- publisher count is correct
- image contains nonzero pixels
- sample check showed:
  - `width=400`
  - `height=1024`
  - `nonzero=1200`
  - `max=255`

That confirms the bridge is alive and writing visible image content.

### 8. Real Harmonic-native radar image generation (bridge retired)

The scan-derived bridge above is now replaced. `radarays_gazebo_plugins` has
its own gz-sim system:

- `radarays_embree_sensor_system` (`src/gz/radarays_embree_sensor_system.cpp`)
- ported from Classic `radarays_ros` (`RadarCPU.cpp`, `radar_algorithms.*`,
  `radar_types.h`, the Perlin noise from `image_algorithms.h`) into a
  self-contained header, `include/radarays_gazebo_plugins/radar_algorithms.hpp`
  — no ROS1, `dynamic_reconfigure`, or OpenCV/`cv_bridge` dependency
- reuses the delegated Embree map from `rmagine_gazebo_plugins` (via the now
  install-exported `MapRegistry`) — the map/scene lifecycle is still not
  duplicated, per the standing migration rule

Physics ported: multi-bounce Fresnel reflection/refraction, cone-sampled
beams (`sample_cone_local`), signal denoising (triangular / gaussian /
maxwell-boltzmann), ambient noise (uniform / Perlin), optional multipath
recording. All of it runs directly off the same Embree map/backend
`rmagine_gazebo_plugins` already validates — no new raycasting core needed.

Materials are intentionally **not yet** SDF-configurable here: every hit
surface uses one default reflective material (matching Classic's own
fallback when no `radarays_material` tags are present). Per-visual material
parsing is still the next milestone (see Longer-Term #2 below).

Two real bugs surfaced only by testing this end-to-end, both fixed:

- **~1200 tiny Embree queries/frame → whole-sim stall.** The first version
  ray-traced one angle at a time (400 angles × up to 3 reflection passes).
  Fixed by batching: since every angle only differs from the sensor pose by
  a zero-translation rotation, that rotation can be pre-baked into each
  wave's local direction and the entire frame's rays traced in one Embree
  call per reflection pass (down to ~3 calls/frame instead of ~1200).
- **Reliable QoS + depth-1 queue on a 400KB message → `publish()` stalls the
  whole gz-sim update loop.** The scan/points publishers copy fine at
  reliable+depth-1 because those messages are a few KB; the image publisher
  needed `rclcpp::SensorDataQoS()` (best-effort/volatile) instead — the
  conventional QoS for image streams. Any new subscriber to `/radar/image`
  needs a compatible (best-effort) QoS or it will silently receive nothing.

Verified end-to-end via `capture_radarays_fixture static_cpu` /
`compare_radarays_fixture static_cpu` on 2026-07-21: `/radar/scan`,
`/radar/points`, `/radar/image` all sustain 4Hz, and the fixture now checks
for a strong center-column return within the known target's range window
(rows 5-20), not just "any nonzero image" — the old bridge's checks would
have passed even with completely wrong geometry.

### 9. Per-visual radar materials

`<radarays_material>` tags on a `<visual>` (velocity/ambient/diffuse/specular)
are now read and actually affect the physics, not just a hardcoded default.

This needed two new pieces of plumbing, both worth knowing about if this
breaks later:

- **Object id -> Entity.** `rmagine_gazebo_plugins`'s map system now records,
  for each geometry it adds to the Embree scene, which gz-sim `Entity` it
  came from (`EmbreeScene::add()` returns the Embree geom id — this was
  previously discarded). Exposed via `MapRegistry::SetObjectEntities` /
  `GetObjectEntities`, keyed the same as the map itself. `map_registry.hpp`
  is now also install-exported (`ament_export_include_directories`) so
  `radarays_gazebo_plugins` can include it as a real dependency instead of
  reaching into the sibling package's source tree.
- **Entity -> SDF `<radarays_material>` element.** gz-sim's ECM only
  exposes typed components for a visual (Name, Pose, Geometry, ...), not
  arbitrary custom child elements — so this can't be read off a component.
  Two more natural-looking approaches were tried and don't work in this
  gz-sim version: climbing `sdf::Element::GetParent()` from the plugin's
  own SDF element up to `<world>` dead-ends (the parent chain isn't live),
  and the world entity has no `SourceFilePath` component to re-read the
  original file from disk. What does work: gz-sim's own
  `/world/<name>/generate_world_sdf` service
  (`gz.msgs.SdfGeneratorConfig` -> `gz.msgs.StringMsg`) returns the full,
  current world SDF as a string (verified with `gz service -i -s ...`),
  which round-trips custom elements faithfully. Fetched lazily on the first
  `PostUpdate` after the map is built (not in `Configure()` — the service
  isn't guaranteed to be up that early), parsed once with `sdf::readString`,
  and cached.
- Material lookup itself: resolve the hit entity's model/link/visual name
  path via the ECM (Name + ParentEntity walk, all real components), then
  find the matching `<visual>` in the fetched SDF tree and check for
  `<radarays_material>` — same triple-nested walk Classic's
  `searchForMaterials()` did, just against a freshly-fetched DOM instead of
  Gazebo Classic's live `this->world->SDF()`.
- `material_id` scheme: `0` is air; for any hit object it's `object_id + 1`
  — no separate id-allocation table needed, and any object without a tag
  falls back to the shared default material automatically.

Verified live: added a test `<radarays_material>` tag (velocity 0, ambient
0.3, diffuse 0.7, specular 50 — deliberately different from the hardcoded
default) to `avz_map_visual` in `gz_static_radar_cpu.sdf`; debug logging
confirmed the exact values were fetched and matched to the right visual.
Fixture still passes with the tag in place.

### 10. Dynamic parameter reconfiguration

The ROS 2 equivalent of Classic's `dynamic_reconfigure` / `rqt_reconfigure`
(`RadarModelConfig`): 24 of the radar-physics params are now live-tunable
ROS 2 parameters on `radarays_embree_sensor_system`'s node — everything
Classic's dyncfg covered except `range_min`/`z_offset`/`scroll_image`/
`particle_noise`, which this port doesn't implement. Structural params
(frame/topic names, angular sampling, map key) intentionally stay SDF-only
— those change the sensor's shape, not just its tuning.

- Declared with `rcl_interfaces::msg::ParameterDescriptor` ranges where a
  bound is obvious (e.g. `signal_denoising` 0-3, probabilities 0-1) so a
  generic ROS 2 parameter UI shows sliders/steppers like Classic's
  `rqt_reconfigure` did, not free-form fields.
- `add_on_set_parameters_callback` validates a whole incoming batch
  *before* applying any of it — rclcpp applies parameter sets atomically,
  so one invalid value must not leave other, valid ones from the same
  batch half-applied.
- Changing `n_samples`, `beam_width_deg`, `beam_sample_dist`, or
  `beam_sample_dist_p_in_cone` invalidates the cached beam-sample cone so
  it regenerates next tick with the new values.

**A real bug found while testing this, worth remembering**: publishing
from this node worked fine without ever spinning it (DDS writes don't go
through the executor), which masked the fact that the node was never
spun at all. Parameter services (`set_parameters`, `list_parameters`, ...)
*do* need the executor — without a spin, `ros2 param set/list/get` against
this node just hangs forever with no error. Fixed with
`rclcpp::spin_some(node_)` once per `PostUpdate`. If a future addition to
this system needs any other ROS 2 service or subscription to actually be
responsive, remember it needs that same spin — it's easy to add a
service/subscription, see it "work" in that publishing never needed
spinning, and not notice it's dead until something tries to call it.

Verified live: launched the scenario, `ros2 param list` showed all 24
params, `ros2 param set n_reflections 7` then `get` confirmed the change,
`ros2 param set n_cells 0` / `signal_denoising 9` were rejected by the
declared ranges, `ros2 param set range_max -5.0` was rejected by the
custom validation (no declared range for that one), and `/radar/image`
kept publishing correctly throughout.

### 11. Dynamic (moving-geometry) radar scenario

A second scenario, mirroring `rmagine_gazebo_plugins`'s proven dynamic
lidar pattern instead of inventing a new one:

- `worlds/gz_dynamic_radar_cpu.sdf` — a 1m box oscillates ±1m along x
  (period 6s, via the existing `test_box_mover_system` plugin — no new
  C++ needed) directly ahead of the radar, range ~1.5-3.5m
- `launch/gz_dynamic_radar_cpu.launch.py`
- fixture: `capture_radarays_fixture dynamic_cpu` /
  `compare_radarays_fixture dynamic_cpu`,
  `testdata/radarays_harmonic/dynamic_cpu_fixture.json`

This exercises code paths the static scenario never touched: `map_revision_`
actually changing every frame (map system rebuilds ~30Hz to track the box),
`RefreshMaterials` re-triggering on every one of those rebuilds, and the
image's beam-sample cache surviving repeated (non-)invalidation. All
confirmed working — no bugs found here, unlike image generation and
dynamic reconfiguration.

The fixture harness needed one real extension: `summarize_image` only
tracked the *latest* frame's peak position, not a time series, so there was
no way to assert the image actually changes with motion (only that some
one frame looked plausible). Added `center_column_peak_row` to the
per-message probe (alongside the existing `nonzero_count`/`max_value`
series) and a `center_column_peak_row_span` in the finalized capture.
`compare_capture` gained matching *optional* dynamic checks
(`center_value_span_min`/`center_value_stddev_min` for scan,
`bbox_midpoint_x_span_min` for points, `center_column_peak_row_span_min`
for image) — additive, so the static fixture's existing stability checks
(`_max` variants, asserting things *don't* change) are untouched.

One non-bug worth recording so it isn't mistaken for one later: this
scenario's `/radar/image` is only ~18% nonzero (vs. ~100% for
`static_cpu`). That's correct, not broken — the office mesh in the static
world fills most of the field of view, so nearly every beam angle gets
some return; here there's only one narrow box in otherwise-empty space, so
most angles get no reflection at all. This port's ambient-noise model
scales with each column's own peak signal (ported verbatim from Classic),
so a column with zero real signal renders as pure black rather than
picking up a noise floor. Real numbers from the reference capture: scan
center-beam distance oscillates 1.5-3.5m (span 2.0, matching the box's
amplitude exactly), point cloud tracks the same range with y-extent ~1m
(the box's own width, roughly constant regardless of range), image
center-column peak row spans 76-179 (matching the same 1.5-3.5m in image
coordinates).

### 12. `radarays_ros` build/interface migration start

First real work on the fourth and last package. Scope: `package.xml` +
`CMakeLists.txt` conversion to `ament_cmake`, and getting the
message/service/action interfaces (`RadarMaterial(s)`, `RadarModel`,
`RadarParams`, `GetRadarParams`, `GenRadarImage`) building as real ROS 2
interfaces. See `src/radarays_ros/MIGRATION.md` (new — this package didn't
have one before) for the full breakdown.

Two things worth knowing if this is picked up again:

- The `.msg`/`.srv`/`.action` files needed **no content changes at all**.
  The instinct to rewrite `sensor_msgs/Image` as `sensor_msgs/msg/Image`
  in the `.action` file is wrong and breaks the build —
  `rosidl_generate_interfaces` expects the legacy `pkg/Type` syntax (no
  `msg/` segment) in these source files; it does the `msg/` qualification
  internally. Verified by breaking it this way first, then fixing it.
- `mesh_msgs` dependency dropped: only used by `mesh_publisher.cpp`, which
  wasn't even in the ROS 1 `CMakeLists.txt`'s build list, and has no ROS 2
  Jazzy release on this system anyway. `dynamic_reconfigure` dropped too,
  same reasoning as `radarays_gazebo_plugins`: ROS 2 parameters are the
  real equivalent, not a ported `dynamic_reconfigure` server.

The node logic (`Radar`/`RadarCPU`/`RadarGPU`, `radar_simulator`,
`ray_reflection_test`) is still entirely ROS 1 and not built. Important:
this is a **different codebase** from `radarays_gazebo_plugins`'s new
`radar_algorithms.hpp` (see "What Has Already Been Done" #8) — same
physics lineage, but `radarays_ros` simulates radar directly against a
static mesh for comparing against real recorded datasets (e.g. MulRan),
independent of any Gazebo loop, and its `Radar`/`RadarCPU` classes are
built around ROS1 node lifecycle + `dynamic_reconfigure`, not just the
math. Porting them is a separate effort from anything already done in
`radarays_gazebo_plugins`.

Verified: full clean rebuild of all four packages together, both
`radarays_gazebo_plugins` fixtures (`static_cpu`, `dynamic_cpu`) still
pass, and `ros2 interface show radarays_ros/action/GenRadarImage` (and the
other three interfaces) print correctly — not just "it compiles."

### 13. `radarays_ros`'s `Radar`/`RadarCPU`/`radar_simulator` node logic

Continuing straight on from #12: the actual node logic, not just the
interfaces. `radarays_ros` now has a real, running CPU radar simulator.

- `Radar`/`RadarCPU`: `ros::NodeHandle`/ROS1 `tf2_ros` -> `rclcpp::Node`/
  ROS2 `tf2_ros`. The physics loop itself is a faithful line-level port
  (same OMP-threaded per-thread-simulator structure, same fresnel/cone
  sampling/denoising/noise) — not a redesign, unlike
  `radarays_gazebo_plugins`'s `radar_algorithms.hpp` rewrite.
- `dynamic_reconfigure::Server<RadarModelConfig>` -> ~30 live ROS 2
  parameters (all of `cfg/RadarModel.cfg`), same validate-then-apply
  pattern as `radarays_gazebo_plugins`.
- Materials loading redesigned: ROS 1's array-of-dicts `XmlRpcValue`
  parameter has no ROS 2 equivalent (ROS 2 params have no array-of-object
  type). Fixed with a `materials_file` parameter pointing at a plain YAML
  file, parsed with `yaml-cpp`. `material_id_air`/`object_materials`
  stayed real ROS 2 parameters (scalar/array of plain numbers, which ARE
  representable).
- `radar_simulator.cpp`: ported only the code path actually used. The
  original's `main()` called `main_publisher`; the action-server variant
  was already commented out (and `main_action_server` doesn't exist as a
  function in the file), so no `rclcpp_action` port was needed. Also
  dropped a `pub_pcl` publisher that was advertised but never published to
  in the original (dead code).

**A real bug found by the runtime smoke test, not a compile check**:
`declare_parameter(name, default)` resolves a `--params-file`/`-p`
override at declaration time, but that resolved value wasn't being read
back into the corresponding member — only a later, explicit `ros2 param
set` did that (via the `onSetParameters` callback). Passing `n_cells: 512`
in a params file was silently ignored, producing the hardcoded default
(3424) instead. Caught by actually checking `/radar/image`'s real
dimensions after launching with a params file, not by the build succeeding.
Fixed by replaying the resolved values through `onSetParameters()` once,
right after declaring everything.

**Two independent physics copies now exist, and that's intentional**:
`radarays_gazebo_plugins`'s `radar_algorithms.hpp` and `radarays_ros`'s own
`Radar`/`RadarCPU`/`radar_algorithms.h/.cpp` implement closely related
physics separately. `radarays_gazebo_plugins` needed a working
implementation while `radarays_ros` was still fully ROS1, so depending on
it wasn't possible at the time. Unifying them now that `radarays_ros` has
a real ROS 2 build (the historically-correct dependency direction, per
this package's own README) is a legitimate future cleanup, but it's a
refactor of shipped, tested code in a different package — not attempted as
part of this migration step. Flagged in `src/radarays_ros/MIGRATION.md`
for whoever picks it up.

Verified live: full clean rebuild of all four packages, `radar_simulator`
launched against a real mesh (`radarays_gazebo_plugins/worlds/avz_no_roof.stl`,
reused rather than needing a new one) with a manually-published static TF
and a hand-written 2-material YAML file — `/radar/image` publishes at the
configured dimensions with plausible pixel content (100% nonzero, max near
the configured `signal_max`), and both `radarays_gazebo_plugins` fixtures
still pass unaffected.

Not done: `RadarGPU`/OptiX (deferred workspace-wide), `ray_reflection_test`
(lower-priority debug tool), real recorded-dataset validation (e.g.
MulRan) — only a synthetic mesh smoke test has been run so far.

### 14. Broader dynamic scenario coverage

`dynamic_cpu` (single translating box) covered geometric motion, but
nothing exercised rotation, multiple simultaneously-moving objects, or
materials on moving geometry. New scenario, `dynamic_multi_cpu`:

- **`test_box_mover_system` (`rmagine_gazebo_plugins`) gained rotation.**
  It only supported linear oscillation along an axis before. Added
  `angular_axis`/`angular_amplitude` SDF params (same `period`), additive
  and backward-compatible — default `angular_amplitude` is `0.0`, so every
  existing world using this plugin (including `rmagine_gazebo_plugins`'s
  own `gz_embree_dynamic.sdf`) is unaffected. Verified: that package's own
  `dynamic` fixture still passes after the change, unmodified.
- **New world**: `target_box_translate` (on boresight, same motion as
  `dynamic_cpu`'s box, now with a rough/diffuse `radarays_material`) plus
  `target_box_rotate` (off-boresight ~31°, rotating in place with zero
  translation, a specular `radarays_material`). Two objects moving
  differently, at the same time, each with their own material — the three
  things "more variety" was supposed to mean.
- **This is a real physics test, not just a geometry-visibility one**: the
  rotating box's specular material means its return strength should swing
  with incidence angle as it yaws (`back_reflection_shader` is
  incidence-angle-dependent) — confirmed via a real, measured signal:
  whole-image nonzero pixel count swings across a ~70k-pixel span over the
  capture window, in a scenario where the *only* thing changing is that
  box's orientation. The harness had no way to check this before — the
  only existing dynamic check (`center_column_peak_row_span`) only sees
  what's on boresight, i.e. only the translating box. Added two more
  optional dynamic checks, additive, mirroring the existing pattern:
  `bbox_midpoint_y_span_min` (points — a rotating, non-translating object
  barely moves in x, its y-extent/visible-footprint is where its motion
  actually shows up) and `nonzero_count_span_min` (image — whole-frame
  content change, not just center-column).
- Fixture (`dynamic_multi_cpu_fixture.json`) calibrated against real
  captured numbers, same discipline as every other fixture this session:
  scan center-beam distance still tracks 1.5-3.5m (only the translating
  box is on boresight, so this matches `dynamic_cpu` almost exactly); scan
  `finite_count` now varies more (132-203 vs. `dynamic_cpu`'s 54-122) from
  the rotating box's changing profile; points bbox y-midpoint span ~0.27m
  (small but real and new); image nonzero-count span ~70656 (vs.
  `dynamic_cpu`'s much smaller, noise-floor-driven variation).

Verified: two independent captures pass stably, plus `static_cpu` and
`dynamic_cpu` still pass unaffected, on a full clean rebuild of all four
packages.

### 15. CI-grade automation

`radarays_gazebo_plugins/scripts/ci_build_and_test.sh` is the single
source of truth for "build the workspace and check every regression
fixture" — runnable both locally and from CI, not CI-only logic that
nobody can reproduce on their own machine. It:

- resolves the workspace root from its own location (so it works
  regardless of where the workspace is checked out)
- applies the `libiomp5.so` workaround automatically if needed (see
  "Known Environment Issues") — GitHub-hosted runners have passwordless
  `sudo` by default, so this doesn't need a human in CI the way it did on
  a personal dev machine earlier this session
- builds all four packages, sanity-checks `radarays_ros`'s interfaces,
  then captures+compares all three `radarays_gazebo_plugins` fixtures
  (`static_cpu`, `dynamic_cpu`, `dynamic_multi_cpu`)

**Verified end-to-end, both directions**, not just "the happy path
compiles": ran it from a clean `build`/`install` wipe (exit 0, all three
fixtures pass); then deliberately corrupted `static_cpu_fixture.json`'s
`latest_max_value_min` to an impossible value (99999, above the uint8 max
of 255) and reran — the script correctly reported the specific failure,
still checked the other two fixtures (which correctly still passed,
proving one bad fixture doesn't mask or skip the rest), and exited
non-zero overall; restored the fixture and confirmed a clean pass again.
Also caught two real bugs in the script itself this way (not the
workspace code): a path-resolution off-by-one (miscounted directory
levels back to the workspace root) and a `set -u` incompatibility with
ROS 2's `setup.bash` (which references variables it doesn't guarantee are
set) — both would have made the script fail on literally every run,
caught by just running it.

A GitHub Actions workflow calling this script was added to each of the
four repos (`radarays_gazebo_plugins`, `radarays_ros`,
`rmagine_gazebo_plugins` — new `.github/workflows/build-and-test.yml` —
and `rmagine`, which already has its own standalone unit-test workflows,
so this one is separately named `radarays_workspace_integration.yml` to
avoid confusion with those). Each checks out itself plus the three sibling
repos (from their real GitHub URLs/branches — `rmagine_gazebo_plugins`
tracks `ros2`, the other three track `main`) into a workspace layout, then
runs the shared script.

**Important honesty note**: only the script itself has been verified by
actually running it, as described above. The GitHub Actions provisioning
around it (exact `ros-tooling/setup-ros` version, `ros-jazzy-ros-gz` and
other apt package names, whether `rosdep` resolves everything needed) has
**not** been exercised against a live GitHub Actions runner — there's no
way to trigger one from this environment. The YAML syntax and embedded
shell blocks were checked (`yaml.safe_load` + `bash -n` on every `run:`
block), which catches gross errors, but that is not the same as a real
green/red CI run. Also: none of this session's actual code changes have
been committed or pushed to any of the four repos (all still local,
uncommitted, in this same working tree) — so pushing these workflow files
alone wouldn't yet have anything meaningful to test against on `origin`.
The first real run of any of these workflows will be the actual
verification of the GitHub Actions side; treat that as still open until
it happens.

### 16. Unified the two independent physics copies

Flagged repeatedly this session as future work, now done. Split
`radarays_ros`'s `radar_algorithms.cpp` (the pure math: fresnel,
back-reflection shader, cone sampling, denoising) plus `radar_math.h`'s
`erfinvf` and `image_algorithms.h`'s `perlin_noise` out into its own
target, `radarays_core` — no ROS dependency at all, just `rmagine::core`
— and properly exported it (`ament_export_targets`, install the headers)
so it's consumable the normal ament way:
`find_package(radarays_ros)` + `target_link_libraries(... radarays_ros::radarays_core)`.
`radarays_gazebo_plugins` now depends on it directly; its own
from-scratch `radar_algorithms.hpp` (written earlier this session, back
when `radarays_ros` was still ROS1 and depending on it wasn't possible)
is deleted.

What stayed local to `radarays_gazebo_plugins`, deliberately:

- Its own material struct (plain 4 floats, resolved from SDF tags) — kept
  separate from `radarays_ros::msg::RadarMaterial` (a ROS message,
  resolved a completely different way). Forcing these together would
  have been a bigger, riskier change for no real benefit.
- A small local `MakeOnDnModel` helper — `radarays_core`'s `make_model()`
  hardcodes `range.max` to `1000.0`; `radarays_gazebo_plugins` needs it
  configurable (`range_max_`, an SDF param).

What's still NOT unified: `radarays_gazebo_plugins`'s batched-Embree-query
optimization (see item #8 above) has no counterpart in `radarays_ros` —
`radar_simulator` still ray-traces one angle at a time, per-thread via
OMP. Not attempted here since perf wasn't this item's goal, and touching
`radar_simulator`'s hot loop is exactly the kind of change that should get
its own dedicated verification pass, not ride along with a
supposed-to-be-behavior-preserving refactor.

**Verified as a real regression test, not just "it compiles"**: after
switching `radarays_gazebo_plugins` over, all three of its fixtures
(`static_cpu`, `dynamic_cpu`, `dynamic_multi_cpu`) still pass with the
exact same committed thresholds — meaningful, since the physics is
numerically identical either way; any subtle behavioral drift from the
switch would have shown up as a threshold miss. Also re-ran `radar_simulator`
against the same mesh/TF/materials smoke test as when it was first ported
— unaffected. Full clean rebuild of all four packages via
`ci_build_and_test.sh` passes.

One real build error surfaced and fixed along the way:
`radarays_ros/include/radarays_ros/image_algorithms.h` unconditionally
`#include`s `opencv2/core.hpp` at file scope (for functions
`radarays_gazebo_plugins` doesn't even use, only `perlin_noise` is needed)
— `radarays_gazebo_plugins` had no OpenCV dependency before this, so it
needed one added (`find_package(OpenCV)`, include dirs, link libs) purely
to satisfy that header's top-of-file include, not because any actual
OpenCV symbol gets called.

### 17. Ported `ray_reflection_test`

Debug/viz tool from `radarays_ros`: shoots one ray (or a 360-degree fan)
from the origin, recurses through `fresnel()` for `n_reflections` bounces,
publishes the traversal as a `visualization_msgs/msg/Marker` LINE_LIST.
Same faithful-line-level-port style as `Radar`/`RadarCPU`:
`dynamic_reconfigure::Server<RayReflectionConfig>` -> ROS 2 parameters with
range descriptors and the same validate/apply/replay-on-declare pattern;
`object_materials`/`velocities`/`material_id_air` -> plain ROS 2 array/
scalar parameters (no YAML detour needed here — unlike the radar materials,
these were always flat arrays in ROS 1, not an array-of-dicts); ROS 2
`tf2_ros` API; `rclcpp::spin_some()` each loop iteration. Added
`visualization_msgs` as an explicit dependency (was implicit via `roscpp`
in ROS 1).

Wired into `CMakeLists.txt` next to `radar_simulator`, under the same
`rmagine::embree`-available guard.

**Verified with a real runtime smoke test**, not just a compile check:
ran against the same `avz_no_roof.stl` mesh as `radar_simulator`, with a
static `map` -> `navtech` TF. Confirmed via `ros2 topic echo`:
`n_reflections: 1` produces exactly 2 line segments (matching
`n_reflections + 1` passes) with point coordinates matching the configured
`ray_yaw` direction and the mesh's real intersection distance; with
`spinning: true`, the hit range changes smoothly frame to frame, proving
it's actually ray-casting against the mesh geometry rather than emitting a
placeholder.

### 18. GPU/OptiX Harmonic port — first milestone (`rmagine_gazebo_plugins`)

Of the remaining items (MulRan real-dataset validation, CI-trigger,
OptiX/GPU parity), only GPU/OptiX parity was actually actionable in this
environment — MulRan needs external data access this session doesn't have,
and triggering the GitHub Actions workflows needs push access this session
doesn't have. Scoped to the smallest real first step: a validated Harmonic
OptiX baseline in `rmagine_gazebo_plugins`, mirroring how the CPU/Embree
migration started — not the full `radarays_ros`/`radarays_gazebo_plugins`
GPU radar path (deliberately deferred; see below).

**Toolchain, from scratch, on this machine:**
- GPU present (NVIDIA RTX 500 Ada, driver 580.173.02) but no CUDA toolkit
  and no OptiX SDK. Installed `nvidia-cuda-toolkit` via apt (user ran it —
  needs sudo password this session doesn't have).
- OptiX SDK is not apt-installable — license-gated, manual download from
  NVIDIA (developer login). User provided **9.1.0** first: built, but
  `optixInit()` failed with `OPTIX_ERROR_UNSUPPORTED_ABI_VERSION` (SDK ABI
  118 vs. whatever this driver's OptiX runtime component actually
  supports — a real driver/SDK version mismatch, not a code bug). User
  then provided **7.5.0** and **7.6.0**; **7.5.0 (ABI 60) worked** —
  matches `rmagine`'s own hardcoded driver-compatibility table in
  `OptixContext.cpp`, which only lists entries up to OptiX 7.5.0, meaning
  this codebase was written/tested against the 7.x SDK generation, not 9.x.
- Verified `rmagine::optix` itself first, isolated from any gz-sim code:
  rebuilt `rmagine` against the 7.5.0 SDK, then ran rmagine's own OptiX
  test binaries (`rmagine_tests_optix_simulation_{spherical,o1dn,pinhole,ondn}`,
  `rmagine_tests_optix_correction_rcc`) directly against the real GPU — all
  passed, including a GPU-vs-CPU cross-check whose statistics matched
  almost exactly. This confirmed the OptiX *backend* works before writing
  any new integration code on top of it.

**New code**, mirroring the existing Embree Harmonic System pair
mechanically (same SDF params, same lifecycle) but for GPU:
- `OptixMapRegistry` (`optix_map_registry.hpp`) — a **separate** singleton
  from `MapRegistry`, not an extension of it. `MapRegistry::Instance()` is
  a function-local-static "singleton" that only stays unified across the
  Embree map/sensor `.so`s because gz-sim loads plugins with symbol
  interposition (`RTLD_GLOBAL`-style) and the class layout is identical in
  every `.so` that includes the header. Adding OptiX-only members to that
  same header would make its layout differ depending on whether a given
  `.so` was compiled with OptiX available — an ODR violation across
  already-built `.so`s. A parallel header sidesteps this entirely.
- `rmagine_optix_map_system` / `rmagine_optix_sensor_system` — line-level
  ports of `rmagine_embree_map_system` / `rmagine_embree_sensor_system`
  swapped to `OptixMap`/`OptixScene`/`SphereSimulatorOptix`, gated behind
  `TARGET rmagine::optix` in `CMakeLists.txt`.

**Two real bugs found and fixed, verified against the actual GPU (not just
a compile check)** — see `rmagine_gazebo_plugins/README.md`, "OptiX / GPU
Harmonic port" for the full detail:
1. `SphereSimulatorOptix`'s single-Transform *returning* `simulate<ResT>(Tbm)`
   overload segfaults (crashes in `SphericalModel::getHeight()` with a bad
   `this` — the result bundle is never sized before the kernel launch).
   Reproduced with a minimal standalone repro outside gz-sim to isolate it
   from any gz-sim-specific cause. Fixed by using the pre-sized void
   `simulate<ResT>(Tbm, ret)` overload instead — the one rmagine's own test
   suite actually exercises.
2. Box/sphere/cylinder primitives silently render at **unit size**
   regardless of configured SDF dimensions — `setTransformAndScale(M)`
   decomposes `M` into both transform AND scale, clobbering a shape's
   already-set size back to `(1,1,1)`. Proven empirically (not just by code
   reading): a 0.2 box and a 1.0 box raytraced to the identical hit
   distance until fixed. Fixed on the OptiX side with `setTransform(T)`
   instead (transform-only, doesn't touch scale). **Confirmed present in the
   CPU/Embree map system too and fixed there as well** (same one-line
   change in `rmagine_embree_map_system.cpp`) — see item #19 below for why
   that didn't need a fixture re-capture after all.

**Runtime-verified end-to-end**: new world `worlds/gz_optix_baseline.sdf`
(straight port of `gz_embree_baseline.sdf` to the OptiX systems), launched
via `gz sim -s -r --headless-rendering`. `/scan` and `/points` publish over
ROS 2; GPU-computed hits on the fixed `1x1x1` box: 76/400 finite,
`2.500m`-`2.549m` — matches the analytically expected front-face distance
exactly.

**Also runtime-verified**: `worlds/gz_optix_dynamic.sdf` (straight port of
`gz_embree_dynamic.sdf` — same `test_box_mover_system`-driven oscillating
box). Over two full 6s oscillation periods the reported min hit range swept
smoothly `1.75m -> 3.28m -> 1.75m`, tracking the box's `±0.75m` motion
around its `2.5m` center distance with no jumps or NaN gaps — confirms the
GPU map-rebuild-on-change + simulator-refresh-on-new-revision cycle works,
not just the static case.

**Not done**: the actual `radarays_ros`/`radarays_gazebo_plugins` GPU radar
path (`RadarGPU.cpp`, GPU-backed `radarays_embree_sensor_system`
equivalent) — this milestone only gets `rmagine_gazebo_plugins` to the same
"validated GPU baseline" point the CPU path started from (static + dynamic
scene, plus the scale bug fixed on both backends now).

**Note**: a `pkill -f "gz sim"` used to tear down a test instance during
this milestone also killed an unrelated, pre-existing `ocean_world.sdf` USV
simulation from another session — broad pattern kills aren't safe in this
workspace; ask before killing processes rather than pattern-matching.

### 19. Fixed the primitive-shape scale-clobber bug on the CPU/Embree side too

Applied the same one-line fix from item #18 (`setTransformAndScale(M)` ->
`setTransform(T)`) to `rmagine_embree_map_system.cpp`. Before assuming a
fixture re-capture was needed, checked what geometry every existing
fixture-backing world actually feeds through the map: `gz_embree_baseline`/
`gz_embree_dynamic`'s boxes are `1x1x1`/`0.2x0.2x0.2` but the `0.2` one is
the ignored sensor's own visual, never entering the map; `radarays`'s
`static_cpu` world's only mapped geometry is a mesh at `<scale>1 1 1</scale>`
(its non-unit `0.4x0.3x0.2` box is likewise the ignored sensor's own
visual); `dynamic_cpu`/`dynamic_multi_cpu`'s target boxes are all `1x1x1`.
Since the bug only manifests as wrong *size* and every traced shape in
every existing world is already unit-scaled (by construction, which is
exactly why the bug went unnoticed originally), the fix is mathematically
a no-op for all of them.

Verified this prediction empirically rather than trusting it blindly: reran
all 5 existing fixtures (`rmagine_gazebo_plugins`'s `baseline`/`dynamic` via
`capture_embree_fixture`/`compare_embree_fixture`, and all three
`radarays_gazebo_plugins` fixtures via `ci_build_and_test.sh`, which also
did a full 4-package rebuild). All 5 passed against their existing
committed values, unchanged — no re-capture needed after all.

One environment snag hit along the way, unrelated to the fix itself: the
`libiomp5.so` workaround symlink (`/usr/local/lib/libiomp5.so`) existed but
the dynamic linker couldn't resolve it — `ldconfig`'s cache was stale (this
session has no passwordless `sudo` to refresh it) and the symlink predates
the last cache rebuild. `LD_LIBRARY_PATH=/usr/local/lib:...` (checked
before the cache in glibc's search order) works around it without needing
`ldconfig` — `ci_build_and_test.sh` already does exactly this
(`export LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH:-}"`), so this
only bit the two standalone `capture_embree_fixture`/`compare_embree_fixture`
invocations run outside that script.

### 20. `radarays_ros`'s GPU radar path (`RadarGPU`) ported

First half of "the actual radar GPU path" (the other half is a GPU-backed
`radarays_gazebo_plugins` sensor system, not started — see below). Ported
`RadarGPU.cpp`, `radar_algorithms.cu`/`.cuh`, `image_algorithms.cu`/`.cuh`
to ROS 2, and added a new `radar_simulator_gpu` node (mirrors
`radar_simulator`, swapped to `RadarGPU`/`OptixMap`). Gated behind
`TARGET rmagine::optix AND CMAKE_CUDA_COMPILER` — doesn't affect the
CPU-only build path at all when no OptiX SDK is present.

Beyond the mechanical ROS 1 -> ROS 2 substitutions already established by
`RadarCPU`'s port (`ros::NodeHandle` -> `rclcpp::Node::SharedPtr`, etc.),
two real gaps surfaced and were fixed:

- `radar_algorithms.cuh` referenced a `RadarMaterial` type that didn't
  exist anywhere in this checkout. In ROS 1, `RadarMaterial` *was* the
  message type (ROS 1 messages are plain structs, safe to memcpy to the
  GPU); the ROS 2 message class isn't trivially-copyable, so a separate
  plain POD `RadarMaterial` struct was added to `radar_types.h` (same 4
  float fields), with explicit field-by-field conversion from
  `msg::RadarMaterial` at the ROS boundary. Also dropped a dead
  `#include <radarays_ros/RadarParams.h>` in the same header — the file
  doesn't exist and nothing in the header actually uses that type.
- `image_algorithms.cuh` unconditionally required OpenCV's CUDA module
  (`opencv2/core/cuda.hpp`) for one `fill_perlin_noise(cv::cuda::GpuMat&, ...)`
  overload that `RadarGPU.cpp` never calls (only the `rm::MemView`-based
  overloads are used). System OpenCV here has no CUDA module at all, so
  this would have been a hard, unnecessary build blocker — deleted the
  unused overload and its kernel from both the header and `.cu` file
  rather than adding a real OpenCV-CUDA dependency for dead code.

`preBuildProgram<ResT>()` is still templated in this rmagine version
(checked the header rather than assuming) — ported unchanged. CUDA needed
`enable_language(CUDA)` added to `radarays_ros`'s own `CMakeLists.txt` —
rmagine enabling it for its own build doesn't carry over to a different
colcon package/CMake project.

**Runtime-verified against the real GPU**: ran `radar_simulator_gpu`
against the same `avz_no_roof.stl` mesh/TF/materials setup as the CPU
`radar_simulator` smoke test. 29 consecutive `simulate()` calls succeeded
at ~5-7ms GPU compute time each. Subscribed to `/radar/image` directly
(not just checked message shape) and computed real pixel statistics:
512x400 `mono8`, 99.87% nonzero, min 0 / max 132 / mean ~14.5 — physically
plausible, non-degenerate output.

**Not done at the time**: the actual Gazebo-side integration — see item
#21, done as an immediate follow-on.

### 21. `radarays_gazebo_plugins`'s GPU radar sensor system ported — full GPU radar path now done

The other half of #20. New `radarays_optix_sensor_system`, mirroring
`radarays_embree_sensor_system` almost exactly (same SDF/ROS 2 parameter
surface, same per-visual `<radarays_material>` lookup via
`generate_world_sdf`, same denoising/ambient-noise/normalization math) —
the physics loop is the real difference, running radarays_ros's GPU CUDA
kernels (`move_waves`/`signal_shader`/`fresnel_split`) in a multi-pass,
doubling-buffer loop generalized from `RadarGPU`'s hardcoded-3-pass
approach to the full `n_reflections` SDF range (1-20, with a debug-log
warning above 6 passes since VRAM cost doubles per pass with no
compaction).

**Split `radarays_ros`'s new `radarays_gpu` target further**, into
`radarays_gpu_core` (the pure CUDA kernels + `RadarMaterial`, no
`Radar`/ROS coupling) vs `radarays_gpu` (keeps `RadarGPU.cpp`, needs
`radarays` for the `Radar` base class). Tried exporting `radarays_gpu` as
one target first — CMake's `install(EXPORT)` correctly refused, since
`radarays_gpu` links `radarays`, which itself links the rosidl typesupport
target (not meant to be re-exported through another target's link
interface). Splitting mirrors the existing `radarays_core`/`radarays`
split exactly, and `radarays_gazebo_plugins` only ever needed the
core half anyway.

Also added `ObjectEntityMap` tracking to `rmagine_optix_map_system.cpp`/
`OptixMapRegistry` (geometry id -> Entity), which the OptiX baseline
milestone (#18) hadn't needed yet — this system's per-visual materials
lookup requires it, same as the CPU/Embree side's `MapRegistry` already
had.

**Two real bugs found and fixed, both root-caused with a standalone repro
built outside gz-sim entirely** (rules out any gz-sim/cross-.so
involvement, and iterates far faster than a full sim launch each time):

1. The materials-not-ready-yet fallback path built a **zero-length**
   `object_materials` GPU array while the map already had real geometry (a
   valid object id `0`) — `signal_shader`/`fresnel_split` then read
   `object_materials[0]` out of an empty device buffer: `cudaErrorIllegalAddress`.
   Fixed by always sizing the GPU material buffers from the map's actual
   object count (`OptixMapRegistry::GetObjectEntities()`), independent of
   whether the `<radarays_material>` SDF fetch has succeeded yet — that
   fetch still gates *when* materials stop being all-default, not
   *whether the buffers are validly sized*.
2. **A real, previously-undiscovered bug in `rmagine_optix_map_system.cpp`
   itself** (from milestone #18, not this one): its MESH case adds the
   `OptixMesh` directly to the top-level scene, unlike the box/sphere/
   cylinder cases right above it, which already wrap in an `OptixInst` via
   `geom_scene->instantiate()`. This was the *first* time the MESH case had
   ever actually been exercised end-to-end — every earlier GPU milestone
   (#18, #19) only used `<box>` geometry. Direct-add is broken in this
   rmagine/OptiX version: `simulate()`/`move_waves()`/`signal_shader()` all
   run and `cudaDeviceSynchronize()` cleanly, but the next device-to-host
   copy throws the same illegal-address error. A standalone repro
   isolated it to exactly one difference: wrapping the mesh in an
   `OptixInst` (matching what `rmagine::import_optix_map()` and the
   primitive-shape cases already do) fixes it outright. Fixed in
   `rmagine_optix_map_system.cpp`'s MESH case. See
   `rmagine_gazebo_plugins/README.md` for the fuller writeup (bug #3
   there).

Both bugs were confirmed NOT to be a "two OptiX simulators fighting over
one CUDA context" problem, despite an early red herring (`rmagine_optix_sensor_system`
+ `radarays_optix_sensor_system` running together produced the exact same
crash before either fix) — after fixing both, the two systems run
together in the same world with no issue at all, so that theory was wrong.

**Runtime-verified against the real GPU**: new world
`worlds/gz_static_radar_gpu.sdf` (straight port of
`gz_static_radar_cpu.sdf`) against the same `avz_no_roof.stl` mesh with a
real `<radarays_material>` tag. Subscribed to `/radar/image` directly
(matching `SensorDataQoS` — a plain-reliable subscription silently
receives nothing) and computed real pixel statistics: 1024x400 `mono8`,
100% nonzero, min 3 / max 255 / mean ~20.6 — physically plausible,
non-degenerate output. Also verified `rmagine_optix_sensor_system` +
`radarays_optix_sensor_system` running together (matching
`gz_static_radar_cpu.sdf`'s own CPU-side precedent): `/radar/scan`,
`/radar/points`, and `/radar/image` all publish real data simultaneously.

**This completes the GPU/OptiX radar path** — the last item from the
"radarays GPU radar path" menu. Remaining work across the whole workspace
is: `radarays_ros`'s real-dataset validation (blocked on data access), and
actually triggering the GitHub Actions CI workflows on a real runner
(blocked on push access).

### 22. Wired fixtures into `colcon test`

With both remaining items (MulRan validation, live CI run) blocked on
external access this session doesn't have, the next actionable, no-blocker
item was finding what's left that doesn't need a dataset or push access.
A quick research pass over every package's own docs (their `README.md`
files lag behind this handoff and still said things like "still missing:
OptiX/GPU path" even though items #18-21 already finished that) surfaced:
the fixture capture/compare harnesses exist and work, but were only ever
invoked by hand or via `ci_build_and_test.sh` — never registered as actual
`colcon test`/CTest tests. Distinct from "trigger CI on GitHub" (blocked):
this needed no external access at all.

Added `ament_add_test()` registrations (`if(BUILD_TESTING AND
RMAGINE_GZSIM_PORT/RADARAYS_GZSIM_PORT)`, `find_package(ament_cmake_test
REQUIRED)`) in both `rmagine_gazebo_plugins` (`embree_fixture_baseline`,
`embree_fixture_dynamic`) and `radarays_gazebo_plugins`
(`radarays_fixture_static_cpu`, `radarays_fixture_dynamic_cpu`,
`radarays_fixture_dynamic_multi_cpu`). Each test runs
`ros2 run <pkg> capture_..._fixture <world> && ros2 run <pkg>
compare_..._fixture <world>` (matching how these scripts are invoked
everywhere else in the workspace — `ros2 run`, not a bare PATH lookup,
since the installed script lives under `lib/<pkg>`, which isn't on PATH by
itself), with `APPEND_LIBRARY_DIRS "/usr/local/lib"` for the `libiomp5.so`
workaround (same one `ci_build_and_test.sh` needs) and per-world
`TIMEOUT`s (120s for the short static/baseline worlds, 180s for the
longer dynamic ones, based on observed capture durations plus gz-sim
startup/teardown overhead).

One real snag: the first attempt failed with `run_test.py`'s own error
`"The test did not generate a result file"` even though the underlying
capture+compare printed success and returned exit code 0 — `ament_add_test`
by default assumes the test command generates its own JUnit-style result
file (like a gtest binary would); a plain shell script doesn't, so
`run_test.py` couldn't find one and reported a fake failure regardless of
the real exit code. Fixed by adding the `GENERATE_RESULT_FOR_RETURN_CODE_ZERO`
option, which tells it to synthesize a pass/fail result from the process's
own return code instead.

**Verified the full user-facing workflow, not just raw `ctest`**: ran
`colcon test --packages-select rmagine_gazebo_plugins
radarays_gazebo_plugins` end-to-end (all 5 tests actually launch gz-sim,
capture, and compare against the committed fixtures — this doesn't skip
or mock any of the real work) and `colcon test-result --all`, which
correctly aggregates all 5 into `Summary: 10 tests, 0 errors, 0 failures,
0 skipped` (10, not 5, because each package's own `Testing/*/Test.xml`
duplicates the per-test result files in the count — expected `colcon
test-result` behavior, not a bug). Added `<test_depend>ament_cmake_test</test_depend>`
to both packages' `package.xml`.

### 23. MulRan dataset work — tooling done, dataset itself still blocked

With both remaining items from #22's writeup (MulRan validation, live CI
run) still external-access-blocked, the user asked to pursue MulRan
anyway, starting with scoping what it actually requires before acquiring
anything. That scoping (see `src/radarays_ros/MIGRATION.md`, "MulRan
dataset work" for the full writeup) turned into real, hands-on assessment
work once the user chose to keep pushing rather than stop at "it's
blocked":

- Cloned three external repos into `src/` (`lvr2`, `rmcl`, `micp_experiments`)
  plus later `file_player_mulran`, one at a time with confirmation between
  each, per explicit instruction after an earlier multi-repo clone attempt
  was interrupted mid-flight.
- One real environment snag along the way: the interrupted clone left
  `micp_experiments`'s git index stale relative to its (fully intact,
  nothing lost) working tree — `git status` showed 76 files as deleted
  that were still physically present on disk. Since it was a zero-investment
  fresh clone (not the user's own work), the safest fix was deleting and
  re-cloning cleanly rather than attempting index surgery.
- **`lvr2`** (mesh reconstruction) and **`rmcl`/`rmcl_msgs`/`rmcl_ros`**
  (MICP-L localization, successor to the original implementation) both
  **build clean on this ROS 2 Jazzy workspace** — only missing system dep
  across both was `libgsl-dev`. Real, working tools (`lvr2_reconstruct`,
  `micp_localization_node`), not just theoretical compatibility.
- **`micp_experiments`**: its `micp_mulran` (ROS 1/catkin) is a dead end
  without a full port. Its `micp_mulran2` (nominally ROS 2) needed two real
  `find_package` ordering fixes to even configure, then hit a genuine API
  version-skew against current `rmcl` (old `Corrector`-based API, since
  refactored into `Updater`/`Pipeline` classes) — confirmed via `rmcl`'s
  own `docs/MICPL.md`, which explicitly says `micp_experiments` is
  "primarily compatible with the ROS 1 version" and points at `rmcl_ros`'s
  current, generic `micp_localization_node` instead. Correctly did not
  force-port stale third-party research code against the author's own
  explicit guidance.
- **`file_player_mulran`** (replays MulRan's raw files as ROS topics): also
  ROS 1/catkin-only, and a 1563-line Qt5 GUI app (play/pause/scrub,
  LiDAR/IMU/GPS via PCL) — porting all of that for a task that only needs
  radar images + ground truth would mostly be wasted effort. Instead,
  reimplemented just the relevant piece of its own
  `ROSThread::SaveRosbag()` as a new, small script in `radarays_ros`:
  `mulran_radar_to_bag.py` (`ros2 run radarays_ros mulran_radar_to_bag`),
  converting a MulRan sequence's radar-polar PNGs + `global_pose.csv`
  ground truth into a standard `ros2 bag` (`/Navtech/Polar`
  `sensor_msgs/msg/Image`, `/gt` `nav_msgs/msg/Odometry`) — replayable with
  plain `ros2 bag play`, no Qt/GUI porting needed.

**Verified the new tool against synthetic data built to match MulRan's
real file layout** (no real MulRan data was available to this session —
see below): generated 3 fake radar-polar PNGs + a matching
`global_pose.csv`, ran the tool, confirmed via `ros2 bag info` (correct
topics/types/counts/timestamps) and by deserializing every message back
out (correct image dimensions/encoding/data length; correct position
values; correct quaternion derived from the identity rotation matrix).

**What's actually still blocking real validation**: only the dataset
itself, not tooling. MulRan requires a registration/access request for the
real sequences (a small no-registration "ParkingLot" sample exists but
lacks IMU/GPS); this session had no way to complete that request (tied to
personal/institutional info). Once real data is available: reconstruct a
mesh via `lvr2_reconstruct`, convert radar+ground-truth via
`mulran_radar_to_bag`, run `radar_simulator`/`radar_simulator_gpu` against
the mesh with ground-truth-driven TF, compare against the real
`/Navtech/Polar` stream.

## What We Are Doing Right Now

The migration has moved past backend proof-of-concept and is now in the
`radarays_gazebo_plugins` integration phase.

Current working interpretation:

- `rmagine_gazebo_plugins` should be treated as the validated Harmonic backend baseline.
- `radarays_gazebo_plugins` is the active migration front.
- The current goal is no longer “prove Harmonic Embree works at all.”
- The current goal is “replace Classic `radarays` functionality incrementally on top of the validated Harmonic backend.”

In practice, that means:

- keep generic map / scan / point responsibilities delegated to `rmagine_gazebo_plugins`
- add missing radar-specific outputs and semantics in `radarays_gazebo_plugins`
- avoid rewriting generic scene/sensor lifecycle logic twice

## Important Current Boundaries

These are the most important limitations to remember on Monday.

### 1. `radarays_gazebo_plugins` is only partially migrated

What exists:

- ROS 2 / Harmonic build path
- one real Harmonic scenario
- delegated `/radar/scan`
- delegated `/radar/points`
- real Harmonic-native `/radar/image` generation (multi-bounce Fresnel,
  cone-sampled beams, denoising, ambient noise)
- per-visual `radarays_material` SDF parsing (velocity/ambient/diffuse/specular)
- dynamic parameter reconfiguration (24 ROS 2 parameters, live-tunable)
- a second, dynamic (moving-geometry) scenario with its own regression fixture

What does not exist yet:

- migrated OptiX / GPU path

### 2. The current `/radar/image` has real physics, materials, live tuning, and dynamic scenarios now

The image publisher runs actual radar wave-propagation physics against the
delegated Embree map, not a derived-from-scan bridge; per-visual
`<radarays_material>` tags are read and applied; the physics params are
live-tunable ROS 2 parameters; and two moving-target scenarios (single
object translating; two simultaneous objects, one translating one
rotating, each with its own material) confirm all of that keeps working
as the map changes every frame (see "What Has Already Been Done" #9, #10,
#11, #14). Objects without a material tag fall back to one shared default
material.

So:

- good enough to validate real radar-image semantics, including
  per-surface material differences, live tuning, and dynamic geometry
  (translation and rotation, single- and multi-object), on three scenarios
- not good enough for “Classic parity achieved” (GPU path still missing)

### 3. `radarays_ros`'s CPU node logic is done; GPU and one debug tool aren't

Build system, interfaces, and the CPU `Radar`/`RadarCPU`/`radar_simulator`
path are migrated and runtime-verified (see "What Has Already Been Done"
#12, #13). Not done: `RadarGPU`/OptiX, `ray_reflection_test`, real
recorded-dataset validation (only a synthetic mesh smoke test so far).

That means:

- the broader old radar stack has a real, working ROS 2 CPU path now
- `radarays_gazebo_plugins` doesn't depend on any of this — its own
  `radar_algorithms.hpp` is a separate, already-working port — so nothing
  there was ever blocked by `radarays_ros`'s remaining work, and still
  isn't

## Short-Term Next Steps

These are the recommended next tasks for the next session.

### 1. ~~Freeze the `/radar/image` bridge with a regression check~~ — done

Already implemented in `scripts/radarays_fixture_harness.py` and
`testdata/radarays_harmonic/static_cpu_fixture.json`: width, height,
nonzero pixel count, max pixel value, and a center-column nonzero check
(covers the "representative slice" suggestion). Re-verified live on
2026-07-21 after the `libiomp5` fix — `capture_radarays_fixture static_cpu`
followed by `compare_radarays_fixture static_cpu` passes cleanly.

### 2. ~~Start replacing the image bridge with true radar-image generation~~ — done

`radarays_embree_sensor_system` now generates the polar image directly from
ray-traced wave physics (see "What Has Already Been Done" #8). CPU/Embree
only, static scenario, real Fresnel/cone-sampling/denoising/noise — the
scan-derived bridge and its Python node are retired from the launch/build.

### 2b. ~~Per-visual radar materials~~ — done

`<radarays_material>` tags now parse and affect the physics (see "What Has
Already Been Done" #9). Objects without a tag still use the shared default.

### 2c. ~~Dynamic parameter reconfiguration~~ — done

24 radar-physics params are live-tunable ROS 2 parameters with range
validation (see "What Has Already Been Done" #10). Structural params
(frame/topic names, angular sampling, map key) are still SDF-only by
design — those aren't "tuning knobs," they change the sensor's shape.

### 3. Audit exactly which pieces can be reused from Classic radar logic

The old Classic code mixes:

- radar simulation core
- radar material lookup
- dynamic reconfigure
- ROS 1 image publication
- Classic custom sensor registration

The next session should decide more explicitly:

- what logic can be ported as plain C++/algorithm code
- what logic is tightly bound to Classic and should be replaced instead

Recommended approach:

- do not port Classic registration/publishing structure
- do port or re-express only the radar-specific computation that still matters

### 4. Keep the current `radarays` scenario as the first integration milestone

Do not jump to a bigger scenario yet.

Use the current `gz_static_radar_cpu.sdf` path as the reference while bringing over:

- image behavior
- material behavior
- later, dynamic behavior

## Longer-Term Next Steps

### 1. ~~Real Harmonic-native radar image generation~~ — done

`radarays_embree_sensor_system` does real polar image synthesis (not derived
from `LaserScan`) with correct frame/timestamp handling. See "What Has
Already Been Done" #8.

### 2. ~~Radar materials in Harmonic~~ — done

Per-visual `<radarays_material>` tags are parsed and resolved against the
delegated Embree map's object ids (via `MapRegistry`'s new object id ->
Entity mapping) and fed into the same fresnel/back-reflection shader
Classic used. See "What Has Already Been Done" #9 for how the SDF DOM
itself gets fetched (gz-sim's `generate_world_sdf` service, since neither
ECM components nor `GetParent()` climbing exposed it).

### 3. ~~Dynamic radar behavior in real `radarays` scenarios~~ — first one done

`gz_dynamic_radar_cpu.sdf` (oscillating box) confirms image output changes
consistently with moving geometry, with a regression fixture
(`dynamic_cpu`). See "What Has Already Been Done" #11. Only one dynamic
scenario exists so far — more varied ones (multiple moving objects,
rotation not just translation, objects with materials) are still open if
more coverage is wanted.

### 4. `radarays_ros` migration

After `radarays_gazebo_plugins` has a meaningful Harmonic-native radar path,
`radarays_ros` becomes the next clear migration layer.

Expected future work:

- `catkin` to `ament_cmake`
- message/service/action migration
- ROS 2 node/launch migration
- reconnecting the downstream radar processing graph

### 5. OptiX / GPU parity

Not a near-term priority.

The right sequence is:

1. CPU / Embree path first
2. real scenario integration
3. radar-specific parity
4. then GPU/OptiX parity

So GPU support is still part of the migration plan, but it is intentionally not the
current focus.

### 6. CI-grade automation

Eventually the migration should not depend on manual local checks only.

Future target:

- keep the existing fixture harnesses
- run them in CI where feasible
- expand from toy backend fixtures to real `radarays` integration checks

## Recommended Restart Point

If restarting, begin in this order:

1. Read:
   - this file
   - `src/radarays_gazebo_plugins/MIGRATION.md`
   - `src/radarays_gazebo_plugins/README.md`
2. Reconfirm current working baseline — the fast way:
   - if `libiomp5.so` is missing on the machine, apply the fix in "Known
     Environment Issues" above first (on a personal dev machine this needs
     a human for the `sudo` step; CI runners have passwordless `sudo`)
   - run `src/radarays_gazebo_plugins/scripts/ci_build_and_test.sh` from
     the workspace root — it builds all four packages and checks all
     three `radarays_gazebo_plugins` fixtures (`static_cpu`, `dynamic_cpu`,
     `dynamic_multi_cpu`) in one shot; exit 0 means the baseline is intact
   - equivalently (now that #22 is done): `colcon test --packages-select
     rmagine_gazebo_plugins radarays_gazebo_plugins` then `colcon
     test-result --all` — runs the same fixtures as real, aggregated
     `colcon test` results instead of a standalone script
   - the slower, piece-by-piece way (useful if that script fails and you
     need to isolate which step broke): build each package individually,
     run each fixture's `capture_radarays_fixture`/`compare_radarays_fixture`
     by hand, `ros2 interface show radarays_ros/action/GenRadarImage`, and
     launch `radarays_ros`'s `radar_simulator` against a real mesh (see
     `src/radarays_ros/MIGRATION.md` for the params-file shape)
3. Start the next real feature. The full GPU/OptiX radar path is done
   (#18-21), the fixture harnesses are wired into `colcon test` (#22), and
   MulRan validation tooling is built and verified (#23: `lvr2`, `rmcl`,
   `mulran_radar_to_bag`, all cloned into `src/` and confirmed working) —
   CPU and GPU are at parity everywhere in this workspace, the regression
   suite runs the normal ROS 2 way, and MulRan validation is genuinely
   just waiting on data, not more engineering. What's left is: getting real
   MulRan data (registration-gated, tied to personal/institutional info —
   see #23), or actually triggering the GitHub Actions CI workflows on a
   real runner (blocked on push access this session didn't have). Both are
   genuinely blocked by external access, not by remaining engineering
   work — see "What Has Already Been Done" #15's honesty note on the CI
   workflows never having run for real.

## Practical Summary

Right now the migration status is:

- backend prototype: working
- toy-world backend validation: working
- first real `radarays` Harmonic scenario: working
- first `radarays` regression fixture: working
- real Harmonic-native `/radar/image` generation: working (scan-derived
  bridge retired)
- per-visual radar materials: working (`<radarays_material>` SDF parsing)
- dynamic parameter reconfiguration: working (24 live-tunable ROS 2 params,
  `radarays_gazebo_plugins`)
- dynamic radar scenarios: working (`dynamic_cpu` — single translating
  object; `dynamic_multi_cpu` — two simultaneous objects, translation +
  rotation, each with its own material; `test_box_mover_system` gained
  rotation support along the way, backward-compatibly)
- `radarays_ros` build system + ROS 2 interfaces: working
- `radarays_ros` CPU node logic (`Radar`/`RadarCPU`/`radar_simulator`,
  ~30 live-tunable ROS 2 params): working, runtime-verified against a real
  mesh
- unifying the two independent physics copies: done — `radarays_ros` now
  exports a ROS-free `radarays_core` target (fresnel, cone sampling,
  denoising, Perlin noise); `radarays_gazebo_plugins` depends on it
  directly and its own from-scratch copy is deleted. Verified via all
  three fixtures passing unchanged plus a full clean CI rebuild (see
  `src/radarays_ros/MIGRATION.md`, "Two independent physics copies")
- `ray_reflection_test` port: done — reflection-path debug visualizer,
  faithful port to ROS 2 params + `visualization_msgs/msg/Marker`,
  runtime-verified against the same mesh as `radar_simulator` (see
  `src/radarays_ros/MIGRATION.md`)
- GPU/OptiX Harmonic port (`rmagine_gazebo_plugins`): static + dynamic
  baseline done — `rmagine::optix` builds and is GPU-verified (OptiX SDK
  7.5.0; 9.1.0 hit a real ABI mismatch, see #18),
  `rmagine_optix_map_system`/`rmagine_optix_sensor_system` runtime-verified
  end-to-end (`gz_optix_baseline.sdf` static, `gz_optix_dynamic.sdf`
  dynamic — `/scan`+`/points` over ROS 2, hit distance/motion both match
  expectations)
- GPU radar path (`radarays_ros` + `radarays_gazebo_plugins`): done and
  GPU-verified end to end (#20-21) — `radar_simulator_gpu` (standalone
  node) and `radarays_optix_sensor_system` (Gazebo System) both produce
  real, non-degenerate `/radar/image` output on the actual GPU, materials
  and all. CPU and GPU are now at parity across the whole workspace. Four
  real bugs found and fixed along the way, all root-caused with standalone
  repros outside gz-sim: a `SphereSimulatorOptix` segfault, a
  primitive-shape scale-clobber bug (fixed on both OptiX and CPU/Embree,
  see #19), a materials-fallback sizing bug, and a previously-undiscovered
  `rmagine_optix_map_system.cpp` MESH-case bug (direct-add instead of
  instance-wrapped, only surfaced once a real mesh — not just boxes — was
  finally exercised through it). Remaining: real recorded-dataset
  validation
- MulRan dataset validation: tooling done and verified (#23) —
  `lvr2`/`rmcl` build clean on Jazzy (cloned into `src/`), a new
  `mulran_radar_to_bag` script replaces the ROS 1/Qt `file_player_mulran`
  for the radar+ground-truth piece specifically. Verified against
  synthetic data matching MulRan's real layout. Only the actual dataset
  (registration-gated) remains
- CI automation: the build+test script is written and verified end-to-end
  locally (pass path and fail path both confirmed); the fixture harnesses
  are now also wired into real `colcon test`/CTest (#22), not just that
  script; a GitHub Actions workflow calling the script exists in all four
  repos but has never actually run on a real runner — that first real run
  is still open
- full `radarays` runtime parity: **done** — CPU and GPU produce real,
  runtime-verified `/radar/image` output through the same package surface
  everywhere in the workspace. Only real-dataset validation remains, and
  that's blocked on external data access, not engineering work.

So the project is in a good place:

- not finished, but everything that was actual engineering work is done
- the only two things left (getting real MulRan data, triggering CI on a
  real runner) are blocked on external access this session doesn't have
  (dataset registration, and push access respectively) — not on more code
- `radarays_ros` has real, working, runtime-verified CPU *and GPU* radar
  nodes now, and so does `radarays_gazebo_plugins`'s Gazebo integration,
  not just a build either way

## Status update: everything above is a snapshot, not current

Since the above was written: `GetRadarParams`/`GenRadarImage` are now
actually served (not just built); `rmagine_gazebo_plugins`'s Harmonic
sensor systems gained Pinhole/O1Dn/OnDn model support alongside Spherical
(both CPU/Embree and GPU/OptiX, all covered by new `colcon test`
fixtures); and MulRan validation was actually **run** against real data
(the user's own ParkingLot sample + calibration files), not just
tooling-readiness-checked — see `src/radarays_ros/MIGRATION.md`, "Real
MulRan validation — first run against the ParkingLot sample", for the
real bugs that surfaced only under real data (a UTM-precision issue, a
directory-layout assumption, an executor-reentrancy crash in the
`sync_topic` path, and a materials-configuration pitfall) and the actual
simulated-vs-real comparison numbers.

## Cross-Package Classic Parity Gap List + Tackle Order

A systematic audit (comparing each package's current ROS2/Harmonic code
against its own frozen ROS1/Gazebo-Classic branch or bundled legacy code)
found 15 concrete gaps across 3 of the 4 packages (`rmagine` itself has
no ROS coupling and nothing relevant on its `develop` branch — not
included below). Full detail with file:line evidence lives in each
package's own MIGRATION.md/README.md; this is the cross-package summary
plus a recommended order to tackle them in, given the user's stated goal
(equip a ROS2 + Gazebo Harmonic USV simulator with realistic LiDAR and
radar).

**Also flagged, not a parity gap but operationally important:** nothing
in any of these 4 package repos has been committed to git yet — the user
has no ownership of the upstream repos and no channel to their
maintainers right now, so the deliberate plan is to keep building and
testing entirely locally, and only figure out branches/pushing/upstream
review later once testing is further along. This is a conscious choice,
not an oversight — noted here so a future session doesn't mistake the
lack of commits for lost work or an accident.

### Tier 1 — likely blocking for a moving-vehicle sensor (the actual goal)

1. **Moving-sensor (egomotion) scenario, `radarays_gazebo_plugins`.**
   Every radar scenario tested so far has a stationary sensor; Classic
   had a full drivable-robot world that's never been ported. A radar/LiDAR
   mounted on a moving USV is the entire point of the end goal — this is
   the single most load-bearing gap.
2. **Plane geometry unsupported, `rmagine_gazebo_plugins`.** Water
   surfaces, docks, and ground planes are commonly modeled as SDF planes
   — if the USV world uses any, they'd be silently invisible to every
   sensor built on this package, with no warning.
3. **Visual `<scale>` hardcoded to identity, `rmagine_gazebo_plugins`.**
   A real, silent wrong-answer bug (not just a missing feature) for any
   scaled mesh — cheap to fix, self-flagged in the code's own comment
   already.

### Tier 2 — realism/completeness, matters once Tier 1 scenarios exist

4. **PointCloud2 missing `ring`/normals/`obj_id`/`face_id`,
   `rmagine_gazebo_plugins`.** Matters if anything downstream (SLAM,
   per-beam grouping/deskewing) expects these fields — common for real
   LiDAR-consuming stacks.
5. **OptiX noise models missing, `rmagine_gazebo_plugins`.** Real sensors
   are noisy; a perfect-precision GPU sensor is a worse stand-in for "the
   real thing" than a CPU one with the (currently CPU-only, need to
   double check) noise model applied.
6. **Heightmap geometry unsupported, `rmagine_gazebo_plugins`.** Only
   relevant if the USV world uses heightmap terrain (coastline/seabed) —
   otherwise skip.

### Tier 3 — perf/scale, matters once the real world is large

7. Full-scene rebuild instead of incremental diffing
   (`rmagine_gazebo_plugins`) — real perf regression for big scenes with
   occasional changes (e.g. other moving vessels).
8. Per-link self-tagging ignore mechanism lost (`rmagine_gazebo_plugins`).
9. No Gazebo-native mesh loader fallback (`rmagine_gazebo_plugins`).
10. Multi-topic/multi-message-type fan-out lost
    (`rmagine_gazebo_plugins`) — workaroundable today via a `ros_gz`
    bridge/relay node in the meantime.

### Tier 4 — usability/cleanup in `radarays_ros`, low risk, not blocking

11. Launch files never ported to ROS2 (still raw ROS1 XML) — needed
    eventually for a clean integration experience, but `ros2 run` +
    manual params already works today.
12. Example params YAML still ROS1 pickle-format — follows from #11.
13. CPU/GPU runtime switch removed (`radar_simulator` vs.
    `radar_simulator_gpu` as separate executables now) — low functional
    risk, both capabilities exist, just worth a guard/warning so an old
    `gpu:=true` param doesn't silently no-op.
14. `radar_simulator_gpu` missing the `serve_action` mode
    `radar_simulator` (CPU) has.
15. Dead `.cfg` dynamic_reconfigure files — trivial cleanup, no functional
    impact (all fields already carried over, verified field-by-field).

Tier assignments for items tied to "does the USV world actually use X"
(planes, heightmaps) are informed guesses, not confirmed against the
user's real world — deliberately, since the agreed scope for this pass is
testing everything without needing information from the actual USV
simulator. Revisit the ordering once the real world's geometry is known.

## Phase 1 Regression Pass — 2026-07-29

All 15 tier items above were completed and verified 2026-07-28 (previous
session). This is the next day's full regression pass: clean rebuild +
full test suite + re-verification of the riskiest runtime paths, before
starting on CI groundwork and MulRan material calibration. Nothing here
was committed/pushed — still no repo ownership, same as always.

**Clean build**: `rm -rf build/install` for all 4 packages +
`colcon build --packages-up-to radarays_ros radarays_gazebo_plugins`.
Exit 0, only pre-existing harmless CMake rpath/tbb warnings.

**Real regression found and fixed**: the clean build silently picked the
wrong OptiX SDK. This machine has two installed (`~/optix-7.5`, the one
that actually works with the installed driver, and `~/optix`, a newer
default that doesn't — `OPTIX_ERROR_UNSUPPORTED_ABI_VERSION` at runtime).
The correct one was previously pinned via a CMake cache variable that a
clean wipe doesn't preserve, since nothing in any `CMakeLists.txt`
hardcodes it. `rmagine`'s own `ctest` suite caught this immediately (5/30
OptiX tests failing) — without running it, this would have shipped
silently broken (compiles fine, only fails when actually run). Fixed by
rebuilding with `-DOptiX_INCLUDE_DIR=/home/saspragkathos/optix-7.5/include`
and rebuilding the 3 downstream packages against the corrected `rmagine`
install. This needs to be a documented, permanent build step (see Phase 2
below) so it doesn't silently regress again on the next clean build or in
CI.

**Test coverage gap found and closed**: `colcon test` (46 tests before
today) only covered pre-existing fixtures — it had zero coverage of the
plane/heightmap/ignore-link geometry features added in the previous
session's tier work; those were only ever verified with ad-hoc scratchpad
SDF worlds that don't survive between sessions (confirmed: gone by today).
Closed by adding 3 permanent `colcon test` fixtures to
`rmagine_gazebo_plugins` (`embree_fixture_plane`/`_heightmap`/
`_ignore_link`, see its README for the individual writeups) following the
exact pattern already established for Pinhole/O1Dn/OnDn. `ignore_link`'s
fixture is deliberately falsifiable (asserts the specific expected range
that would fail if the feature regressed), not just crash-freedom.
**Still open**: mesh-by-URI caching (Tier 3 #7) and all OptiX-side
features (noise models, GPU multi-topic, GPU egomotion) still have zero
automated regression coverage — `rmagine_gazebo_plugins`'s fixture suite
is Embree/CPU-only, same for `radarays_gazebo_plugins`'s. Scoped out of
today's pass for time; flagged here so it isn't forgotten.

**Full test suite, final state**: 52 tests, 0 errors, 0 failures, 0
skipped, across `rmagine` (30), `rmagine_gazebo_plugins` (8, was 5),
`radarays_gazebo_plugins` (3). `radarays_ros` has zero registered
tests — not a failure, just nothing there (a real gap, lower priority
than the above since its correctness has been proven repeatedly via the
MulRan runbook instead).

**Manual re-verification, post-fix**: both permanent egomotion worlds
(`gz_egomotion_radar_cpu.sdf`/`_gpu.sdf`) re-run clean — sensor oscillates
correctly, z stable at 0.500 (no gravity drift regression),
`finite_ranges` mostly 400/400 on both CPU and GPU. `mulran_sim.launch.py`
re-run end to end against a freshly regenerated MulRan bag at rate 5.0 (the
exact scenario that segfaulted before yesterday's `RadarCPU` race fix) —
survived the full 30s test window, all 3 processes (`rosbag2_player`,
`mulran_gt_to_tf`, `radar_simulator`) alive and `/radar/image` still
publishing at the end. Both yesterday's fixes and today's rebuild hold
together.

## Phase 2: CI on a real runner — 2026-07-29

CI workflows already existed for all four packages from earlier work
(`.github/workflows/build-and-test.yml` in `rmagine_gazebo_plugins`/
`radarays_gazebo_plugins`/`radarays_ros`, plus `rmagine`'s own
`radarays_workspace_integration.yml`), all sharing one script,
`radarays_gazebo_plugins/scripts/ci_build_and_test.sh`. None had been run
against a live GitHub-hosted runner yet (no push rights), but the script
itself is runnable locally, which is how it and everything below was
actually verified.

**Real gap found and fixed**: the existing script never called `colcon
test` at all — it hand-reimplemented just the 3
`radarays_gazebo_plugins` CPU fixtures as a bash loop, silently skipping
`rmagine`'s own 30-test CTest suite and all 8 `rmagine_gazebo_plugins`
fixtures entirely. It would **not** have caught the OptiX SDK regression
from Phase 1 above even on a machine where that mattered. Fixed by
replacing the hand-rolled loop with a real `colcon test` +
`colcon test-result --all --verbose` call. Re-verified locally: 52/52
tests pass, same numbers as the manual `colcon test` run in Phase 1.

**GPU/OptiX had no CI coverage at all** — every existing workflow runs on
`ubuntu-24.04` (GitHub-hosted, no GPU/CUDA), so `rmagine`'s own OptiX
build is auto-skipped there and none of the OptiX-dependent code
(`rmagine_optix_map_system`, `rmagine_optix_sensor_system`,
`radarays_optix_sensor_system`, `radar_simulator_gpu`) gets built or
tested in CI at all. Added:

- `radarays_gazebo_plugins/scripts/ci_build_and_test_gpu.sh` — same shape
  as the CPU script, but requires an `OPTIX_INCLUDE_DIR` environment
  variable (fails loudly if unset, no hardcoded default — see the script's
  own header for why: a *wrong but present* default is exactly the failure
  mode that silently broke this migration once already, in Phase 1 above).
  **Runtime-verified** on this dev machine (`OPTIX_INCLUDE_DIR=~/optix-7.5/
  include`): builds clean, all 52 tests pass, including the 5 OptiX ones.
- `build-and-test-gpu.yml` added to all 4 repos (`rmagine`'s own is
  `radarays_workspace_integration_gpu.yml`, matching its existing CPU
  naming), `runs-on: [self-hosted, gpu]`, reading `OPTIX_INCLUDE_DIR` from
  a repo-level Actions variable rather than a workflow-file constant, since
  the real value is specific to whatever machine ends up as the runner.

**To actually turn GPU CI on** (not done — needs repo ownership/admin
access this session doesn't have): register a self-hosted runner labeled
`gpu` against each repo (Settings → Actions → Runners) on a machine with a
real NVIDIA GPU, a driver, and an OptiX SDK whose ABI version that driver
actually supports (see Phase 1's dual-SDK finding — verify with
`rmagine`'s own OptiX CTest suite before trusting it, don't just assume
the SDK that happens to be installed is the right one). Then set the
`OPTIX_INCLUDE_DIR` repository variable (Settings → Secrets and variables
→ Actions → Variables) to that SDK's `include/` dir.

**Known gap, not addressed**: even once GPU CI is live, `colcon test`
still only proves `rmagine`'s own OptiX unit tests pass — there is no
automated fixture yet for OptiX-side `rmagine_gazebo_plugins`/
`radarays_gazebo_plugins` features (noise models, GPU multi-topic, GPU
egomotion) or for `radar_simulator_gpu` itself, matching the same gap
already flagged in Phase 1's fixture work. Manual verification (as done
throughout this whole migration) is still required for those.

## Phase 3: Automatic material optimizer — 2026-07-29

Ported `dev/opti`'s `radaray_opti.py` (an automatic radar-material
property optimizer) to ROS 2, and ran it against a real MulRan-reconstructed
mesh, not a synthetic scene.

**Checked `dev/opti` first, found it's genuinely unfinished, not just
ROS 1**: `to_param_vec()`/`vec_to_params()` hardcode `materials.data[1]`
("wall") and `materials.data[3]` ("glass") as an 8-parameter problem, tied
to a richly-labeled indoor Gazebo scene (`config/mulran_kaist02.yaml`'s
`object_materials` names actual objects like `DoorHallway1Glass-mesh`).
Its own `grid_search(bounds, N=5)` over 8 dimensions is 5⁸ ≈ 390,000
simulate-and-compare evaluations — never meant to finish, and its `main()`
doesn't even call it productively (a stray undefined `res` reference in
the dead code path proves this was never run to completion in this exact
shape). Also found: `config/mulran_kaist02.yaml`'s own `materials:` list
only has 2 entries, but `object_materials` references indices up to 4 —
inconsistent in both branches, not something this session introduced.

**Real adaptation, not a mechanical port**: a point-cloud reconstruction
from `lvr2_reconstruct` (see below) is one undifferentiated triangle
soup — no per-object labeling exists to split into "wall" vs "glass".
Ported the optimizer to tune exactly one material's 4 properties
(velocity/ambient/diffuse/specular, selected by `--material-index`), and
replaced the infeasible grid search with `scipy.optimize.
differential_evolution` — a real global optimizer suited to this exact
problem shape (black-box, noisy, no gradient, bounded, low dimensionality).
Also replaced `sklearn.metrics.mutual_info_score` (sklearn isn't installed
in this environment) with a small direct 2D-joint-histogram normalized
mutual information implementation — same estimator sklearn uses
internally, no new system dependency.

**Real mesh reconstruction**: ran the full runbook against the free
MulRan ParkingLot sample — `mulran_lidar_to_cloud` (stride 5, 236 of 1176
scans, 15.4M points) → `lvr2_reconstruct` (voxelsize 0.5) → a real
962,646-vertex / 1,660,979-face triangle mesh, ~340m × 315m bounding box.

**A second real, pre-existing bug found while wiring this up**:
`radar_simulator`/`radar_simulator_gpu` advertise `get_radar_params`/
`gen_radar_image` as plain, un-namespaced names (`/get_radar_params`,
confirmed via `ros2 service list`) — NOT prefixed with the server node's
own name the way ROS 1's private-namespace convention would imply. The
first port attempt assumed `{server_node_name}/get_radar_params` and
failed immediately; fixed by using the plain names directly. Worth
flagging for anyone else porting a ROS 1 action/service client script in
this workspace — the same assumption could bite again.

**Materials config**: added `config/mulran_optimizer_materials.yaml` —
2 entries (air, and the one real material to tune) plus
`object_materials: [1]`. This `object_materials` entry is load-bearing,
not decoration: `RadarCPU.cpp` falls back to material index 0 for any
object id past the end of `object_materials`
(`obj_id < m_object_materials.size() ? m_object_materials[obj_id] : 0`),
and index 0 is the air-like entry — an empty `object_materials` (as used
for the quick smoke-test materials.yaml elsewhere in this migration)
would make every hit silently resolve to air regardless of what real
material is tuned at index 1.

**Runtime-verified with a real convergence run** against the reconstructed
mesh, driven by a looping real MulRan bag (`radar_simulator --serve_action`,
`mulran_gt_to_tf` for live TF, `n_cells`/`n_samples` matched to the real
sensor's resolution via `cfg/mulran_kaist_dyncfg.yaml` so real/simulated
image shapes align without needing the resize fallback). First pass (same
day) hit a 180s wall-clock cap partway through at 908/2460 evaluations —
re-ran to full completion once isolation was set up (see below), all
**2460 evaluations** (40 generations × population 60), and it shows clean,
textbook differential-evolution convergence, not just noise:

```
generation  1- 5: f(x) = -0.00297
generation  6-15: f(x) = -0.00302
generation 16-23: f(x) = -0.00304
generation 24-40: f(x) = -0.00308   (held stable for the final 17/40 generations)
```

Baseline (initial, un-tuned params) score: 0.0025. Best found: **0.0031**
— a ~24% relative improvement, at `velocity=0.0157, ambient=0.487,
diffuse=0.221, specular=3132.7`. Consistent with the earlier partial run's
own trend (that one was heading toward low velocity / high specular too,
just hadn't gotten there yet) -- both runs independently converging on the
same region of parameter space is itself a good sign this isn't noise.

Absolute NMI scores are low in absolute terms (real MulRan radar images
vs. a generic-material simulation of a raw point-cloud reconstruction, no
prior calibration) -- expected for a first real run, not a red flag. What
matters here is that the optimizer's own infrastructure (real action-based
simulation loop, real image comparison, real global optimizer) is proven
correct and converges on real data to completion, which was the actual
goal of this phase. A materials config with more than one tunable material
for a richer scene is a natural next step for whoever picks this up, not
a blocker.

**Found and fixed a real cross-talk risk while re-running this**: `ros2
node list` unexpectedly showed a large set of unrelated nodes
(`guidance_node`, `thrust_control_node`, `obstacles_tracker_node`,
`ros_gz_bridge`, etc) -- the user's own separate USV-simulator framework,
running concurrently on this machine. `ROS_DOMAIN_ID` was unset (defaults
to 0) for both, so the two shared one ROS graph with no isolation at all
by default -- this test pipeline's looping bag/TF publications could have
cross-talked with a live, unrelated system. Fixed for this session only by
running the whole pipeline (`mulran_gt_to_tf`, `radar_simulator`, `ros2
bag play`, `radaray_opti`) under `ROS_DOMAIN_ID=77`, set as a plain shell
env var for these specific commands -- deliberately NOT hardcoded into any
launch file, script, or config, since the user needs the default domain
preserved for other work. Confirmed via `ros2 node list` under domain 77
vs domain 0 that the two no longer share a graph. Worth remembering for
any future test session on this machine: check `ros2 node list` before
assuming a clean graph, and set `ROS_DOMAIN_ID` per-shell rather than
system-wide.

Minor known cosmetic issue: intermittent `Ignoring unexpected result
response` / `unexpected goal response` warnings from the action client
during back-to-back goal sends. Did not affect correctness (scores
tracked real parameter changes throughout both the 908-eval and
2460-eval runs).

**Root-caused 2026-07-30**, by reading rclpy's own source rather than
guessing: `/opt/ros/jazzy/.../rclpy/action/client.py`, `execute()`
(around line 294-334). Action goal-send/result-request calls are
service calls under the hood, tracked client-side by a sequence number
in `_pending_goal_requests`/`_pending_result_requests`. The warning
fires whenever a response arrives whose sequence number isn't in that
dict anymore -- i.e. a **late-arriving duplicate response for a request
already resolved and cleaned up**, which is exactly the kind of thing
DDS reliable-QoS retransmission produces under real timing, especially
with thousands of goal/result round-trips in a tight optimizer loop.
rclpy's own code treats this as a normal, safely-ignorable case (a
`warning`, not an `error` -- the message is just informing you it
happened, not indicating anything went wrong). Nothing to fix in
`radaray_opti.py` -- this is expected upstream rclpy_action behavior
under sustained real throughput, not a bug in this codebase.

This was the last of the three tracks planned for today (full regression
pass, CI groundwork, automatic material optimizer). All three produced
real, verified artifacts; several genuine bugs were found and fixed along
the way (see each phase above) rather than just "done, no findings".

## Phase 4: Closing remaining test-coverage gaps — 2026-07-29, later still

After Phase 1-3, two coverage gaps were still explicitly flagged as open:
`radarays_ros` had zero registered tests at all, and OptiX/GPU code across
both Gazebo-plugin packages had none either. Closed both.

**`radarays_ros`**: added its first-ever fixture, `radarays_ros_fixture_wall`
(standalone `radar_simulator` + static TF against a self-contained
synthetic mesh, no Gazebo at all). See `radarays_ros/MIGRATION.md`, "
`radar_simulator` regression fixture", for the full writeup — this single
fixture surfaced 3 separate real findings while being built: a wrong
"boresight column" assumption (`radar_simulator` sweeps 360°, not a fixed
FOV), a startup-transient false failure (fixed by waiting past warm-up,
not loosening thresholds), and a recurrence of the `ros2 run`
wrapper-vs-child PID mistake from earlier in this migration (leaked test
processes cross-talked with another package's fixture over the shared
`/radar/image` topic name).

**OptiX/GPU**: `rmagine_gazebo_plugins` gained 4 fixtures
(`optix_fixture_baseline`/`_dynamic`/`_noise`/`_multi`) and
`radarays_gazebo_plugins` gained 1 (`radarays_fixture_egomotion_gpu`). Two
of these (`optix_baseline`/`optix_dynamic`, and `egomotion_gpu`) reused
world files that had existed on disk since earlier in this migration but
were simply never wired into anything — a quick, high-value fix once
noticed. The other three (`optix_noise`, `optix_multi`, and the
motion-falsifiability checks on `egomotion_gpu`) needed new world files
and, for `optix_multi`, a small harness extension to subscribe to more
than the two hardcoded topic names. See `rmagine_gazebo_plugins/README.md`
for the full writeup and real numbers.

**A real process-management lesson reinforced twice in one session**:
while verifying these, `ros2 node list` showed cross-talk between this
session's own leaked test processes (from earlier manual debugging,
killed by the `ros2 run` wrapper's PID instead of the actual child) and
a currently-running fixture test, briefly producing a false failure that
looked like a real regression. Root-caused via `ps aux` rather than just
loosening a threshold to make it pass. See `[[project_ros_domain_sharing]]`
/ `feedback_process_cleanup.md`-equivalent memory entries for the standing
note — always run `ps aux | grep <exe>` after killing a `ros2 run`
launch, not just check the wrapper's exit.

**Final test count across all four packages**: 64 tests, 0 errors, 0
failures — `rmagine` 30 (unchanged), `rmagine_gazebo_plugins` 12 (was 5 at
the very start of this migration, then 8 after Phase 1, now 12),
`radarays_gazebo_plugins` 4 (was 3), `radarays_ros` 1 (was 0). Confirmed
stable across 3 separate full-suite runs, no process leaks.

**Still open at that point**: mesh-by-URI caching (Tier3 #7) had no
regression fixture — didn't fit the capture/compare-JSON pattern cleanly
(it's about an internal rebuild *count*, not published topic values).

## Phase 5: Mesh-by-URI caching fixture — and a real bug it found

Closed the last item above with a standalone script,
`check_mesh_cache_fixture.py` (`embree_fixture_mesh_cache` in `colcon
test`), against a new world (`gz_embree_mesh_cache.sdf`): three static
entities sharing one self-contained mesh file, plus an unrelated moving
box forcing many rebuild passes. Added a `(cache miss)` debug log line to
`rmagine_embree_map_system.cpp` (there was no way to observe cache hits/
misses at all before) and asserted the miss count stays at exactly 1,
shared across all 3 entities and every subsequent rebuild pass.

**First run found a real bug, not just a gap**: the count came back 3,
growing to 383 within seconds. Root cause: the cache-hit condition in
`BuildStaticMap()` (both `rmagine_embree_map_system.cpp` and
`rmagine_optix_map_system.cpp`) required a per-pass "first use" guard in
addition to the key being cached — meant to prevent a same-pass
population race between two entities loading a brand-new key
simultaneously. But `BuildStaticMap()` runs single-threaded (confirmed by
inspection), so that race never existed — the guard's only real effect
was that whenever 2+ entities share an already-cached mesh in the same
rebuild pass, only the *first* gets the legitimate hit; every other one
silently reloads the file from disk (re-running Assimp) every single
pass, forever. This defeated mesh-by-URI caching for exactly the case it
exists for (multiple entities sharing one mesh file), on every clean
rebuild, invisibly, since nothing was measuring it until this fixture
existed. Fixed on both backends: hits now depend only on the key being in
`mesh_cache_`; the now-dead `within_pass_mesh_uris_` guard was removed
from both `.cpp`/`.hpp` pairs. Re-verified: count is 1, stays at 1 across
~26s and dozens of forced rebuild passes.

**Final test count, all four packages: 66 tests, 0 failures**
(`rmagine` 30, `rmagine_gazebo_plugins` 13, `radarays_gazebo_plugins` 4,
`radarays_ros` 1). Every Classic-parity gap from the original systematic
audit now has automated regression coverage. Remaining genuinely-blocked
items: GPU CI going live (needs repo admin access -- update: turned out
not to be blocked at all, see below), richer multi-material optimizer
tuning (needs a real labeled scene, not available), and the Phase 3
cosmetic action-client warning (root-caused later, see below -- benign
upstream rclpy behavior, not a bug here).

## Phase 2 (continued): GPU CI actually went live — 2026-07-30

Update to the "genuinely-blocked" item above: it wasn't actually blocked
on repo *admin* access — that assumption was wrong. GPU CI only needs
admin on the *upstream* `uos/*` repos, which we don't have. But each of
the 4 repos was already forked to `sotomotocross/*` (pre-existing, from
before this session), and fork admin is fully ours. So GPU CI could go
live today, on our own forks, with zero owner involvement.

Did exactly that: committed and pushed real work to all 4 forks
(branch `ros2-jazzy-harmonic`), registered a self-hosted GPU runner
against each fork on this dev machine, and got **all 4 repos' GPU CI
green — 66/66 tests each, confirmed via real test-count log output, not
just a green checkmark**. Real bugs found and fixed to get there (not
glossed over):

1. **`rosdep`/`setup.bash` unbound-variable crash** — `source
   /opt/ros/jazzy/setup.bash` under `set -euo pipefail` hits an unset
   `AMENT_TRACE_SETUP_FILES`. `ci_build_and_test.sh` already had the
   `set +u`/`set -u` workaround; the workflow YAML's own inline rosdep
   step never got it. Fixed in all 8 workflow files (CPU + GPU × 4 repos).
2. **Sibling-package clones pointed at unported upstream** — each
   workflow's "checkout sibling packages" step cloned `uos/*` main/ros2,
   which has none of this migration's work. Temporarily redirected to
   `sotomotocross/*` at `ros2-jazzy-harmonic` instead, clearly marked
   TEMPORARY in each file with a comment explaining it must revert to
   `uos/*` once the eventual PRs are open/merged.
3. **Self-hosted runner workspace isn't ephemeral** — GitHub-hosted
   runners get a fresh VM every run; a self-hosted runner's `_work`
   directory persists across runs, so a plain `git clone` into an
   already-populated sibling directory fails on the *second* run. Added
   `rm -rf` before each clone.
4. **A chain of missing sudo permissions**, discovered one at a time by
   actually running the job on real hardware: writing `/etc/timezone`,
   setting env vars on `apt-get`, `locale-gen`, a second `ln -sf`
   variant, `rosdep init`. All resolved via one narrowly-scoped sudoers
   file (see below) — not a blanket `NOPASSWD: ALL`, deliberately, per
   direct instruction to keep the blast radius small.
5. **Two real fixture-timing flakes on actual hardware** — `radarays_ros`'s
   `wall` fixture and `radarays_gazebo_plugins`'s `egomotion_gpu` fixture
   both needed wider capture windows than local development had shown was
   necessary; this specific machine, under real CI load, runs the
   simulation somewhat slower (~4 Hz vs. ~10 Hz seen locally for
   `egomotion_gpu`). Widened `duration_sec`/`min_messages`/`min_rate_hz`
   with real margin instead of just hoping a retry would pass.

### What's now on this machine (durable, standing state)

- **`/etc/sudoers.d/gh-actions-runner-radarays`** — narrowly-scoped
  passwordless sudo for exactly: `apt-get` (incl. `SETENV` for
  `DEBIAN_FRONTEND`/`RTI_NC_LICENSE_ACCEPTED`), `dpkg -i
  /tmp/ros2-apt-source.deb`, `ln` (any args), `ldconfig`, `timedatectl
  set-timezone UTC`, `bash -c "echo 'Etc/UTC' > /etc/timezone"`,
  `bash -c "rm /etc/ros/rosdep/sources.list.d/20-default.list || true"`,
  `locale-gen`, `rosdep`. Deliberately not `NOPASSWD: ALL`.
- **4 self-hosted runner installs**, one per repo, each in its own
  directory so they can all run concurrently without clashing:
  - `~/actions-runner-radarays-gazebo-plugins` (registered against
    `sotomotocross/radarays_gazebo_plugins`)
  - `~/actions-runner-rmagine` (`sotomotocross/rmagine`)
  - `~/actions-runner-rmagine-gazebo-plugins` (`sotomotocross/rmagine_gazebo_plugins`)
  - `~/actions-runner-radarays-ros` (`sotomotocross/radarays_ros`)
  
  Each is registered with the `gpu` label and an `OPTIX_INCLUDE_DIR`
  repo variable pointing at `~/optix-7.5/include` (see
  [[project_optix_sdk_pin]] for why that exact SDK, not the newer
  default one also on this machine).
- **`~/restart-ci-runners.sh`** — brings back any of the 4 runners that
  died (reboot, crash, manual stop). Safe to run any time; skips
  already-running ones. Usage: just run it, no arguments.

### Not installed as system services (deliberately, for now)

These run as plain background processes (`nohup ./run.sh &`), not
systemd services — installing as a service needs one more interactive
`sudo` command per runner (`sudo ./svc.sh install && sudo ./svc.sh
start`, run from each runner's own directory), which wasn't done since
this was still an active work session, not a "set and forget forever"
decision point. Consequence: none of the 4 runners survive a reboot on
their own -- run `~/restart-ci-runners.sh` after any reboot/login to
bring them back. If persistence-across-reboot is wanted later, the
systemd install is a one-time step per runner directory.

### Real, standing limitation: tied to this one laptop

Right now, "GPU CI" for all 4 repos means *this specific personal dev
machine*, and only while its runner processes happen to be running. That
is a genuine architectural weakness, flagged explicitly rather than
glossed over:

- If this laptop is off, asleep, or its runners aren't started, GPU CI
  for all 4 repos silently has nothing to pick up queued jobs -- they'll
  sit queued indefinitely, not fail loudly.
- It's a single point of failure with no redundancy, tied to one
  person's personal hardware, not real shared team/project
  infrastructure.
- This was always meant as a way to *prove the GPU CI pipeline works at
  all* before involving the repo owners (Phase 2's original goal) --
  not a proposal for where GPU CI should permanently live.

**Real alternatives for later, not evaluated/actioned this session**
(worth a deliberate decision, not a default):
1. Ask the upstream owners, once a PR is open, whether they already have
   (or are willing to set up) their own GPU-capable self-hosted runner
   for `uos/*` -- the natural long-term home if this ever merges.
2. A cloud GPU instance (AWS/GCP/Azure GPU VM, or a GPU-enabled CI
   provider) registered as the self-hosted runner instead of a personal
   laptop -- costs money, but decouples from any one person's machine
   being on.
3. Accept CPU-only CI as the actual gate for merging (already fully
   solid, GitHub-hosted, zero dependency on any personal machine) and
   treat GPU verification as something run manually/periodically rather
   than on every push -- lower rigor, but zero infra to maintain.
No decision made on this yet -- explicitly deferred ("let's not be
dependent on the laptop that's running").

## Phase 6: Multi-material optimizer support -- built, wired, and honestly evaluated -- 2026-07-30

The 3rd "known documented gap" was `radaray_opti.py` only ever tuning one
material at a time, with the stated reason being "no available scene has
more than one distinctly-labeled material to exercise more than that."
Rather than leave that gap as permanently blocked, built a synthetic scene
specifically to exercise it, and generalized the optimizer to actually
support it -- then ran it for real and am reporting what it actually did,
not just what it was designed to do.

**What object ID actually means for a multi-mesh map file** -- had to
answer this first: `rmagine::import_embree_map()`
(`rmagine/src/rmagine_embree/include/rmagine/map/EmbreeMap.hpp`, line 94)
loads the whole file into one Assimp `aiScene`, then
`make_embree_scene()` (`EmbreeScene.cpp`) turns *each* `aiMesh` entry into
its own `EmbreeMesh`, added to the top-level scene as its own instance --
each getting its own Embree geometry ID at ray-hit time (`obj_id`), in the
order the scene graph's mesh-bearing nodes are traversed. Confirmed this
empirically, not just by reading the code: wrote a scratch verification
executable (`rmagine/tests/embree/verify_two_walls_scratch.cpp`, added
temporarily, built, run, then fully reverted -- not committed) that cast
single rays at a 2-object COLLADA file and printed `obj_id` per hit.
Result: first document-order node -> `obj_id 0`, second -> `obj_id 1`,
exactly as hypothesized. This confirms `object_materials[obj_id]`
(`RadarCPU.cpp`/`radar_algorithms.cu`) is genuinely per-sub-mesh, not
per-file -- a multi-object scene file is all it takes, no Gazebo required.

**The synthetic scene**: `testdata/two_walls_test.dae` -- two separate
COLLADA `<node>`/`<geometry>` box pairs (same 8-vert/12-tri topology as
the existing `wall_test.ply` single-wall fixture), one at x=[5.0, 5.2]
("WallNear"), one at x=[9.0, 9.2] ("WallFar"). Paired with
`two_walls_test_materials.yaml`: 3 materials (air, "near", "far"),
`object_materials: [1, 2]`.

**Optimizer generalization** (`scripts/radaray_opti.py`): `to_param_vec`/
`vec_to_params` now take a list of material indices instead of one int,
concatenating/slicing 4 params per index; `--material-index` is now
`nargs="+"` (still defaults to `[1]`, so existing single-material usage is
unchanged) -- `--material-index 1 2` optimizes 8 parameters
(velocity/ambient/diffuse/specular x2) in one `differential_evolution`
run. `PARAM_BOUNDS`/`PARAM_NAMES` generalized the same way (bounds
repeated per material, names prefixed `mat{idx}_...`).

**Real verification run** (not just "it doesn't crash"): started
`radar_simulator` against the 2-wall scene with `serve_action:=true`,
rendered a "ground truth" image using known, hidden material values
(mat1: velocity 0.22/ambient 0.8/diffuse 0.3/specular 3000; mat2: velocity
0.02/ambient 0.1/diffuse 0.9/specular 200 -- deliberately different from
the yaml's own defaults), published that image on `/Navtech/Polar` like a
real bag would, then ran `radaray_opti --material-index 1 2` against it
for real and compared the recovered values to the hidden ground truth.

Two runs, increasing budget:

| run | maxiter/popsize | evals | baseline score | best score | notes |
|---|---|---|---|---|---|
| 1 | 20 / 10 | ~240 | 0.4707 | 0.4729 | DE converged (internal tol) after ~2 generations |
| 2 | 80 / 20 | ~800 | 0.4823 | 0.4864 | DE converged after only 4 generations again |

**Honest result**: the mechanism is genuinely correct and working --
multi-material parameters are separated and applied correctly (confirmed
directly: `mat1_diffuse` landed at 0.30993 against a true 0.3, almost
exact), the search consistently moves in the beneficial direction (best
score > baseline in both runs, never worse), and nothing crashes across an
800-eval, 8-dimensional run. What did **not** happen is full recovery of
all 8 ground-truth values -- several (`mat1_ambient`, `mat2_ambient`,
`mat2_diffuse`, `mat2_specular`) are far from their true values in both
runs, and a 3.3x increase in eval budget (240 -> 800) barely moved the
best score (0.4729 -> 0.4864), while `differential_evolution`'s own
internal convergence criterion kept firing after only a handful of
generations well before genuinely exhausting the search.

This isn't attributed to a bug in the new code -- it's the same class of
behavior noted in Phase 3's original single-material work, which needed
real recorded MulRan images (much richer structure than two flat wall
returns) and up to ~2460 evaluations to tune even 4 parameters well. A
synthetic scene with only two simple flat-panel returns likely doesn't
give whole-image normalized mutual information enough structure to
distinguish all 8 parameters' effects from each other -- several
combinations of ambient/diffuse/specular probably render visually similar
sparse energy peaks, which is an identifiability limit of the metric/scene
combination, not of the optimizer's plumbing.

**Where this actually leaves the gap**: closed in the sense that mattered
-- "the optimizer's architecture cannot tune more than one material" was
false, and is now demonstrated false with working code, not just an
argument. Not closed in the sense of "proven to reliably recover exact
material values" -- that would need either a richer synthetic scene (more
distinct geometric features per material, not just two flat panels) or
real recorded data, exactly the same real-data dependency the original gap
was about in the first place. Documented here rather than silently
upgraded to "done."

Cleanup: the scratch verification executable and its temporary
`tests/embree/CMakeLists.txt` entry were fully removed after use (`git
status` confirmed clean in `rmagine` afterward) -- only
`two_walls_test.dae`/`two_walls_test_materials.yaml` and the
`radaray_opti.py` generalization are new, real, permanent additions.

Also written this session, alongside this gap work: `ARCHITECTURE.md`,
`QUICKSTART.md`, `INTEGRATION_GUIDE.md` (this repo's root, alongside this
file) -- a consolidated
explainer of how the 4 packages fit together, a hands-on runbook of
concrete commands to see each part working today, and a guide for
attaching these sensors to an existing Gazebo Harmonic sim (e.g. a vehicle
simulator built independently of this migration).

## Phase 7: `mesh_publisher.cpp` -- ported without its blocked dependency -- 2026-07-30

The 2nd "known documented gap" was `mesh_publisher.cpp` -- unbuilt because
its message type (`mesh_msgs`) and RViz consumer (`rviz_map_plugin`/
`mesh_tools`) are both ROS 1-only upstream, confirmed still true by
re-checking the actual upstream repo rather than trusting old notes (see
`MIGRATION.md`'s "What Is Not Migrated Yet").

Rather than leave it blocked on a dependency we don't own, re-targeted it:
same rmagine-side logic (walk the `EmbreeScene`'s mesh/instance graph,
compose transforms, extract vertices/faces), but publishing
`visualization_msgs/msg/MarkerArray` (one `TRIANGLE_LIST` `Marker` per
mesh) instead of `mesh_msgs::MeshGeometryStamped`. This is the same
message type `ray_reflection_test.cpp` already publishes successfully --
RViz2 renders it natively, no plugin needed, no ROS 1-only dependency
anywhere in the chain.

Ported the ROS side to `rclcpp` (`Node`/`declare_parameter`/
`create_publisher`/`Rate`), kept the same `map_file`/`pre_transform`
parameter shapes, added the executable to `CMakeLists.txt` next to
`ray_reflection_test` (same `target_link_libraries`/
`ament_target_dependencies` shape). Built clean, then runtime-verified for
real: ran it against `testdata/two_walls_test.dae` and confirmed via
`ros2 topic echo /mesh_markers` that the published triangle points
(`x: 5.0, y: -2.0, z: 0.0`, ...) are the mesh's actual, correct vertices --
not just "the process didn't crash."

One thing noticed while testing, not a regression: sending this node a
plain `kill -INT`/`SIGTERM` doesn't make it exit promptly (waited 8s, no
exit). Checked whether this was something introduced by the port --
`radar_simulator.cpp` and `ray_reflection_test.cpp` use the exact same
`while(rclcpp::ok()) { ...; rclcpp::spin_some(node); }` shutdown pattern
with no custom signal handling, so this is shared, pre-existing behavior
across every standalone node in this package, not something new. The
fixture harness (`radarays_ros_fixture_harness.py`'s `stop_process()`)
already accounts for exactly this -- SIGINT, then a 5s timeout, then
SIGKILL -- so nothing to fix here specifically for this port.

`package.xml` already declared `visualization_msgs`/`geometry_msgs` (used
by `ray_reflection_test`); no new dependency was needed. `mesh_msgs` was
never declared there to begin with, confirming it was never really wired
in even before this migration.
