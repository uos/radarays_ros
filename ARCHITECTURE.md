# Architecture: how the 4 packages fit together

This is the "what actually talks to what" map for this workspace, written
after the ROS 2 Jazzy + Gazebo Harmonic migration reached full CPU/GPU
parity (see `MIGRATION_HANDOFF.md`'s Practical Summary). Read this before
`QUICKSTART.md` if you've never touched this workspace, or before
`INTEGRATION_GUIDE.md` if you're trying to bolt this onto another
simulator.

## The 4 packages, bottom to top

```
┌─────────────────────────────────────────────────────────────────┐
│ radarays_ros            standalone ROS 2 nodes: radar_simulator,│
│                          radar_simulator_gpu, ray_reflection_test,│
│                          radaray_opti (material auto-tuner)      │
├─────────────────────────────────────────────────────────────────┤
│ radarays_gazebo_plugins  Gazebo Harmonic Systems: radar sensor   │
│                          (CPU/Embree + GPU/OptiX), per-visual    │
│                          <radarays_material> SDF tag, dynamic    │
│                          reconfigure (24 live ROS 2 params)      │
├─────────────────────────────────────────────────────────────────┤
│ rmagine_gazebo_plugins   Gazebo Harmonic Systems: keeps gz-sim's │
│                          scene graph (meshes, poses, motion)     │
│                          mirrored into an rmagine EmbreeMap/     │
│                          OptixMap in real time                   │
├─────────────────────────────────────────────────────────────────┤
│ rmagine                 ray-tracing library, 2 backends:         │
│                          Embree (CPU) / OptiX (GPU). No ROS,     │
│                          no Gazebo dependency at all.            │
└─────────────────────────────────────────────────────────────────┘
```

Each layer only depends on the one(s) below it. `rmagine` is a plain C++
library — it has never heard of ROS or Gazebo. Everything ROS/Gazebo-shaped
is added by the layers above it.

## `rmagine` — the ray-tracing engine

A standalone library for casting many rays against a triangle-mesh scene
and getting back hit distance, point, normal, and **object ID** per ray.
Two interchangeable backends:

- **Embree** (`rmagine::EmbreeMap`, CPU, Intel's ray-tracing kernels)
- **OptiX** (`rmagine::OptixMap`, GPU, NVIDIA's ray-tracing SDK)

The object-ID mechanic matters a lot for everything above it: when a scene
file has more than one distinct sub-mesh (e.g. two separate `<geometry>`
nodes in a COLLADA file, or two separate Gazebo `<link>`s), each one gets
its own integer ID at ray-hit time, assigned in the order the scene graph
was traversed when the map was built (`EmbreeScene.cpp`'s
`make_embree_scene()` / the equivalent OptiX path). Every material system
above this layer is really just "an array indexed by that object ID."

`rmagine` has its own `import_embree_map(file)` for loading a standalone
mesh file directly (used by `radarays_ros`'s standalone nodes, no Gazebo
involved) — separate from `rmagine_gazebo_plugins`'s per-entity live scene
mirroring described next.

## `rmagine_gazebo_plugins` — Gazebo's scene, live, in rmagine's format

A Gazebo Harmonic `System` plugin that watches the sim's entity component
system (poses, meshes, primitive shapes being spawned/moved/removed) and
keeps a matching `rmagine::EmbreeMap`/`OptixMap` up to date, frame by frame.
This is what makes rmagine aware of a *dynamic* Gazebo world — moving boxes,
rotating objects, whatever the world file animates — not just a frozen
snapshot taken at load time.

It also exposes rmagine's ray-casting as ROS 2-facing sensors in their own
right (`/scan`, `/points` — plain LiDAR-shaped output), independent of the
radar-specific logic above it. This is the layer to look at if what you
want is a generic ray-traced LiDAR/depth sensor, not specifically a radar.

## `radarays_gazebo_plugins` — the radar sensor, as a Gazebo System

Built on top of `rmagine_gazebo_plugins`'s live scene. Adds:

- Radar-specific physics on top of raw ray hits: Fresnel reflection/
  refraction, cone sampling (a radar beam isn't a single ray), Perlin-noise
  clutter, multi-bounce/multi-path returns — all in a ROS-free
  `radarays_core` target shared with `radarays_ros` (so the physics is
  implemented exactly once, not twice).
- Per-visual materials: tag any Gazebo `<visual>` with a custom
  `<radarays_material>` SDF element and it gets its own reflectivity/
  velocity/etc, independent of every other object in the world.
- Live tuning: ~24 of those physics parameters are ROS 2 dynamic
  parameters — change them on a running sim with `ros2 param set` and the
  next `/radar/image` reflects it immediately, no restart.
- Both CPU (`radarays_embree_sensor_system`) and GPU
  (`radarays_optix_sensor_system`) variants, at parity.

Output is a real `sensor_msgs/Image` on `/radar/image` — a synthetic
rotating-radar polar image (azimuth × range bins), not a point cloud.

## `radarays_ros` — the same radar physics, without Gazebo

The same `radarays_core` physics, but driven by `rmagine`'s standalone
`import_embree_map()`/OptiX equivalent instead of a live Gazebo world. Two
main entry points:

- **`radar_simulator` / `radar_simulator_gpu`** — load one static mesh file
  + a materials YAML, publish `/radar/image` continuously (matching a
  ROS bag's timing via `sync_topic`), or serve on-demand images through a
  `GenRadarImage` action + `GetRadarParams` service (`serve_action:=true`).
  This is what a real MulRan-dataset validation run and the optimizer below
  both drive.
- **`radaray_opti`** — a `scipy.optimize.differential_evolution`-based
  auto-tuner. Drives `radar_simulator`'s action interface with candidate
  material parameters, scores each rendered image against a real recorded
  radar image via normalized mutual information, and searches for the
  material values that best match reality. Originally limited to tuning
  one material's 4 properties at a time; now supports tuning several at
  once (`--material-index 1 2 ...`), verified against a purpose-built
  2-object synthetic scene (`testdata/two_walls_test.dae` — see
  `MIGRATION_HANDOFF.md`'s multi-material optimizer verification writeup).

Also present: `ray_reflection_test` (a debug visualizer publishing
`visualization_msgs/Marker`s for individual reflection paths — useful when
a radar return looks wrong and you need to see exactly which surface it
bounced off), and dataset-import tooling (`mulran_radar_to_bag`,
`mulran_lidar_to_cloud`, `mulran_gt_to_tf`) for replaying real recorded
radar sequences (MulRan) through this same physics for validation.

## Two ways to use this radar model

1. **Inside a live Gazebo Harmonic world** — `radarays_gazebo_plugins`'s
   Systems, attached to an SDF world with your own robot/vehicle/objects.
   Real-time, reacts to a moving simulation. This is the path
   `INTEGRATION_GUIDE.md` covers for bolting the radar onto a bigger
   existing sim.
2. **Standalone, against one static mesh file** — `radarays_ros`'s
   `radar_simulator`. No Gazebo process at all, just a mesh + a materials
   YAML + ROS 2. This is the path used for dataset validation and for the
   optimizer, and the fastest way to experiment with radar physics/material
   parameters in isolation (see `QUICKSTART.md`).

Both paths share the exact same `radarays_core` physics — a parameter
tuned via `radaray_opti` against the standalone path is the same physics
that runs inside a full Gazebo world.
