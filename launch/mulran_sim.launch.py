"""ROS 2 port of the ROS 1 `mulran_sim.launch`.

Runs the same pipeline this session's MulRan runbook (MIGRATION.md,
"Runbook: reproducing or extending this") already verified manually across
several terminals -- as one launch command:

  ros2 bag play <bagfile>                     (paused, matches ROS 1 --pause)
  mulran_gt_to_tf --calib <calib>              (map -> base_link -> sensor TF)
  radar_simulator --sync_topic /Navtech/Polar  (materials_file + object_materials
                                                 + the ROS 2-format dyncfg params)

`bagfile`/`meshfile`/`calib` have no defaults -- they are per-sequence
artifacts produced by the runbook's own steps 2-3 (mulran_lidar_to_cloud +
lvr2_reconstruct, mulran_radar_to_bag), not something this generic launch
file can guess a path for.

RViz is deliberately NOT ported here (unlike the ROS 1 file's `gui` arg):
no `.rviz2` config exists in this repo, and the ROS 1 one relies on the
`rviz_map_plugin`/mesh_tools fork that isn't part of this migration's scope
either -- see README.md's Migration Status note on `mesh_tools`.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os
import yaml


def _launch_setup(context, *args, **kwargs):
    meshfile = LaunchConfiguration("meshfile").perform(context)
    bagfile = LaunchConfiguration("bagfile").perform(context)
    calib = LaunchConfiguration("calib").perform(context)
    rate = LaunchConfiguration("rate").perform(context)
    materials_file = LaunchConfiguration("materials_file").perform(context)
    radarparams = LaunchConfiguration("radarparams").perform(context)
    map_frame = LaunchConfiguration("map_frame").perform(context)
    base_frame = LaunchConfiguration("base_frame").perform(context)
    sensor_frame = LaunchConfiguration("sensor_frame").perform(context)
    sync_topic = LaunchConfiguration("sync_topic").perform(context)

    # object_materials/material_id_air are real declared ROS 2 parameters on
    # radar_simulator (unlike the "materials" list itself, which
    # loadRadarMaterialsFromFile() reads straight off materials_file's YAML,
    # bypassing the ROS 2 param system entirely) -- pull them from the same
    # materials_file so there is one source of truth, matching how the ROS 1
    # launch loaded the whole config/mulran_kaist02.yaml as rosparams.
    with open(materials_file, "r") as f:
        materials_yaml = yaml.safe_load(f) or {}
    object_materials = materials_yaml.get("object_materials", [])
    material_id_air = materials_yaml.get("material_id_air", 0)

    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", bagfile, "-r", rate, "--start-paused"],
        output="screen",
    )

    gt_to_tf = Node(
        package="radarays_ros",
        executable="mulran_gt_to_tf",
        name="mulran_gt_to_tf",
        output="screen",
        arguments=[
            "--calib", calib,
            "--map-frame", map_frame,
            "--base-frame", base_frame,
            "--sensor-frame", sensor_frame,
        ],
    )

    radar_params = {
        "map_file": meshfile,
        "map_frame": map_frame,
        "sensor_frame": sensor_frame,
        "sync_topic": sync_topic,
        "materials_file": materials_file,
        "material_id_air": material_id_air,
    }
    # launch_ros can't infer a type for an empty list (materials_file may
    # have no object_materials key, e.g. a single-object stand-in mesh) --
    # omit the key entirely rather than pass [], letting radar_simulator's
    # own declared default (std::vector<int64_t>{}) apply.
    if object_materials:
        radar_params["object_materials"] = object_materials

    radar_simulator = Node(
        package="radarays_ros",
        executable="radar_simulator",
        name="radar_simulator",
        output="screen",
        parameters=[radarparams, radar_params],
    )

    return [bag_play, gt_to_tf, radar_simulator]


def generate_launch_description():
    pkg_share = get_package_share_directory("radarays_ros")

    return LaunchDescription([
        DeclareLaunchArgument("meshfile", description=
            "Reconstructed MulRan mesh (see MIGRATION.md runbook step 2, "
            "mulran_lidar_to_cloud + lvr2_reconstruct)"),
        DeclareLaunchArgument("bagfile", description=
            "ros2 bag produced by `ros2 run radarays_ros mulran_radar_to_bag` "
            "(MIGRATION.md runbook step 3)"),
        DeclareLaunchArgument("calib", description=
            "MulRan calib_base2radar.txt (or calib_base2outer.txt) for the "
            "sensor extrinsic used by mulran_gt_to_tf"),
        DeclareLaunchArgument("rate", default_value="1.0"),
        DeclareLaunchArgument("materials_file",
            default_value=os.path.join(pkg_share, "config", "mulran_kaist02.yaml")),
        DeclareLaunchArgument("radarparams",
            default_value=os.path.join(pkg_share, "cfg", "mulran_kaist_dyncfg.yaml")),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("sensor_frame", default_value="radar_polar"),
        DeclareLaunchArgument("sync_topic", default_value="/Navtech/Polar"),
        OpaqueFunction(function=_launch_setup),
    ])
