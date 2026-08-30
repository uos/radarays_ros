cmake_minimum_required(VERSION 3.8)
project(radarays_ros VERSION 1.0.1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

# Milestone 1 of the ROS 2 port: the message/service/action interfaces.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RadarMaterial.msg"
  "msg/RadarMaterials.msg"
  "msg/RadarModel.msg"
  "msg/RadarParams.msg"
  "srv/GetRadarParams.srv"
  "action/GenRadarImage.action"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs action_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_export_dependencies(rosidl_default_runtime)

# Pure radar-physics math (Ray/DirectedWave, fresnel, back-reflection
# shader, cone sampling, denoising, Perlin noise) -- no ROS 1/2 node
# dependency at all, just rmagine::core. Built and exported unconditionally
# (like the interfaces above) so other packages can depend on this single
# canonical copy of the physics instead of maintaining their own.
# radarays_gazebo_plugins does exactly that -- see its CMakeLists.txt and
# MIGRATION_HANDOFF.md, "Two independent physics copies", for why there
# used to be two and why there's only one now.
find_package(rmagine 2.3 COMPONENTS core REQUIRED)

add_library(radarays_core
  src/radarays_ros/radar_algorithms.cpp
)
target_include_directories(radarays_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(radarays_core PUBLIC
  rmagine::core
)
install(TARGETS radarays_core
  EXPORT export_radarays_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_targets(export_radarays_core HAS_LIBRARY_TARGET)
ament_export_dependencies(rmagine)

# Milestone 2: the node logic. CPU/Embree only for now -- GPU/OptiX parity
# is deferred workspace-wide, matching radarays_gazebo_plugins. RadarGPU.cpp
# is not ported yet; see MIGRATION.md.
option(RADARAYS_ROS_BUILD_NODES "Build the ported radar_simulator node (CPU/Embree)" ON)

if(RADARAYS_ROS_BUILD_NODES)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_package(image_transport REQUIRED)
  find_package(cv_bridge REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(OpenCV REQUIRED)
  find_package(yaml-cpp REQUIRED)
  find_package(rmagine 2.3
    COMPONENTS
      core
    OPTIONAL_COMPONENTS
      embree
      cuda
      optix
  )

  include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
  )

  add_library(radarays
    src/radarays_ros/ros_helper.cpp
    src/radarays_ros/Radar.cpp
  )
  target_include_directories(radarays PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(radarays
    radarays_core
    "${cpp_typesupport_target}"
    rmagine::core
    ${OpenCV_LIBS}
    yaml-cpp
  )
  ament_target_dependencies(radarays
    rclcpp
    tf2_ros
    sensor_msgs
    geometry_msgs
  )
  install(TARGETS radarays
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  if(TARGET rmagine::embree)
    find_package(OpenMP REQUIRED)

    add_library(radarays_cpu
      src/radarays_ros/RadarCPU.cpp
    )
    target_link_libraries(radarays_cpu
      radarays
      rmagine::embree
      OpenMP::OpenMP_CXX
    )
    ament_target_dependencies(radarays_cpu
      cv_bridge
    )
    install(TARGETS radarays_cpu
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

    add_executable(radar_simulator
      src/radar_simulator.cpp
    )
    target_link_libraries(radar_simulator
      radarays
      radarays_cpu
      rmagine::core
      ${OpenCV_LIBS}
    )
    ament_target_dependencies(radar_simulator
      rclcpp
      rclcpp_action
      tf2_ros
      image_transport
      cv_bridge
      sensor_msgs
    )
    install(TARGETS radar_simulator
      DESTINATION lib/${PROJECT_NAME}
    )

    add_executable(ray_reflection_test
      src/ray_reflection_test.cpp
    )
    target_link_libraries(ray_reflection_test
      radarays_core
      rmagine::core
      rmagine::embree
    )
    ament_target_dependencies(ray_reflection_test
      rclcpp
      tf2_ros
      visualization_msgs
      geometry_msgs
      std_msgs
    )
    install(TARGETS ray_reflection_test
      DESTINATION lib/${PROJECT_NAME}
    )

    add_executable(mesh_publisher
      src/mesh_publisher.cpp
    )
    target_link_libraries(mesh_publisher
      rmagine::core
      rmagine::embree
    )
    ament_target_dependencies(mesh_publisher
      rclcpp
      visualization_msgs
      geometry_msgs
      std_msgs
    )
    install(TARGETS mesh_publisher
      DESTINATION lib/${PROJECT_NAME}
    )
  else()
    message(WARNING "rmagine::embree not found -- radar_simulator will not be built.")
  endif()

  # GPU/OptiX radar path (RadarGPU/radar_algorithms.cu/image_algorithms.cu).
  # Needs the CUDA language enabled in *this* CMake project too -- rmagine
  # enabling it for its own build doesn't carry over to a different colcon
  # package. See MIGRATION_HANDOFF.md, "radarays GPU radar path", for the
  # OptiX SDK/driver ABI notes that also apply here.
  if(TARGET rmagine::optix)
    if(CMAKE_CUDA_COMPILER)
      enable_language(CUDA)
    else()
      find_package(CUDAToolkit QUIET)
      if(CUDAToolkit_FOUND)
        enable_language(CUDA)
      endif()
    endif()
  endif()

  if(TARGET rmagine::optix AND CMAKE_CUDA_COMPILER)
    # The pure CUDA kernels (move_waves/signal_shader/fresnel_split/
    # draw_signals + the plain-POD RadarMaterial) have no dependency on
    # `radarays`/Radar.hpp at all -- split out exactly like radarays_core
    # vs radarays above, so this piece alone can be cleanly exported
    # (`radarays` itself pulls in the rosidl typesupport target, which
    # isn't meant to be re-exported through another target's link
    # interface -- tried it, CMake's install(EXPORT) correctly refuses).
    # radarays_gazebo_plugins depends on this, not full `radarays_gpu`.
    add_library(radarays_gpu_core
      src/radarays_ros/radar_algorithms.cu
      src/radarays_ros/image_algorithms.cu
    )
    set_target_properties(radarays_gpu_core PROPERTIES
      CUDA_SEPARABLE_COMPILATION ON
    )
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.23" AND NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
      set_target_properties(radarays_gpu_core PROPERTIES CUDA_ARCHITECTURES all)
    endif()
    target_include_directories(radarays_gpu_core PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(radarays_gpu_core PUBLIC
      radarays_core
      rmagine::optix
    )
    install(TARGETS radarays_gpu_core
      EXPORT export_radarays_gpu_core
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
    ament_export_targets(export_radarays_gpu_core HAS_LIBRARY_TARGET)

    # RadarGPU.cpp itself (the ROS-Node-coupled class, like RadarCPU) --
    # not exported, matching radarays_cpu's precedent; only used here for
    # radar_simulator_gpu.
    add_library(radarays_gpu
      src/radarays_ros/RadarGPU.cpp
    )
    find_package(cv_bridge REQUIRED)
    target_link_libraries(radarays_gpu
      radarays
      radarays_gpu_core
      cv_bridge::cv_bridge
      ${OpenCV_LIBS}
    )
    install(TARGETS radarays_gpu
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )

    add_executable(radar_simulator_gpu
      src/radar_simulator_gpu.cpp
    )
    target_link_libraries(radar_simulator_gpu
      radarays
      radarays_gpu
      radarays_gpu_core
      rmagine::core
      ${OpenCV_LIBS}
    )
    ament_target_dependencies(radar_simulator_gpu
      rclcpp
      rclcpp_action
      tf2_ros
      image_transport
      cv_bridge
      sensor_msgs
    )
    install(TARGETS radar_simulator_gpu
      DESTINATION lib/${PROJECT_NAME}
    )
  elseif(TARGET rmagine::optix)
    message(WARNING "rmagine::optix found but no CUDA compiler -- radar_simulator_gpu will not be built.")
  endif()
endif()

# MulRan dataset validation tooling -- converts a MulRan sequence's
# radar-polar images + ground-truth poses into a ros2 bag, for comparing
# against this package's own simulated /radar/image. See
# MIGRATION_HANDOFF.md, "MulRan dataset work", for why this exists instead
# of porting file_player_mulran (a ROS1/Qt GUI app) wholesale.
install(
  PROGRAMS
    scripts/mulran_radar_to_bag.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mulran_radar_to_bag
)

# Companion tool: accumulates a sequence's Ouster LiDAR scans (+ calib +
# ground truth) into one world-frame point cloud for lvr2_reconstruct --
# see MIGRATION.md, "MulRan dataset work", runbook step 2.
install(
  PROGRAMS
    scripts/mulran_lidar_to_cloud.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mulran_lidar_to_cloud
)

# Companion tool: republishes a mulran_radar_to_bag.py bag's /gt as a live
# TF tree (map -> base_link) plus a static sensor extrinsic from a MulRan
# calib file (base_link -> radar/ouster) -- see MIGRATION.md, "MulRan
# dataset work", runbook step 4.
install(
  PROGRAMS
    scripts/mulran_gt_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mulran_gt_to_tf
)

# ROS 2 port of the dev/opti branch's automatic material-property
# optimizer, adapted for a real MulRan-reconstructed mesh (one material,
# not the original's hardcoded wall+glass split) and a real
# differential_evolution optimizer (not the original's infeasible
# 5**8-evaluation grid search) -- see MIGRATION.md, "MulRan dataset work",
# and the script's own header comment for the full rationale.
install(
  PROGRAMS
    scripts/radaray_opti.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME radaray_opti
)

# Launch files + example params/materials configs -- previously not
# installed at all (gap #1 in MIGRATION.md), meaning `ros2 launch
# radarays_ros ...` had nothing usable regardless of whether a .launch.py
# existed. Needed so mulran_sim.launch.py can locate its own default
# config/cfg files via get_package_share_directory() once installed.
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# radar_simulator's own first-ever regression fixture (previously zero
# registered tests in this package at all) -- a synthetic self-contained
# scene (testdata/wall_test.ply, a thin wall with a known distance/range,
# not a real dataset mesh or another package's world file), so this
# doesn't depend on anything outside radarays_ros itself. See MIGRATION.md,
# "radar_simulator regression fixture".
install(
  DIRECTORY testdata/
  DESTINATION share/${PROJECT_NAME}/testdata
)
install(
  PROGRAMS
    scripts/radarays_ros_fixture_harness.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME capture_radarays_ros_fixture
)
install(
  PROGRAMS
    scripts/radarays_ros_fixture_harness.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME compare_radarays_ros_fixture
)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)

  ament_add_test(radarays_ros_fixture_wall
    COMMAND bash -c "ros2 run radarays_ros capture_radarays_ros_fixture wall && ros2 run radarays_ros compare_radarays_ros_fixture wall"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )
endif()

ament_package()
