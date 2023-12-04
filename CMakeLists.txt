cmake_minimum_required(VERSION 3.2)
project(radarays_ros VERSION 1.0.1)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    sensor_msgs
    tf2_ros
    dynamic_reconfigure
    mesh_msgs
    image_transport
    cv_bridge
    actionlib_msgs
)

generate_dynamic_reconfigure_options(
  cfg/RadarModel.cfg
  cfg/RayReflection.cfg
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

find_package(rmagine 2.2.1...
COMPONENTS
    core
OPTIONAL_COMPONENTS
    embree
    cuda
    optix
)

include_directories(${rmagine_INCLUDE_DIRS})

set(RADARAYS_ROS_LIBRARIES radarays)
set(RADARAYS_ROS_DEFINITIONS)

if(TARGET rmagine::optix)
    list(APPEND RADARAYS_ROS_LIBRARIES radarays_gpu)
    set(RADARAYS_WITH_GPU True)
    list(APPEND RADARAYS_ROS_DEFINITIONS -DRADARAYS_WITH_GPU)
endif()


add_message_files(
  FILES
    RadarMaterial.msg
    RadarMaterials.msg
    RadarModel.msg
    RadarParams.msg
)

add_service_files(
  FILES
    GetRadarParams.srv
)

add_action_files(
  DIRECTORY action
  FILES 
    GenRadarImage.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
    actionlib_msgs
)

catkin_package(
    INCLUDE_DIRS 
        include
    CATKIN_DEPENDS 
        roscpp
        geometry_msgs
        sensor_msgs
        tf2_ros
        actionlib_msgs
    DEPENDS 
        rmagine
    LIBRARIES
        ${RADARAYS_ROS_LIBRARIES}
)

if(RADARAYS_WITH_GPU)
    find_package(CUDA REQUIRED)
    enable_language(CUDA)
    include_directories(${CUDA_INCLUDE_DIRS})
endif()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(radarays
    src/radarays_ros/radar_algorithms.cpp
    src/radarays_ros/ros_helper.cpp
    src/radarays_ros/Radar.cpp
    src/radarays_ros/RadarCPU.cpp
)

add_dependencies(radarays 
   ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS}
   ${PROJECT_NAME}_gencfg
)

target_link_libraries(radarays
    rmagine::core
    rmagine::embree
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

install(TARGETS radarays
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

if(RADARAYS_WITH_GPU)
    add_library(radarays_gpu SHARED
        src/radarays_ros/image_algorithms.cu
        src/radarays_ros/radar_algorithms.cu
        src/radarays_ros/RadarGPU.cpp
    )
    add_dependencies(radarays_gpu 
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg
    )
    target_link_libraries(radarays_gpu
        ${CUDA_LIBRARIES}
        radarays
        rmagine::optix
    )
    install(TARGETS radarays_gpu
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
    )
endif(RADARAYS_WITH_GPU)

add_executable(radar_simulator
    src/radar_simulator.cpp
)

add_dependencies(radar_simulator 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(radar_simulator
    ${catkin_LIBRARIES}
    rmagine::core
    rmagine::embree
    ${OpenCV_LIBS}
    radarays
)

target_compile_definitions(radar_simulator
    PUBLIC ${RADARAYS_ROS_DEFINITIONS}
)

if(RADARAYS_WITH_GPU)
    target_compile_definitions(radar_simulator PUBLIC RADARAYS_WITH_GPU)
    target_link_libraries(radar_simulator
        radarays_gpu
    )
endif(RADARAYS_WITH_GPU)

add_executable(ray_reflection_test src/ray_reflection_test.cpp)

add_dependencies(ray_reflection_test 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(ray_reflection_test
    ${catkin_LIBRARIES}
    rmagine::core
    rmagine::embree
    ${OpenCV_LIBS}
    radarays
)
