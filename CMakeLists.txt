cmake_minimum_required(VERSION 3.2)
project(radarays_ros)

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

find_package(rmagine REQUIRED)
include_directories(${rmagine_INCLUDE_DIRS})

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
)

include_directories(
    include
)

#add_executable(mesh_publisher src/mesh_publisher.cpp)

#add_dependencies(mesh_publisher 
#    ${${PROJECT_NAME}_EXPORTED_TARGETS}
#    ${catkin_EXPORTED_TARGETS}
#    ${PROJECT_NAME}_gencfg
#)

## Specify libraries to link a library or executable target against
#target_link_libraries(mesh_publisher
#    ${catkin_LIBRARIES}
#    ${rmagine_LIBRARIES}
#)

find_package(CUDA)
if(CUDA_FOUND)
    message(WARNING "RADARAYS_ROS: CUDA_FOUND")
    enable_language(CUDA)
    include_directories(${CUDA_INCLUDE_DIRS})
endif(CUDA_FOUND)

set(CUDA_FOUND 0)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_library(radarays
    src/radarays_ros/radar_algorithms.cpp
)

target_link_libraries(
    radarays
    ${rmagine_LIBRARIES}
)

if(CUDA_FOUND)
    add_library(radarays_cuda SHARED
        src/radarays_ros/image_algorithms.cu
    )
endif(CUDA_FOUND)


add_executable(radar_simulator 
    src/radar_simulator.cpp)

add_dependencies(radar_simulator 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(radar_simulator
    ${catkin_LIBRARIES}
    ${rmagine_LIBRARIES}
    ${OpenCV_LIBS}
    radarays
)


if(CUDA_FOUND)
    target_compile_definitions(radar_simulator PUBLIC WITH_CUDA)
    target_link_libraries(radar_simulator
        radarays_cuda
    )
endif(CUDA_FOUND)

add_executable(ray_reflection_test src/ray_reflection_test.cpp)

add_dependencies(ray_reflection_test 
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)

## Specify libraries to link a library or executable target against
target_link_libraries(ray_reflection_test
    ${catkin_LIBRARIES}
    ${rmagine_LIBRARIES}
    ${OpenCV_LIBS}
    radarays
)
