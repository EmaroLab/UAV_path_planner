cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

find_package(catkin REQUIRED COMPONENTS
        pcl_conversions
        pcl_ros
        roscpp
        rospy
        sensor_msgs
        geometry_msgs
        message_generation
        octomap_ros
        octomap_msgs
        )
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES path_planner
        CATKIN_DEPENDS
        #  DEPENDS system_lib
)

## Declare a C++ library
add_library(path_planner
        lib/path_planner.cpp
        include/path_planner.h
        include/surface_function.h
        include/cylinder.h
        include/plane.h
        lib/cylinder.cpp
        lib/plane.cpp)

add_executable(path_planner_node src/path_planner_node.cpp)
add_executable(point_cloud_downsampling src/point_cloud_downsampling.cpp)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
)
link_libraries(
        ${OCTOMAP_LIBRARIES}
)

target_link_libraries(
        point_cloud_downsampling
        ${catkin_LIBRARIES})

target_link_libraries(
        path_planner_node
        path_planner
        ${catkin_LIBRARIES}
        )
