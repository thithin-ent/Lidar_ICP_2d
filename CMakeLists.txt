cmake_minimum_required(VERSION 3.0.2)
project(Lidar_ICP_2d)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES Lidar_ICP_2d
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs tf2 tf2_ros
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(Lidar_ICP_2d src/Lidar_ICP.cpp src/Lidar_ICP_node.cpp)
target_link_libraries(Lidar_ICP_2d ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
