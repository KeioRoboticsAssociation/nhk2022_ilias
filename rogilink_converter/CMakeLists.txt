cmake_minimum_required(VERSION 3.0.2)
project(rogilink_converter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  rogi_link_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES swerve_odom_publisher
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
#   include
  ${catkin_INCLUDE_DIRS}
)
add_executable(rogilink_converter src/main.cpp src/rogilink_converter.cpp)
target_link_libraries(rogilink_converter ${catkin_LIBRARIES})
add_dependencies(rogilink_converter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})