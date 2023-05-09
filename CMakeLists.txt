cmake_minimum_required(VERSION 3.0.2)
project(mach1_navigation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mach1_msgs
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vel_switch_node src/vel_switch.cpp)

target_link_libraries(vel_switch_node ${catkin_LIBRARIES})
