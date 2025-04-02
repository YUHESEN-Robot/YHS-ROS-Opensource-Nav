cmake_minimum_required(VERSION 3.0.2)
project(yhs_nav)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs tf2_geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


