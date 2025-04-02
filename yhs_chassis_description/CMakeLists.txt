cmake_minimum_required(VERSION 2.8.3)
project(yhs_chassis_description)



find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
  rviz
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES autolabor_description
#  CATKIN_DEPENDS joint_state_publisher robot_state_publisher rviz
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


