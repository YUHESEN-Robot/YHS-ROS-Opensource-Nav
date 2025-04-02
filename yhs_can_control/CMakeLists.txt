cmake_minimum_required(VERSION 2.8.3)
project(yhs_can_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rostopic
  rospy
  std_msgs
  message_generation
  yhs_msgs
  tf2
  tf2_ros
)


## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_node
  CATKIN_DEPENDS roscpp rostopic rospy std_msgs message_runtime 
#  DEPENDS system_lib
)


link_directories(
  include
  include/yhs_can_control
  ${catkin_INCLUDE_DIRS} yhs_can_control
)

include_directories(
  include
  include/yhs_can_control
  ${catkin_INCLUDE_DIRS} yhs_can_control
)

add_executable(${PROJECT_NAME}_node src/yhs_dt_can_control.cpp src/yhs_mk_can_control.cpp
src/yhs_fw_can_control.cpp src/yhs_fr_can_control.cpp src/main.cpp)


add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)



