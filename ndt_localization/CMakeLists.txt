cmake_minimum_required(VERSION 3.0.2)
project(ndt_localization)


add_compile_options(-std=c++14)

find_package(PCL REQUIRED QUIET)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  ndt_omp
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  nmea_ros_driver
)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ndt_localization
#  CATKIN_DEPENDS geometry_msgs nav_msgs ndt_omp pcl_conversions pcl_ros roscpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros
#  DEPENDS system_lib
)



include_directories(
  include
  include/ndt_localization
  ${catkin_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/ndt_localization_node.cpp src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
