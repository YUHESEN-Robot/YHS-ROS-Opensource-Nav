cmake_minimum_required(VERSION 2.8.3)
project(navi_tools_rviz_plugin)

find_package(catkin REQUIRED COMPONENTS 
  rviz 
  geometry_msgs 
  std_msgs 
  actionlib_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  rosgraph_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

set(CMAKE_AUTOMOC ON)

## This plugin includes Qt widgets, so we must include Qt.
## We'll use the version that rviz used so they are compatible.
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

add_definitions(-DQT_NO_KEYWORDS)


set(SOURCE_FILES
  src/multi_navi_goal_panel.cpp
  ${MOC_FILES}
)


add_library(${PROJECT_NAME} ${SOURCE_FILES})


target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})



