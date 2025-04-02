add_executable(timoo_cloud_node cloud_node.cc convert.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(timoo_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(timoo_cloud_node timoo_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS timoo_cloud_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(timoo_cloud_nodelet cloud_nodelet.cc convert.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(timoo_cloud_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(timoo_cloud_nodelet timoo_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS timoo_cloud_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(timoo_transform_node transform_node.cc transform.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(timoo_transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(timoo_transform_node timoo_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS timoo_transform_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(timoo_transform_nodelet transform_nodelet.cc transform.cc pointcloudXYZIR.cc organized_cloudXYZIR.cc)
add_dependencies(timoo_transform_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(timoo_transform_nodelet timoo_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS timoo_transform_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
