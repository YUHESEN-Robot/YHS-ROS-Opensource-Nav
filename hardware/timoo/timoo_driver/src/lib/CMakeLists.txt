add_library(timoo_input input.cc)
target_link_libraries(timoo_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(timoo_input ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS timoo_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
