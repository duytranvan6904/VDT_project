# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target vdt_msgs::vdt_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${vdt_msgs_TARGETS}.
if(vdt_msgs_TARGETS AND NOT TARGET vdt_msgs::vdt_msgs)
  add_library(vdt_msgs::vdt_msgs INTERFACE IMPORTED)
  set_target_properties(vdt_msgs::vdt_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${vdt_msgs_TARGETS}")
endif()
