cmake_minimum_required(VERSION 3.8)
project(px4_state_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(fsm_state_machine REQUIRED)

add_executable(px4_state_bridge_node src/px4_bridge_node.cpp)
target_include_directories(px4_state_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(px4_state_bridge_node
  rclcpp px4_msgs nav_msgs geometry_msgs std_msgs tf2_ros fsm_state_machine)

install(TARGETS px4_state_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(frame_conversion_test test/frame_conversion_test.cpp)
  target_include_directories(frame_conversion_test PRIVATE include)
  target_compile_options(frame_conversion_test PRIVATE -UNDEBUG)
  add_test(NAME frame_conversion_test COMMAND frame_conversion_test)
endif()

ament_package()