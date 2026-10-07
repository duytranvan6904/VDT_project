cmake_minimum_required(VERSION 3.8)
project(vision_interface_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(vdt_msgs REQUIRED)

add_executable(vision_interface_bridge_node src/vision_interface_bridge_node.cpp)
target_include_directories(vision_interface_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vision_interface_bridge_node
  rclcpp std_msgs sensor_msgs vision_msgs vdt_msgs)

install(TARGETS vision_interface_bridge_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(bridge_logic_test test/bridge_logic_test.cpp)
  target_include_directories(bridge_logic_test PRIVATE include)
  target_compile_options(bridge_logic_test PRIVATE -UNDEBUG)
  add_test(NAME bridge_logic_test COMMAND bridge_logic_test)
endif()

ament_package()