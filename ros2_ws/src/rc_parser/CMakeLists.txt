cmake_minimum_required(VERSION 3.8)
project(rc_parser)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(RC_DEBUG_BUILD=1)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vdt_msgs REQUIRED)

add_library(rc_parser_lib src/rc_logic.cpp)
target_include_directories(rc_parser_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(rc_parser_lib rclcpp vdt_msgs)

add_executable(rc_node
  src/main.cpp
  src/rc_node.cpp
  src/rc_sbus.cpp
  src/rc_uart.cpp
)
target_link_libraries(rc_node rc_parser_lib)
ament_target_dependencies(rc_node rclcpp vdt_msgs)

install(TARGETS rc_parser_lib
  EXPORT export_rc_parser_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS rc_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  enable_testing()
  add_executable(rc_logic_test test/rc_logic_test.cpp)
  target_link_libraries(rc_logic_test rc_parser_lib)
  ament_target_dependencies(rc_logic_test rclcpp vdt_msgs)
  add_test(NAME rc_logic_test COMMAND rc_logic_test)
endif()

ament_export_targets(export_rc_parser_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp vdt_msgs)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()