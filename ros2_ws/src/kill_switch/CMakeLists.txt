cmake_minimum_required(VERSION 3.8)
project(kill_switch)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(KILL_SWITCH_DEBUG_BUILD=1)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rc_parser REQUIRED)

add_executable(kill_switch_node
  src/main.cpp
  src/kill_switch_node.cpp
  src/kill_switch_logic.cpp
)

target_include_directories(kill_switch_node PRIVATE include)
ament_target_dependencies(kill_switch_node rclcpp std_msgs px4_msgs rc_parser)

install(TARGETS kill_switch_node DESTINATION lib/${PROJECT_NAME})

ament_package()