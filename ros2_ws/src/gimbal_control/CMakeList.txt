cmake_minimum_required(VERSION 3.8)
project(gimbal_control)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(GIMBAL_DEBUG_BUILD=1)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fsm_state_machine REQUIRED)

add_executable(gimbal_node
  src/main.cpp
  src/gimbal_node.cpp
  src/gimbal_logic.cpp
)

target_include_directories(gimbal_node PRIVATE include)
ament_target_dependencies(gimbal_node rclcpp std_msgs nav_msgs fsm_state_machine)

install(TARGETS gimbal_node DESTINATION lib/${PROJECT_NAME})

ament_package()