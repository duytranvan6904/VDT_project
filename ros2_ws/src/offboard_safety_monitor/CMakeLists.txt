cmake_minimum_required(VERSION 3.8)
project(offboard_safety_monitor)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(SAFETY_MONITOR_DEBUG_BUILD=1)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(offboard_manager REQUIRED)

add_executable(safety_monitor_node
  src/main.cpp
  src/safety_monitor_node.cpp
  src/safety_logic.cpp
)

target_include_directories(safety_monitor_node PRIVATE include)
ament_target_dependencies(safety_monitor_node rclcpp std_msgs px4_msgs offboard_manager)

install(TARGETS safety_monitor_node DESTINATION lib/${PROJECT_NAME})

enable_testing()
add_executable(safety_logic_test test/safety_logic_test.cpp src/safety_logic.cpp)
target_include_directories(safety_logic_test PRIVATE include)
add_test(NAME safety_logic_test COMMAND safety_logic_test)

ament_package()