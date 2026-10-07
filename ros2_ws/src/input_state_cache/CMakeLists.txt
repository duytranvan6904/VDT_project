cmake_minimum_required(VERSION 3.8)
project(input_state_cache)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(INPUT_CACHE_DEBUG_BUILD=1)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vdt_msgs REQUIRED)

add_executable(input_cache_node
  src/main.cpp
  src/input_cache_node.cpp
  src/input_cache_logic.cpp
)
target_include_directories(input_cache_node PRIVATE include)
ament_target_dependencies(input_cache_node rclcpp nav_msgs vdt_msgs)

install(TARGETS input_cache_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_gtest(input_cache_logic_test
    test/unit/input_state_logic_test.cpp
    src/input_cache_logic.cpp)
  target_include_directories(input_cache_logic_test PRIVATE include)
  ament_target_dependencies(input_cache_logic_test rclcpp nav_msgs vdt_msgs)

  add_launch_test(test/ros2/test_input_cache_node_ros.py TIMEOUT 60)
  add_launch_test(test/rosbag/test_input_cache_rosbag.py TIMEOUT 120)
endif()

ament_package()