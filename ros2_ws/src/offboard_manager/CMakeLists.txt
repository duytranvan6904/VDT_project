cmake_minimum_required(VERSION 3.8)
project(offboard_manager)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(OFFBOARD_DEBUG_BUILD=1)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(vdt_msgs REQUIRED)

add_executable(offboard_node
  src/main.cpp
  src/offboard_node.cpp
  src/offboard_logic.cpp
)
target_include_directories(offboard_node PRIVATE include)
ament_target_dependencies(offboard_node rclcpp std_msgs px4_msgs vdt_msgs)

install(TARGETS offboard_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  enable_testing()
  add_executable(offboard_logic_test test/offboard_logic_test.cpp src/offboard_logic.cpp)
  target_include_directories(offboard_logic_test PRIVATE include)
  ament_target_dependencies(offboard_logic_test rclcpp std_msgs px4_msgs vdt_msgs)
  add_test(NAME offboard_logic_test COMMAND offboard_logic_test)
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()