cmake_minimum_required(VERSION 3.8)
project(fsm_state_machine)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_definitions(FSM_DEBUG_BUILD=1)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vdt_msgs REQUIRED)

add_library(fsm_logic src/fsm_logic.cpp)
target_include_directories(fsm_logic PUBLIC include)
ament_target_dependencies(fsm_logic rclcpp std_msgs nav_msgs vdt_msgs)

add_library(fsm_actions src/fsm_actions.cpp)
target_include_directories(fsm_actions PUBLIC include)
target_link_libraries(fsm_actions fsm_logic)
ament_target_dependencies(fsm_actions rclcpp std_msgs nav_msgs vdt_msgs)

add_library(fsm_node_lib src/fsm_node.cpp)
target_include_directories(fsm_node_lib PUBLIC include)
target_link_libraries(fsm_node_lib fsm_logic fsm_actions)
ament_target_dependencies(fsm_node_lib rclcpp std_msgs nav_msgs vdt_msgs)

add_executable(fsm_node src/main.cpp)
target_link_libraries(fsm_node fsm_node_lib)
ament_target_dependencies(fsm_node rclcpp std_msgs nav_msgs vdt_msgs)

install(TARGETS fsm_node DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(test/rosbag/test_fsm_rosbag.py TIMEOUT 60)

  ament_add_gtest(fsm_logic_test test/unit/fsm_logic_test.cpp)
  target_link_libraries(fsm_logic_test fsm_logic)

  ament_add_gtest(fsm_actions_test test/ros2/fsm_actions_test.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(fsm_actions_test fsm_actions gtest)

  ament_add_gtest(fsm_node_test test/ros2/fsm_node_test.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(fsm_node_test fsm_node_lib gtest)

  set_tests_properties(fsm_actions_test fsm_node_test PROPERTIES RESOURCE_LOCK ros_graph)
endif()

ament_package()