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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(vdt_msgs REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_fsm_rosbag.py TIMEOUT 60)
  ament_add_gtest(fsm_logic_test test/fsm_logic_test.cpp)
  target_link_libraries(fsm_logic_test fsm_logic)
  ament_add_gtest(fsm_actions_test test/fsm_actions_test.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(fsm_actions_test fsm_actions gtest)
  ament_add_gtest(fsm_node_test test/fsm_node_test.cpp SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(fsm_node_test fsm_node_lib gtest)
  set_tests_properties(fsm_actions_test fsm_node_test PROPERTIES RESOURCE_LOCK ros_graph)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(vdt_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(fsm_node
  src/main.cpp
  src/fsm_node.cpp
  src/fsm_logic.cpp
  src/fsm_actions.cpp
)

target_include_directories(fsm_node PRIVATE include)
target_link_libraries(fsm_node "${cpp_typesupport_target}")
ament_target_dependencies(fsm_node rclcpp std_msgs nav_msgs vdt_msgs)

install(TARGETS fsm_node DESTINATION lib/${PROJECT_NAME})

enable_testing()
add_executable(fsm_logic_test test/fsm_logic_test.cpp src/fsm_logic.cpp)
target_include_directories(fsm_logic_test PRIVATE include)
target_link_libraries(fsm_logic_test "${cpp_typesupport_target}")
add_test(NAME fsm_logic_test COMMAND fsm_logic_test)

ament_package()