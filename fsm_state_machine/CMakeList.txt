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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VisionMarker.msg"
  "msg/AltEstimate.msg"
  "msg/RcFsmInput.msg"
  "msg/TimeoutFlags.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(fsm_node
  src/main.cpp
  src/fsm_node.cpp
  src/fsm_logic.cpp
  src/fsm_actions.cpp
)

target_include_directories(fsm_node PRIVATE include)
target_link_libraries(fsm_node "${cpp_typesupport_target}")
ament_target_dependencies(fsm_node rclcpp std_msgs nav_msgs)

install(TARGETS fsm_node DESTINATION lib/${PROJECT_NAME})

ament_package()