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
find_package(fsm_state_machine REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RcChannelsRaw.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(rc_node
  src/main.cpp
  src/rc_node.cpp
  src/rc_logic.cpp
  src/rc_sbus.cpp
  src/rc_uart.cpp
)

target_include_directories(rc_node PRIVATE include)
target_link_libraries(rc_node "${cpp_typesupport_target}")
ament_target_dependencies(rc_node rclcpp fsm_state_machine)

install(TARGETS rc_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)
ament_package()