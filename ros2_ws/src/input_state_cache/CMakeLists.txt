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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TimeoutFlags.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(input_cache_node
  src/main.cpp
  src/input_cache_node.cpp
  src/input_cache_logic.cpp
)

target_include_directories(input_cache_node PRIVATE include)
target_link_libraries(input_cache_node "${cpp_typesupport_target}")
ament_target_dependencies(input_cache_node rclcpp nav_msgs)

install(TARGETS input_cache_node DESTINATION lib/${PROJECT_NAME})

ament_package()