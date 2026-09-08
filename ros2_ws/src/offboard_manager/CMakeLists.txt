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
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlannerOutput.msg"
  "msg/OffboardStatus.msg"
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(offboard_node
  src/main.cpp
  src/offboard_node.cpp
  src/offboard_logic.cpp
)

target_include_directories(offboard_node PRIVATE include)
target_link_libraries(offboard_node "${cpp_typesupport_target}")
ament_target_dependencies(offboard_node rclcpp std_msgs px4_msgs)

install(TARGETS offboard_node DESTINATION lib/${PROJECT_NAME})

ament_package()