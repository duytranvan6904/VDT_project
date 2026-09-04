cmake_minimum_required(VERSION 3.0.2)
project(map_generator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(random_forest_sensing src/random_forest_sensing.cpp)
target_link_libraries(random_forest_sensing ${catkin_LIBRARIES})
