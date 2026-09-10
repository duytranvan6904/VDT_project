cmake_minimum_required(VERSION 3.8)
project(vdt_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/../../../System_Params.yaml
	DESTINATION share/${PROJECT_NAME})

ament_package()
