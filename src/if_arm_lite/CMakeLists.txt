cmake_minimum_required(VERSION 3.8)
project(if_arm_lite)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch meshes urdf
		DESTINATION share/${PROJECT_NAME})

ament_package()