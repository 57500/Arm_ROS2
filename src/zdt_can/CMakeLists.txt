cmake_minimum_required(VERSION 3.16)
project(zdt_can)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 开启常用告警，写错了编译期就能看到
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# 本库只依赖标准库和 Linux 头文件，不依赖 ROS。
add_library(zdt_can SHARED src/can_bus.cpp src/zdt_driver.cpp)
target_include_directories(zdt_can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS zdt_can
  EXPORT export_zdt_can
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_zdt_can HAS_LIBRARY_TARGET)
ament_package()
