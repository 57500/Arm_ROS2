cmake_minimum_required(VERSION 3.16)
project(if_arm_web)

find_package(ament_cmake REQUIRED)

# 桥接节点（Python），用 ros2 run/launch 启动
install(PROGRAMS scripts/bridge.py DESTINATION lib/${PROJECT_NAME})

# 前端静态文件 + launch
install(DIRECTORY launch web DESTINATION share/${PROJECT_NAME})

ament_package()
