cmake_minimum_required(VERSION 3.8)
project(arm_control_layer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  scripts/joint_state_listener.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/trajectory_feedback_listener.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/trajectory_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/trajectory_action_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/trajectory_proxy.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/d2_test_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
