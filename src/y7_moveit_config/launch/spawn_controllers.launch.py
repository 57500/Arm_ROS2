from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ros2_control_controllers = [
        "manipulator_controller",
        "joint_state_broadcaster",
    ]
    ld = LaunchDescription()
    for controller in ros2_control_controllers:
        ld.add_action(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller],
                output="screen",
            )
        )
    return ld