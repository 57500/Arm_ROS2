"""启动上位机桥接节点。

浏览器访问 http://127.0.0.1:<http_port>/
WebSocket 走 ws://127.0.0.1:<ws_port>/
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("http_port", default_value="8000"),
            DeclareLaunchArgument("ws_port", default_value="8001"),
            DeclareLaunchArgument("group_name", default_value="manipulator"),
            DeclareLaunchArgument("tip_link", default_value="link6"),
            DeclareLaunchArgument("mesh_package", default_value="if_arm_lite"),
            Node(
                package="if_arm_web",
                executable="bridge.py",
                output="screen",
                parameters=[
                    {
                        "http_port": ParameterValue(
                            LaunchConfiguration("http_port"), value_type=int
                        ),
                        "ws_port": ParameterValue(
                            LaunchConfiguration("ws_port"), value_type=int
                        ),
                        "group_name": ParameterValue(
                            LaunchConfiguration("group_name"), value_type=str
                        ),
                        "tip_link": ParameterValue(
                            LaunchConfiguration("tip_link"), value_type=str
                        ),
                        "mesh_package": ParameterValue(
                            LaunchConfiguration("mesh_package"), value_type=str
                        ),
                    }
                ],
            ),
        ]
    )
