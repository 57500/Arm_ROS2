import launch
import launch_ros.actions
import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share=get_package_share_directory('if_arm_lite')

    urdf_file=os.path.join(pkg_share,'urdf','if_arm_lite.urdf')


    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description':open(urdf_file).read()
            }]
        ),

        launch_ros.actions.Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
        ),

        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
        ),
    ])