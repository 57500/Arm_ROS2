# ============================================================
#  bringup.launch.py  —— 【交给你写】
#
#  目标：一条命令启动完整链路
#      ros2 launch y7_bringup bringup.launch.py
#      ros2 launch y7_bringup bringup.launch.py use_rviz:=false
#      ros2 launch y7_bringup bringup.launch.py use_proxy:=false
#
#  验收标准（写完自测）：
#      1) 启动日志里：joint_state_broadcaster 和 manipulator_controller 都
#         "Configured and activated"
#      2) ros2 action info /trajectory_proxy/follow_joint_trajectory -t
#         → Client count = 1
#      3) RViz 里拖动末端 → Plan & Execute 成功（不需要另开终端起 proxy）
#      4) Ctrl-C 一次，所有进程一起退出
#
#  填写规则：
#      - 每个【TODO】由你补，填完就运行，报错就修
#      - 只讲必要的概念；语法边界问题不讨论
#      - 查字段取值用：ros2 pkg executables <包名>
#      - 参考（只看结构，不抄）：archive/launch_lessons/lesson01_minimal.launch.py
# ============================================================

import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ─────────── TODO-1：两个开关 ───────────
    # 声明 use_rviz（默认 'true'）、use_proxy（默认 'true'）
    # 提示：DeclareLaunchArgument('名字', default_value='true', description='...')
    #      再各取一个 LaunchConfiguration('名字') 备用
    declared_arguments=[
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='是否启动Rviz',
        ),
        DeclareLaunchArgument(
            'use_proxy',
            default_value='true',
            description='是否启动 trajectory_proxy 中间层',
        )
    ]
    use_rviz=LaunchConfiguration('use_rviz')
    use_proxy=LaunchConfiguration('use_proxy')

    # ─────────── TODO-2：读 MoveIt 配置 ───────────
    # 机器人名 'y7_arm_lite'，包名 'y7_moveit_config'
    # 提示：MoveItConfigsBuilder(...).to_moveit_configs()
    moveit_config = MoveItConfigsBuilder(
        'y7_arm_lite',
        package_name='y7_moveit_config',
    ).to_moveit_configs()

    # ─────────── TODO-3：取出 URDF 字符串 ───────────
    # 它在 moveit_config 里，是个字典（键名就是 'robot_description'）
    robot_description = moveit_config.robot_description

    # ─────────── TODO-4：控制器配置文件路径 ───────────
    # ① get_package_share_directory('y7_moveit_config') 拿到安装目录
    # ② os.path.join 拼上 'config' 和 'ros2_controllers.yaml'
    y7_moveit_share = get_package_share_directory('y7_moveit_config')
    controllers_yaml = os.path.join(y7_moveit_share,'config','ros2_controllers.yaml')

    # ─────────── TODO-5：robot_state_publisher ───────────
    # 字段：package / executable / name / output='screen' / parameters=[robot_description]
    # 查 executable 名：ros2 pkg executables robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ─────────── TODO-6：ros2_control_node（ros2_control 的大脑）───────────
    # parameters 只给 controllers_yaml（不要给 robot_description！）
    # remappings 一行：('/controller_manager/robot_description', '/robot_description')
    #   ^ 这一行的作用：让它去 /robot_description 话题拿 URDF（由 rsp 发布）
    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[controllers_yaml],
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
        ],
    )

    # ─────────── TODO-7：中间控制层 trajectory_proxy ───────────
    # package='arm_control_layer'；executable 注意带 .py
    # 加 condition=IfCondition(use_proxy)
    proxy_node = Node(
        package='arm_control_layer',
        executable='trajectory_proxy.py',
        name='trajectory_proxy',
        output='screen',
        condition=IfCondition(use_proxy),
    )

    # ─────────── TODO-8：Include 三个现成的 launch ───────────
    # y7_moveit_config 里已有：spawn_controllers.launch.py / move_group.launch.py / moveit_rviz.launch.py
    # 提示：IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(...)))
    # moveit_rviz 那个要加 condition=IfCondition(use_rviz)
    y7_moveit_launch_dir = os.path.join(get_package_share_directory('y7_moveit_config'), 'launch')

    spawn_controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(y7_moveit_launch_dir, 'spawn_controllers.launch.py'))
    )
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(y7_moveit_launch_dir, 'move_group.launch.py'))
    )
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(y7_moveit_launch_dir, 'moveit_rviz.launch.py')),
        condition=IfCondition(use_rviz),
    )


    # ─────────── TODO-9：返回 ───────────
    # 把 declared_arguments 和上面所有动作装进 LaunchDescription
    return LaunchDescription(
        declared_arguments + [
            rsp_node,
            control_node,
            proxy_node,
            spawn_controllers_launch,
            move_group_launch,
            rviz_launch,
        ]
    )

