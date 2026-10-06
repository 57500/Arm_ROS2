#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# =============================================================================
#  D2 验证工具（测试脚手架，不是学习目标，但每行都有注释，随便改）
#
#  用法（注意：ros2 run 要写【完整文件名】，带 .py —— 和 trajectory_proxy.py 一样）：
#     ros2 run arm_control_layer d2_test_client.py
#         → 模式A (cancel)：发一条 10 秒长轨迹，3 秒后自动 cancel，看下游是否当场停
#     ros2 run arm_control_layer d2_test_client.py --ros-args -p mode:=preempt
#         → 模式B (preempt)：发第一条，3 秒后【立刻发第二条】，看第一条是否被抢占
#
#  判定标准：
#     模式A：proxy 打印"取消已转发给下游"；机械臂当场停；
#            输出 status=5 (CANCELED)；取消响应 goals_canceling=1
#     模式B：proxy 打印"新目标抢占旧目标"；goal#1 → status=5；goal#2 → status=4；
#            机械臂不抽搐（两条轨迹不叠加）
#
#  变量来源（和参考文件同一套规矩）：
#     B) rclpy：Node / ActionClient / ExternalShutdownException
#     C) ROS 消息类型：JointState / JointTrajectoryPoint / FollowJointTrajectory
#                     / GoalStatus
#     D) 消息字段：msg.position（关节角数组）
#                  goal.trajectory.joint_names / .points / point.time_from_start
#                  future.result().status（终态码）/ .result.error_code（业务码）
# =============================================================================
import rclpy
from action_msgs.msg import GoalStatus                    # 终态码常量（4/5/6）
from control_msgs.action import FollowJointTrajectory     # 动作类型
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException     # Ctrl-C 正常退出的异常
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

PROXY_ACTION = '/trajectory_proxy/follow_joint_trajectory'   # 被测目标
STATUS_NAME = {                                            # 只为打印好读
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',       # 4
    GoalStatus.STATUS_CANCELED: 'CANCELED',         # 5
    GoalStatus.STATUS_ABORTED: 'ABORTED',           # 6
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',       # 2
    GoalStatus.STATUS_CANCELING: 'CANCELING',       # 3
}


class D2TestClient(Node):
    def __init__(self):
        super().__init__('d2_test_client')

        # ROS 参数：默认 cancel；命令行用 --ros-args -p mode:=preempt 覆盖
        self.mode_ = self.declare_parameter('mode', 'cancel').value
        # 预期要收到几个 goal 的最终结果（收齐了才退出）
        self.expected_ = 2 if self.mode_ == 'preempt' else 1

        self.pos_ = None            # 当前关节角（抄自 /joint_states）；None=还没收到
        self.sent_ = 0              # 已发出几个 goal
        self.done_ = 0              # 已收到几个最终结果
        self.cancel_sent_ = False   # 取消是否已发过（防重复）
        self.goal_handles_ = []     # 已被接受的 goal 把手（取消时要靠它）

        self.action_client_ = ActionClient(self, FollowJointTrajectory, PROXY_ACTION)

        # 订阅 /joint_states：轨迹起点必须是"当前位置"，否则第一点就会跳变
        self.create_subscription(JointState, '/joint_states', self.on_joint_state, 10)

        self.create_timer(0.5, self.send_first_goal)   # 反复尝试，直到有位置且 server 就绪
        self.create_timer(3.0, self.on_timer_3s)       # 3 秒后：取消 / 或发第二条

        self.get_logger().info(f'模式 = {self.mode_}（cancel=3 秒后取消 / preempt=3 秒后抢占）')

    # ── /joint_states 回调：只记第一帧（此时机械臂静止，作为轨迹起点）──
    def on_joint_state(self, msg):
        if self.pos_ is None:
            self.pos_ = list(msg.position)     # 用 list() 转成普通列表，不持有消息对象
            self.get_logger().info(f'拿到当前位姿：{len(self.pos_)} 个关节')

    # ── 造一条 10 秒长轨迹：当前位置 → 各关节 +delta，10 秒走完 ──
    def build_goal(self, delta):
        goal = FollowJointTrajectory.Goal()
        # joint_names 顺序要和控制器期望的一致（joint1..jointN）
        goal.trajectory.joint_names = [f'joint{i}' for i in range(1, len(self.pos_) + 1)]

        p0 = JointTrajectoryPoint()              # 起点：当前位置，时刻 0
        p0.positions = list(self.pos_)
        p0.time_from_start.sec = 0

        p1 = JointTrajectoryPoint()              # 终点：每关节 +delta，时刻 10 秒
        p1.positions = [q + delta for q in self.pos_]
        p1.time_from_start.sec = 10

        goal.trajectory.points = [p0, p1]
        return goal


    # ── 第一条 goal（发出去之前先确认 server 存在）──
    def send_first_goal(self):
        if self.pos_ is None or self.sent_ >= 1:
            return
        # wait_for_server(0.0) = 只问一次"server 在吗"，不阻塞
        if not self.action_client_.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn(f'等不到 {PROXY_ACTION}（proxy 启动了吗？）',
                                   throttle_duration_sec=2.0)
            return
        self.sent_ = 1
        self.send_goal(0.3, 1)

    # ── 3 秒定时器：两种模式的"动作戏"都在这里 ──
    def on_timer_3s(self):
        if self.sent_ < 1:
            return
        if self.mode_ == 'cancel':
            if self.cancel_sent_ or not self.goal_handles_:
                return
            self.cancel_sent_ = True
            self.get_logger().info('>>> 3 秒到，发送 cancel_goal_async()')
            # cancel_goal_async() 返回 Future；等响应回来才知道"准不准取消"
            self.goal_handles_[0].cancel_goal_async().add_done_callback(
                self.on_cancel_response)
        else:
            if self.sent_ >= 2:
                return
            self.sent_ = 2
            self.get_logger().info('>>> 3 秒到，抢占：立刻发第 2 条（不主动发 cancel）')
            self.send_goal(-0.3, 2)

    def send_goal(self, delta, tag):
        self.get_logger().info(f'>>> 发 goal#{tag}：10 秒长轨迹，delta={delta}')
        fut = self.action_client_.send_goal_async(
            self.build_goal(delta),
            feedback_callback=self.on_feedback,
        )
        fut.add_done_callback(lambda f: self.on_goal_response(f, tag))

    def on_goal_response(self, future, tag):
        gh = future.result()                 # 已完成，可立即取：下游的 goal 把手
        if not gh.accepted:
            self.get_logger().error(f'goal#{tag} 被拒绝')
            return
        self.get_logger().info(f'goal#{tag} 已被接受')
        self.goal_handles_.append(gh)
        gh.get_result_async().add_done_callback(lambda f: self.on_result(f, tag))

    def on_cancel_response(self, future):
        resp = future.result()
        # CancelGoal 服务的响应只有两个字段：
        #   return_code     : 0=ERROR_NONE 1=ERROR_REJECTED 2=UNKNOWN_GOAL_ID 3=GOAL_ALREADY_TERMINATED
        #   goals_canceling : 被接受取消的 goal 列表
        # （注意：没有 goals_killing 这个字段，别写错）
        self.get_logger().info(f'取消响应：goals_canceling={len(resp.goals_canceling)} '
                               f'return_code={resp.return_code}')
        if not resp.goals_canceling:
            self.get_logger().error('取消被拒绝 → 检查 proxy 的 cancel_callback 是否 return ACCEPT')

    def on_feedback(self, feedback_msg):
        pos = feedback_msg.feedback.actual.positions
        if pos:
            self.get_logger().info(f'  feedback: actual={[round(p, 3) for p in pos]}',
                                   throttle_duration_sec=1.0)

    def on_result(self, future, tag):
        status = future.result().status                    # 终态码（4/5/6）
        code = future.result().result.error_code           # 业务码
        self.get_logger().info(f'<<< goal#{tag} 结束：status={status} '
                               f'({STATUS_NAME.get(status, "?")}) error_code={code}')
        self.done_ += 1
        if self.done_ >= self.expected_:
            self.get_logger().info('测试结束')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = D2TestClient()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass                                  # Ctrl-C 正常退出，不报 RCLError
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

