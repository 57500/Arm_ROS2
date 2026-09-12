#!/usr/bin/env python3
import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from control_msgs.action import FollowJointTrajectory


class TrajectoryProxy(Node):
    def __init__(self):
        super().__init__('trajectory_proxy')

        self.cb_group_ = ReentrantCallbackGroup()

        self.action_server_ = ActionServer(
            self,
            FollowJointTrajectory,
            '/trajectory_proxy/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            callback_group=self.cb_group_
        )

        self.action_client_ = ActionClient(
            self,
            FollowJointTrajectory,
            '/manipulator_controller/follow_joint_trajectory',
            callback_group=self.cb_group_
        )

        self.get_logger().info('TrajectoryProxy 已启动')
        self.get_logger().info('Action Server: /trajectory_proxy/follow_joint_trajectory')
        self.get_logger().info('Action Client 目标: /manipulator_controller/follow_joint_trajectory')

        thread = threading.Thread(target=self._wait_for_controller, daemon=True)
        thread.start()

    def _wait_for_controller(self):
        self.get_logger().info('后台等待控制器 Action Server...')
        self.action_client_.wait_for_server()
        self.get_logger().info('控制器 Action Server 已就绪！')

    def execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.get_logger().info(f'[1] 收到轨迹：{len(goal.trajectory.points)} 个路径点')

        if not self.action_client_.server_is_ready():
            self.get_logger().warn('[2] 控制器未就绪，等待中（最多10秒）...')
            ready = self.action_client_.wait_for_server(timeout_sec=10.0)
            if not ready:
                self.get_logger().error('[2] 等待超时！控制器 Action Server 不存在')
                self.get_logger().error('请确认仿真已启动且 manipulator_controller 已激活')
                self.get_logger().error('运行 ros2 action list 检查')
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = -1
                return result

        self.get_logger().info('[3] 控制器已就绪，发送 goal')

        def on_feedback(feedback_msg):
            goal_handle.publish_feedback(feedback_msg.feedback)

        send_goal_future = self.action_client_.send_goal_async(
            goal, feedback_callback=on_feedback
        )

        self.get_logger().info('[4] 等待控制器接受...')
        while not send_goal_future.done():
            time.sleep(0.01)

        gh_result = send_goal_future.result()
        if not gh_result.accepted:
            self.get_logger().warn('[5] 控制器拒绝了轨迹')
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = -1
            return result

        self.get_logger().info('[5] 控制器已接受，等待执行完成...')
        result_future = gh_result.get_result_async()

        while not result_future.done():
            time.sleep(0.01)

        result = result_future.result().result
        self.get_logger().info(f'[6] 执行完成，error_code={result.error_code}')

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryProxy()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()