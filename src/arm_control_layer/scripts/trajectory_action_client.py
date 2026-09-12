#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState


class TrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('trajectory_action_client')

        self.action_client_=ActionClient(
            self,
            FollowJointTrajectory,
            '/manipulator_controller/follow_joint_trajectory'
        )

        self.current_positions=None
        self.subscription_=self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.timer_=self.create_timer(0.5,self.try_send_goal)
        self.send_=False

    def joint_state_callback(self,msg):
            self.current_positions=list(msg.position)

    def try_send_goal(self):
        while not self.action_client_.wait_for_server(1.0):
            self.get_logger().warn("Waiting for FollowJointTrajectory server...")

        if self.send_ or self.current_positions is None:
            return

        self.send_ = True

        goal_msg_=FollowJointTrajectory.Goal()

        goal_msg_.trajectory.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4',
            'joint5', 'joint6', 'joint7'
        ]

        point1 = JointTrajectoryPoint()
        point1.positions = self.current_positions   
        point1.time_from_start.sec = 0

        point2 = JointTrajectoryPoint()
        point2.positions = [p + 0.1 for p in point1.positions]              
        point2.time_from_start.sec = 2

        goal_msg_.trajectory.points=[point1,point2]

        send_goal_future_=self.action_client_.send_goal_async(
            goal_msg_,
            feedback_callback=self.feedback_callback
        )

        send_goal_future_.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self,future):
        goal_handle_=future.result()
        if not goal_handle_.accepted:
            return

        result_future_=goal_handle_.get_result_async()
        result_future_.add_done_callback(self.result_callback)

    def feedback_callback(self,feedback_msg):
        fb=feedback_msg.feedback

        for i,name in enumerate(fb.joint_names):
            self.get_logger().info(f"joint_name: {name}")
            self.get_logger().info(f"  desired position: {fb.desired.positions[i]}")
            self.get_logger().info(f"  actual position: {fb.actual.positions[i]}")
    
    def result_callback(self,future):
        result_=future.result().result
        if result_.error_code==0:
            self.get_logger().info(f"success")
            return

        self.get_logger().info(f"fail")

def main(args=None):
    rclpy.init(args=args)
    node=TrajectoryActionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
        main()