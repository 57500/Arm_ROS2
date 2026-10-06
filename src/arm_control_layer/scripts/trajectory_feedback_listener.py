#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState

class TrajectoryFeedbackListener(Node):
    def __init__(self):
        super().__init__('trajectory_feedback_listener')
        self.subscription_=self.create_subscription(
            JointTrajectoryControllerState,
            '/manipulator_controller/controller_state',
            self.feedback_callback,
            10
        )
        self.last_print_time=self.get_clock().now()
        self.print_interval=0.5

    def feedback_callback(self,msg):
        now=self.get_clock().now()
        if(now-self.last_print_time).nanoseconds/1e9<self.print_interval:
            return

        self.last_print_time=now


        self.get_logger().info(f"=== 轨迹执行反馈 ===")
        for name,reference,feedback,error in zip(msg.joint_names,msg.
            reference.positions,msg.feedback.positions,msg.error.positions):

            self.get_logger().info(f"关节: {name}")
            self.get_logger().info(f"期望位置: {reference}")
            self.get_logger().info(f"实际位置: {feedback}")
            self.get_logger().info(f"位置误差: {error}")

def main(args=None):
    rclpy.init(args=args)
    node=TrajectoryFeedbackListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()