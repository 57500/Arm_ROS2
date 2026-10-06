#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_=self.create_publisher(
            JointTrajectory,
            '/manipulator_controller/joint_trajectory',
            10
        )

        self.current_positions=None
        self.subscription_=self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.timer_=self.create_timer(0.5,self.try_publish)
        self.published_=False

    def joint_state_callback(self,msg):
        self.current_positions=list(msg.position)

    def try_publish(self):
        if self.published_ or self.current_positions is None:
            return
        self.published_=True

        msg = JointTrajectory()
        msg.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4',
            'joint5', 'joint6', 'joint7'
        ]

        point1 = JointTrajectoryPoint()
        point1.positions = self.current_positions   
        point1.time_from_start.sec = 0

        point2 = JointTrajectoryPoint()
        point2.positions = self.current_positions.copy()
        point2.positions[0] = 0.0                 
        point2.time_from_start.sec = 2

        msg.points = [point1, point2]
        self.publisher_.publish(msg)
        self.get_logger().info(f'轨迹已发布！joint1: {self.current_positions[0]:.3f} → 0.0')

def main(args=None):
    rclpy.init(args=args)
    node=TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__=='__main__':
        main()