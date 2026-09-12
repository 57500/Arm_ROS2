#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateListener(Node):
    def __init__(self):
        super().__init__('joint_state_listener')

        self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )
        self.get_logger().info('joint_state_listener节点已启动')

        self.print_interval=0.5
        self.last_print_time=self.get_clock().now()

    def callback(self, msg):
        now = self.get_clock().now()
        if (now - self.last_print_time).nanoseconds / 1e9 >= self.print_interval:
            for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
                self.get_logger().info(f"  {name:>8s}: pos={pos:+8.3f}  vel={vel:+8.3f}")
            self.last_print_time = now


def main(args=None):
    rclpy.init(args=args)
    node=JointStateListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()