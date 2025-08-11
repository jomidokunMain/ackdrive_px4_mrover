#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import time


class CmdVelToEncoderNode(Node):
    def __init__(self):
        super().__init__('cmdvel_to_encoder_node')
        # Parameters for topic names
        self.declare_parameter('teleop_topic', '/bicycle_steering_controller/reference')
        teleop_topic = self.get_parameter('teleop_topic').value

        # Subscriber: listens to /bicycle_steering_controller/reference
        self.subscription = self.create_subscription(
            Twist,
            teleop_topic,
            self.cmd_vel_callback,
            10
        )

        # Publisher: publishes to /encoder_values
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/encoder_values',
            10
        )

        # Internal state for encoder positions
        self.steer_position = 0.0
        self.throttle_position = 0.0

        # Last time update
        self.last_time = time.time()

        self.get_logger().info(f"CmdVel → Encoder Position node started. Listening to {teleop_topic}")

        # self.get_logger().info("CmdVel → Encoder node started.")

    def cmd_vel_callback(self, msg: Twist):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # Simulated integration: position += velocity × dt
        steer = msg.angular.z
        throttle_velocity = msg.linear.x

        self.steer_position = steer
        self.throttle_position += throttle_velocity * dt

        # Publish simulated encoder positions
        encoder_msg = Float32MultiArray()
        encoder_msg.data = [self.steer_position, self.throttle_position]

        self.publisher.publish(encoder_msg)

        self.get_logger().info(
            f"Updated encoder positions: steer={self.steer_position:.4f}, throttle={self.throttle_position:.4f} (dt={dt:.3f}s)"
        )
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToEncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
