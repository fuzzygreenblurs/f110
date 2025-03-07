#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('relay')
        self.subscription = self.create_subscription(
                AckermannDriveStamped,
                'drive',
                self.listener_cb,
                10)

        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive_relay', 10)

    def listener_cb(self, msg):
        msg.drive.speed = msg.drive.speed * 3
        msg.drive.steering_angle = msg.drive.steering_angle * 3
        
        self.get_logger().info(f"speed: {msg.drive.speed}, steering_angle: {msg.drive.steering_angle}, relay: {msg}")    
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
