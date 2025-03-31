#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import datetime
import os


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)

        # Create log file with timestamp in name
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file_path = f"subscription_log_{timestamp}.txt"
        self.get_logger().info(f'Logs will be saved to: {self.log_file_path}')

        # Create the file with headers
        with open(self.log_file_path, 'w') as f:
            f.write(f"Subscription Log - Started: {datetime.datetime.now()}\n")
            f.write("-" * 50 + "\n")

    def listener_callback(self, msg):
        current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        log_message = f"[{current_time}] Received: '{msg.data}'"

        # Log to console
        self.get_logger().info(log_message)

        # Log to file
        with open(self.log_file_path, 'a') as f:
            f.write(log_message + "\n")


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
