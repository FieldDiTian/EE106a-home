#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_chatter_msgs.msg import TimestampString


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            TimestampString,
            '/user_messages',
            self.message_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Subscriber node has been started')
        self.get_logger().info('Waiting for messages on /user_messages topic...')

    def message_callback(self, msg):
        # Get current time when message is received
        received_time = self.get_clock().now()
        received_timestamp = received_time.nanoseconds
        
        # Print the message in the specified format
        print(f'Message: {msg.msg}, Sent at: {msg.timestamp}, Received at: {received_timestamp}')
        
        # Also log to ROS logger
        self.get_logger().info(f'Received message: "{msg.msg}"')


def main(args=None):
    rclpy.init(args=args)
    
    subscriber_node = SubscriberNode()
    
    try:
        # Keep the node running until interrupted
        rclpy.spin(subscriber_node)
    except KeyboardInterrupt:
        subscriber_node.get_logger().info('Subscriber node interrupted by user')
    finally:
        subscriber_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()