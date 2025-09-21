#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from my_chatter_msgs.msg import TimestampString


class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(TimestampString, '/user_messages', 10)
        self.get_logger().info('Publisher node has been started')
        
    def publish_message(self):
        while rclpy.ok():
            try:
                # Prompt user for input
                user_input = input('Please enter a line of text and press <Enter>: ')
                
                # Create message
                msg = TimestampString()
                msg.msg = user_input
                
                # Get current time in nanoseconds
                current_time = self.get_clock().now()
                msg.timestamp = current_time.nanoseconds
                
                # Publish the message
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published: "{msg.msg}" at timestamp: {msg.timestamp}')
                
            except KeyboardInterrupt:
                self.get_logger().info('Publisher node interrupted by user')
                break
            except Exception as e:
                self.get_logger().error(f'Error occurred: {str(e)}')
                break


def main(args=None):
    rclpy.init(args=args)
    
    publisher_node = PublisherNode()
    
    try:
        # Start publishing messages
        publisher_node.publish_message()
    except KeyboardInterrupt:
        pass
    finally:
        publisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()