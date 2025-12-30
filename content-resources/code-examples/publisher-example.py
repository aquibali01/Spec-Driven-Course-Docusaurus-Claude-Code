#!/usr/bin/env python3
"""
Publisher Example for ROS 2

This example demonstrates how to create a publisher that sends messages
on a topic. This is the "publish" part of the publish/subscribe pattern.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PublisherExample(Node):
    """
    Publisher Example Node

    This node demonstrates the creation and usage of a publisher in ROS 2.
    It periodically publishes messages to a topic that subscribers can receive.
    """

    def __init__(self):
        """Initialize the publisher node."""
        super().__init__('publisher_example')

        # Create a publisher that will publish String messages on the 'chatter' topic
        # with a queue size of 10
        self.publisher = self.create_publisher(String, 'chatter', 10)

        # Create a timer that will call the timer_callback method every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter for the messages
        self.counter = 0

        # Log that the publisher has started
        self.get_logger().info('Publisher node started, publishing to /chatter')

    def timer_callback(self):
        """
        Timer callback function.

        This function is called every timer tick (every 0.5 seconds in this example).
        It creates a message and publishes it to the topic.
        """
        # Create a new String message
        msg = String()

        # Set the message data
        msg.data = f'Hello ROS 2 World: {self.counter}'

        # Publish the message to the topic
        self.publisher.publish(msg)

        # Log the published message (for demonstration purposes)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter for the next message
        self.counter += 1


def main(args=None):
    """
    Main function to run the publisher node.

    This follows the standard ROS 2 Python node pattern:
    1. Initialize ROS communication
    2. Create node instance
    3. Spin the node (process callbacks)
    4. Clean up when done
    """
    # Initialize ROS communications
    rclpy.init(args=args)

    # Create the publisher node instance
    publisher_example = PublisherExample()

    try:
        # Use rclpy.spin() to process callbacks until the program is shut down
        # This will keep the node running and calling the timer callback
        rclpy.spin(publisher_example)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        publisher_example.get_logger().info('Interrupted by user')
    finally:
        # Clean up: destroy the node and shut down ROS communications
        publisher_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()