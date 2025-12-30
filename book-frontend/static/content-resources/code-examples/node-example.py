#!/usr/bin/env python3
"""
Basic ROS 2 Node Example

This example demonstrates the fundamental structure of a ROS 2 node.
It creates a simple publisher that sends messages at regular intervals.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    A minimal ROS 2 publisher node.

    This node demonstrates the basic structure of a ROS 2 node with:
    - Node initialization
    - Publisher creation
    - Timer-based publishing
    - Message creation and publishing
    """

    def __init__(self):
        """Initialize the node with a name and set up publisher and timer."""
        super().__init__('minimal_publisher')

        # Create a publisher that will publish String messages on the 'topic' topic
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        # Set up a timer to call the timer_callback method every 0.5 seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter for messages
        self.i = 0

    def timer_callback(self):
        """Callback function that runs every timer tick."""
        # Create a String message
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # Publish the message
        self.publisher_.publish(msg)

        # Log the published message
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment the counter
        self.i += 1


def main(args=None):
    """
    Main function to run the ROS 2 node.

    This follows the standard ROS 2 Python node pattern:
    1. Initialize ROS communication
    2. Create node instance
    3. Spin the node (process callbacks)
    4. Clean up when done
    """
    # Initialize ROS communications
    rclpy.init(args=args)

    # Create the node instance
    minimal_publisher = MinimalPublisher()

    # Use rclpy.spin() to process callbacks until the program is shut down
    rclpy.spin(minimal_publisher)

    # Clean up: destroy the node and shut down ROS communications
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()