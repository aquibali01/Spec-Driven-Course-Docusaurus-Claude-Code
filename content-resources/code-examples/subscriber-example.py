#!/usr/bin/env python3
"""
Subscriber Example for ROS 2

This example demonstrates how to create a subscriber that receives messages
from a topic. This is the "subscribe" part of the publish/subscribe pattern.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubscriberExample(Node):
    """
    Subscriber Example Node

    This node demonstrates the creation and usage of a subscriber in ROS 2.
    It listens to messages published on a topic and processes them in a callback.
    """

    def __init__(self):
        """Initialize the subscriber node."""
        super().__init__('subscriber_example')

        # Create a subscription to the 'chatter' topic
        # When messages are received, the listener_callback method will be called
        self.subscription = self.create_subscription(
            String,
            'chatter',  # Topic name
            self.listener_callback,  # Callback function
            10  # Queue size
        )

        # Make sure the subscription is created properly
        self.subscription  # prevent unused variable warning

        # Log that the subscriber has started
        self.get_logger().info('Subscriber node started, listening to /chatter')

    def listener_callback(self, msg):
        """
        Callback function for processing received messages.

        This function is called whenever a new message is received on the subscribed topic.

        Args:
            msg: The received message (String type in this case)
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')

        # Here you could process the message further
        # For example, parse the data, update internal state, etc.
        self.process_message(msg.data)

    def process_message(self, message_data):
        """
        Process the received message data.

        This is where you would implement any logic for processing
        the received message data.

        Args:
            message_data (str): The string data from the received message
        """
        # Example processing: check if message contains certain keywords
        if 'Hello' in message_data:
            self.get_logger().info('Received a greeting message!')

        # Example processing: extract numerical data if present
        try:
            # Look for numbers in the message
            import re
            numbers = re.findall(r'\d+', message_data)
            if numbers:
                self.get_logger().info(f'Found numbers in message: {numbers}')
        except Exception as e:
            self.get_logger().warn(f'Error processing message: {e}')


def main(args=None):
    """
    Main function to run the subscriber node.

    This follows the standard ROS 2 Python node pattern:
    1. Initialize ROS communication
    2. Create node instance
    3. Spin the node (process callbacks)
    4. Clean up when done
    """
    # Initialize ROS communications
    rclpy.init(args=args)

    # Create the subscriber node instance
    subscriber_example = SubscriberExample()

    try:
        # Use rclpy.spin() to process callbacks until the program is shut down
        # This will keep the node running and calling the listener callback
        # whenever new messages arrive on the 'chatter' topic
        rclpy.spin(subscriber_example)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        subscriber_example.get_logger().info('Interrupted by user')
    finally:
        # Clean up: destroy the node and shut down ROS communications
        subscriber_example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()