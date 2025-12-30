#!/usr/bin/env python3
"""
Action Example for ROS 2

This example demonstrates how to create both an action server and an action client.
Actions provide goal-based communication with feedback and result in ROS 2.
"""

import time
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from example_interfaces.action import Fibonacci


class ActionServerExample(Node):
    """
    Action Server Example Node

    This node demonstrates how to create an action server that processes goals
    and provides feedback during execution. Actions are ideal for long-running tasks.
    """

    def __init__(self):
        """Initialize the action server node."""
        super().__init__('action_server_example')

        # Create an action server
        # - Action type: Fibonacci
        # - Action name: 'fibonacci'
        # - Callback function: execute_callback
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        # Log that the action server has started
        self.get_logger().info('Action server started, listening to /fibonacci')

    def execute_callback(self, goal_handle):
        """
        Callback function for processing action goals.

        This function is called when a client sends a goal to the action server.
        It processes the goal and provides feedback during execution.

        Args:
            goal_handle: The goal handle that contains the goal request

        Returns:
            Fibonacci.Result: The result of the action
        """
        self.get_logger().info(f'Executing goal: {goal_handle.request.order}')

        # Initialize the result sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Process the goal request
        for i in range(1, goal_handle.request.order):
            # Check if the goal has been canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                return result

            # Update the sequence
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i - 1])

            # Publish feedback to the client
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            # Simulate processing time (in a real application, this would be actual work)
            time.sleep(1)

        # Complete the goal successfully
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info(f'Result: {result.sequence}')
        return result


class ActionClientExample(Node):
    """
    Action Client Example Node

    This node demonstrates how to create an action client that sends goals
    to an action server and receives feedback and results.
    """

    def __init__(self):
        """Initialize the action client node."""
        super().__init__('action_client_example')

        # Create an action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

    def send_goal(self, order):
        """
        Send a goal to the action server.

        Args:
            order (int): The order of the Fibonacci sequence to calculate

        Returns:
            Future: A future object that will contain the result
        """
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create a goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        # Send the goal asynchronously
        self.get_logger().info(f'Sending goal: {order}')
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        # Set a callback for when the goal is accepted
        send_goal_future.add_done_callback(self.goal_response_callback)

        return send_goal_future

    def goal_response_callback(self, future):
        """
        Callback function for when the goal response is received.

        Args:
            future: The future containing the goal response
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Request the result
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Callback function for processing feedback from the action server.

        Args:
            feedback_msg: The feedback message from the action server
        """
        self.get_logger().info(
            f'Received feedback: {feedback_msg.feedback.sequence}')

    def get_result_callback(self, future):
        """
        Callback function for processing the result from the action server.

        Args:
            future: The future containing the result
        """
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')


def run_action_server():
    """
    Function to run the action server.

    This function initializes ROS, creates the action server node,
    and spins it to handle incoming goals.
    """
    # Initialize ROS communications
    rclpy.init(args=None)

    # Create the action server node
    action_server = ActionServerExample()

    try:
        # Spin the node to handle action goals
        rclpy.spin(action_server)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        action_server.get_logger().info('Action server interrupted by user')
    finally:
        # Clean up: destroy the node and shut down ROS communications
        action_server.destroy_node()
        rclpy.shutdown()


def run_action_client():
    """
    Function to run the action client.

    This function initializes ROS, creates the action client node,
    sends a goal, and waits for the result.
    """
    # Initialize ROS communications
    rclpy.init(args=None)

    # Create the action client node
    action_client = ActionClientExample()

    # Send a goal to calculate Fibonacci sequence of order 10
    future = action_client.send_goal(10)

    try:
        # Spin to process callbacks until the result is received
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        action_client.get_logger().info('Action client interrupted by user')
    finally:
        # Clean up: destroy the node and shut down ROS communications
        action_client.destroy_node()
        rclpy.shutdown()


def main_server(args=None):
    """
    Main function to run the action server.

    This function is intended to be run in one terminal to start the server.
    """
    run_action_server()


def main_client(args=None):
    """
    Main function to run the action client.

    This function is intended to be run in another terminal to start the client.
    """
    run_action_client()


if __name__ == '__main__':
    # To run the server: python3 action-example.py server
    # To run the client: python3 action-example.py client
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == 'server':
            main_server()
        elif sys.argv[1] == 'client':
            main_client()
        else:
            print("Usage: python3 action-example.py [server|client]")
    else:
        print("Usage: python3 action-example.py [server|client]")