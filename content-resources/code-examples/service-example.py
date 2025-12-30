#!/usr/bin/env python3
"""
Service Example for ROS 2

This example demonstrates how to create both a service server and a service client.
Services provide synchronous request/response communication in ROS 2.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class ServiceServerExample(Node):
    """
    Service Server Example Node

    This node demonstrates how to create a service server that responds to requests.
    When a client sends a request, this server processes it and returns a response.
    """

    def __init__(self):
        """Initialize the service server node."""
        super().__init__('service_server_example')

        # Create a service that will use the AddTwoInts interface
        # The service name is 'add_two_ints'
        # The callback function is add_two_ints_callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        # Log that the service server has started
        self.get_logger().info('Service server started, listening to /add_two_ints')

    def add_two_ints_callback(self, request, response):
        """
        Callback function for processing service requests.

        This function is called when a client sends a request to the service.

        Args:
            request: The request message containing the data from the client
            response: The response message that will be sent back to the client

        Returns:
            response: The filled response message
        """
        # Perform the operation (in this case, adding two integers)
        response.sum = request.a + request.b

        # Log the request and response
        self.get_logger().info(f'Incoming request: {request.a} + {request.b} = {response.sum}')

        # Return the response
        return response


class ServiceClientExample(Node):
    """
    Service Client Example Node

    This node demonstrates how to create a service client that sends requests
    to a service server and waits for the response.
    """

    def __init__(self):
        """Initialize the service client node."""
        super().__init__('service_client_example')

        # Create a client for the AddTwoInts service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for the service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        """
        Send a request to the service server.

        Args:
            a (int): First integer to add
            b (int): Second integer to add

        Returns:
            Future: A future object that will contain the response
        """
        # Set the request data
        self.request.a = a
        self.request.b = b

        # Send the request asynchronously
        self.future = self.cli.call_async(self.request)

        # Log the request
        self.get_logger().info(f'Sent request: {a} + {b}')

        # Return the future to allow the caller to wait for the response
        return self.future


def run_service_server():
    """
    Function to run the service server.

    This function initializes ROS, creates the service server node,
    and spins it to handle incoming requests.
    """
    # Initialize ROS communications
    rclpy.init(args=None)

    # Create the service server node
    service_server = ServiceServerExample()

    try:
        # Spin the node to handle service requests
        rclpy.spin(service_server)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        service_server.get_logger().info('Service server interrupted by user')
    finally:
        # Clean up: destroy the node and shut down ROS communications
        service_server.destroy_node()
        rclpy.shutdown()


def run_service_client():
    """
    Function to run the service client.

    This function initializes ROS, creates the service client node,
    sends a request, and waits for the response.
    """
    # Initialize ROS communications
    rclpy.init(args=None)

    # Create the service client node
    service_client = ServiceClientExample()

    # Send a request to add 2 + 3
    future = service_client.send_request(2, 3)

    try:
        # Wait for the response
        rclpy.spin_until_future_complete(service_client, future)

        # Get the response
        response = future.result()

        # Log the result
        service_client.get_logger().info(f'Result of 2 + 3: {response.sum}')

    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        service_client.get_logger().info('Service client interrupted by user')
    finally:
        # Clean up: destroy the node and shut down ROS communications
        service_client.destroy_node()
        rclpy.shutdown()


def main_server(args=None):
    """
    Main function to run the service server.

    This function is intended to be run in one terminal to start the server.
    """
    run_service_server()


def main_client(args=None):
    """
    Main function to run the service client.

    This function is intended to be run in another terminal to start the client.
    """
    run_service_client()


if __name__ == '__main__':
    # To run the server: python3 service-example.py server
    # To run the client: python3 service-example.py client
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == 'server':
            main_server()
        elif sys.argv[1] == 'client':
            main_client()
        else:
            print("Usage: python3 service-example.py [server|client]")
    else:
        print("Usage: python3 service-example.py [server|client]")