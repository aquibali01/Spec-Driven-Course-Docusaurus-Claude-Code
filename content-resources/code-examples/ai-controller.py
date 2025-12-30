#!/usr/bin/env python3
"""
AI Controller Interface Example

This example demonstrates how to create an AI controller that interfaces
with ROS 2 using rclpy. It shows the integration of AI algorithms with
robot control systems.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np
import time


class AIControllerInterface(Node):
    """
    AI Controller Interface for Robot Control

    This node demonstrates how to integrate AI algorithms with ROS 2.
    It subscribes to sensor data, processes it through an AI algorithm,
    and publishes control commands to the robot.
    """

    def __init__(self):
        super().__init__('ai_controller_interface')

        # Internal state for the AI controller
        self.current_sensor_data = {}
        self.robot_state = {}
        self.ai_model_state = {}

        # Setup ROS 2 components
        self.setup_subscribers()
        self.setup_publishers()
        self.setup_timers()

        # Initialize AI components
        self.initialize_ai_components()

        self.get_logger().info('AI Controller Interface initialized and ready')

    def setup_subscribers(self):
        """
        Set up subscribers for sensor data and robot state.
        """
        # Quality of Service profile for sensor data
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            sensor_qos
        )

        self.get_logger().info('Subscribers configured')

    def setup_publishers(self):
        """
        Set up publishers for control commands.
        """
        # Publisher for joint position commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.get_logger().info('Publishers configured')

    def setup_timers(self):
        """
        Set up timers for periodic AI processing.
        """
        # Timer for AI decision making (10 Hz)
        self.ai_timer = self.create_timer(
            0.1,  # 0.1 seconds = 10 Hz
            self.ai_decision_loop
        )

        self.get_logger().info('Timers configured')

    def initialize_ai_components(self):
        """
        Initialize AI model and related components.
        """
        # Initialize any AI state
        self.ai_model_state = {
            'last_action': None,
            'current_policy': 'default',
            'learning_enabled': False
        }

        self.get_logger().info('AI components initialized')

    def joint_state_callback(self, msg):
        """
        Callback for joint state messages.

        Args:
            msg: JointState message containing joint positions, velocities, etc.
        """
        # Update internal state with joint information
        self.current_sensor_data['joint_positions'] = list(msg.position)
        self.current_sensor_data['joint_velocities'] = list(msg.velocity)
        self.current_sensor_data['joint_efforts'] = list(msg.effort)

        # Update robot state
        self.robot_state['timestamp'] = self.get_clock().now()
        self.robot_state['joint_count'] = len(msg.name)
        self.robot_state['joint_names'] = list(msg.name)

        # Log for debugging
        self.get_logger().debug(f'Joint positions: {self.current_sensor_data["joint_positions"]}')

    def ai_decision_loop(self):
        """
        Main AI decision making loop.

        This method is called periodically by the timer and contains
        the core AI logic for processing sensor data and making decisions.
        """
        # Check if we have enough sensor data to make a decision
        if not self.current_sensor_data:
            self.get_logger().debug('Waiting for sensor data...')
            return

        # Process sensor data through AI algorithm
        try:
            action = self.process_with_ai_model()

            # Execute the action by publishing commands
            self.execute_action(action)

        except Exception as e:
            self.get_logger().error(f'Error in AI decision loop: {e}')

    def process_with_ai_model(self):
        """
        Process sensor data through AI model to generate actions.

        Returns:
            action: The action to be executed by the robot
        """
        # Extract relevant sensor data
        joint_positions = self.current_sensor_data.get('joint_positions', [])
        joint_velocities = self.current_sensor_data.get('joint_velocities', [])

        # Prepare input for AI model
        sensor_input = np.concatenate([
            np.array(joint_positions),
            np.array(joint_velocities)
        ])

        # Apply AI model (simplified example)
        # In practice, this would be your actual AI algorithm
        action = self.simple_balance_policy(sensor_input)

        return action

    def simple_balance_policy(self, sensor_input):
        """
        Simple balance policy as an example AI algorithm.

        Args:
            sensor_input: Array of sensor data

        Returns:
            action: Joint commands to maintain balance
        """
        # This is a simplified example of an AI policy
        # In practice, this would be your trained AI model

        # Simple proportional control for balance
        # (This is just an example - real balance control is much more complex)
        balance_commands = []
        for i, position in enumerate(sensor_input[:len(sensor_input)//2]):
            # Simple proportional control: command opposite to deviation
            command = -0.1 * position
            balance_commands.append(command)

        return balance_commands

    def execute_action(self, action):
        """
        Execute the action generated by the AI model.

        Args:
            action: The action to execute (e.g., joint commands)
        """
        if not action:
            return

        # Create and publish joint commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = action

        self.joint_cmd_pub.publish(cmd_msg)

        # Log the action
        self.get_logger().info(f'Executed action: {action[:5]}...')  # Show first 5 values


class AdvancedAIController(AIControllerInterface):
    """
    Advanced AI Controller with additional features.

    This extends the basic AI controller with more sophisticated features
    like learning capabilities, multiple control modes, and safety checks.
    """

    def __init__(self):
        super().__init__()

        # Additional state for advanced features
        self.learning_buffer = []
        self.safety_limits = {
            'max_velocity': 1.0,
            'max_effort': 10.0,
            'max_position_error': 0.5
        }

        # Setup additional components
        self.setup_advanced_features()

    def setup_advanced_features(self):
        """
        Setup advanced features like learning and safety systems.
        """
        # Initialize learning components
        self.learning_enabled = False
        self.exploration_rate = 0.1

        self.get_logger().info('Advanced features configured')

    def process_with_ai_model(self):
        """
        Enhanced AI processing with learning and safety features.
        """
        # Check if we have enough sensor data
        if not self.current_sensor_data:
            return [0.0] * 5  # Default safe action

        # Extract sensor data
        joint_positions = self.current_sensor_data.get('joint_positions', [])
        joint_velocities = self.current_sensor_data.get('joint_velocities', [])

        # Prepare input for AI model
        sensor_input = np.concatenate([
            np.array(joint_positions),
            np.array(joint_velocities)
        ])

        # Apply safety checks
        if self.is_unsafe_state(sensor_input):
            return self.get_safe_recovery_action()

        # Apply AI model
        action = self.advanced_policy(sensor_input)

        # Apply safety limits
        action = self.apply_safety_limits(action)

        # Store for learning if enabled
        if self.learning_enabled:
            self.store_experience(sensor_input, action)

        return action

    def is_unsafe_state(self, sensor_input):
        """
        Check if the current state is unsafe.

        Args:
            sensor_input: Current sensor state

        Returns:
            bool: True if state is unsafe, False otherwise
        """
        # Example safety check: if joint positions are too extreme
        joint_positions = sensor_input[:len(sensor_input)//2]
        for pos in joint_positions:
            if abs(pos) > self.safety_limits['max_position_error']:
                return True
        return False

    def get_safe_recovery_action(self):
        """
        Get a safe recovery action when in an unsafe state.

        Returns:
            list: Safe recovery action
        """
        # Return zero commands to stop all movement
        return [0.0] * 5

    def advanced_policy(self, sensor_input):
        """
        Advanced AI policy with learning capabilities.

        Args:
            sensor_input: Array of sensor data

        Returns:
            action: Joint commands to execute
        """
        # Example: simple balance policy with some exploration
        balance_commands = []
        for i, position in enumerate(sensor_input[:len(sensor_input)//2]):
            # Base command from balance policy
            base_command = -0.1 * position

            # Add some exploration if learning is enabled
            exploration = 0.0
            if self.learning_enabled and np.random.random() < self.exploration_rate:
                exploration = np.random.normal(0, 0.05)

            command = base_command + exploration
            balance_commands.append(command)

        return balance_commands

    def apply_safety_limits(self, action):
        """
        Apply safety limits to the action.

        Args:
            action: Raw action from AI model

        Returns:
            action: Action with safety limits applied
        """
        # Limit maximum values
        limited_action = []
        for cmd in action:
            limited_cmd = max(
                -self.safety_limits['max_velocity'],
                min(self.safety_limits['max_velocity'], cmd)
            )
            limited_action.append(limited_cmd)

        return limited_action

    def store_experience(self, state, action):
        """
        Store experience for learning.

        Args:
            state: Current state
            action: Action taken
        """
        # Store in learning buffer (in practice, this would be more sophisticated)
        experience = {
            'state': state,
            'action': action,
            'timestamp': time.time()
        }
        self.learning_buffer.append(experience)

        # Keep buffer size reasonable
        if len(self.learning_buffer) > 1000:
            self.learning_buffer.pop(0)


def main(args=None):
    """
    Main function to run the AI controller node.

    This follows the standard ROS 2 Python node pattern:
    1. Initialize ROS communication
    2. Create node instance
    3. Spin the node (process callbacks)
    4. Clean up when done
    """
    # Initialize ROS communications
    rclpy.init(args=args)

    # Create the AI controller node (use AdvancedAIController for more features)
    ai_controller = AdvancedAIController()

    try:
        # Use rclpy.spin() to process callbacks until the program is shut down
        rclpy.spin(ai_controller)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        ai_controller.get_logger().info('AI Controller interrupted by user')
    finally:
        # Clean up: destroy the node and shut down ROS communications
        ai_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()