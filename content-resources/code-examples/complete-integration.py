#!/usr/bin/env python3
"""
Complete AI-ROS2 Integration Example

This example demonstrates a complete integration of an AI agent with ROS 2,
showing how to connect all the concepts from the previous examples into a
cohesive system for robot control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState, Image, LaserScan, Imu
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger, SetBool
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient
import numpy as np
import time
import threading
from concurrent.futures import ThreadPoolExecutor
import queue


class CompleteAIIntegration(Node):
    """
    Complete AI-ROS2 Integration Node

    This node demonstrates a complete integration of AI algorithms with ROS 2,
    incorporating all the communication patterns and integration techniques
    discussed in the module.
    """

    def __init__(self):
        super().__init__('complete_ai_integration')

        # Internal state
        self.current_sensor_data = {}
        self.robot_state = {}
        self.ai_model_state = {}
        self.ai_decision_history = []

        # Thread pool for AI processing
        self.ai_executor = ThreadPoolExecutor(max_workers=2)

        # Queues for thread communication
        self.sensor_queue = queue.Queue(maxsize=20)
        self.command_queue = queue.Queue(maxsize=20)
        self.decision_queue = queue.Queue(maxsize=10)

        # Setup all ROS components
        self.setup_parameters()
        self.setup_subscribers()
        self.setup_publishers()
        self.setup_services()
        self.setup_actions()
        self.setup_timers()
        self.setup_ai_components()

        # Start background threads
        self.start_background_threads()

        self.get_logger().info('Complete AI Integration Node initialized')

    def setup_parameters(self):
        """
        Setup ROS parameters for the node.
        """
        # Declare parameters with default values
        self.declare_parameter('update_rate', 10)
        self.declare_parameter('ai_model_path', '')
        self.declare_parameter('safety_threshold', 0.5)
        self.declare_parameter('exploration_rate', 0.1)

        # Get parameter values
        self.update_rate = self.get_parameter('update_rate').value
        self.ai_model_path = self.get_parameter('ai_model_path').value
        self.safety_threshold = self.get_parameter('safety_threshold').value
        self.exploration_rate = self.get_parameter('exploration_rate').value

    def setup_subscribers(self):
        """
        Setup all subscribers for sensor data.
        """
        # Quality of Service profiles for different sensor types
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        high_freq_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        critical_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Joint states (critical for control)
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            critical_qos
        )

        # IMU data (critical for balance)
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            critical_qos
        )

        # Laser scan (for navigation)
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            sensor_qos
        )

        # Odometry (for localization)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            sensor_qos
        )

        # Camera (high frequency)
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            high_freq_qos
        )

        self.get_logger().info('All subscribers configured')

    def setup_publishers(self):
        """
        Setup all publishers for commands.
        """
        # Joint command publisher
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',
            10
        )

        # Velocity command publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Status publisher
        self.status_pub = self.create_publisher(
            String,
            '/ai_status',
            10
        )

        self.get_logger().info('All publishers configured')

    def setup_services(self):
        """
        Setup services for AI functionality.
        """
        # Service to trigger AI planning
        self.plan_service = self.create_service(
            Trigger,
            'plan_action',
            self.plan_service_callback
        )

        # Service to enable/disable learning
        self.learning_service = self.create_service(
            SetBool,
            'set_learning_mode',
            self.learning_service_callback
        )

        self.get_logger().info('Services configured')

    def setup_actions(self):
        """
        Setup action clients for complex tasks.
        """
        # Action client for navigation
        self.nav_action_client = ActionClient(
            self,
            Fibonacci,  # Using Fibonacci as an example
            'navigate_to_pose'
        )

        self.get_logger().info('Actions configured')

    def setup_timers(self):
        """
        Setup timers for periodic tasks.
        """
        # AI decision timer (main control loop)
        self.ai_timer = self.create_timer(
            1.0 / self.update_rate,  # Based on update rate parameter
            self.ai_decision_timer_callback
        )

        # Status publishing timer
        self.status_timer = self.create_timer(
            1.0,  # Every second
            self.publish_status
        )

        self.get_logger().info('Timers configured')

    def setup_ai_components(self):
        """
        Initialize AI-specific components.
        """
        # Initialize AI model state
        self.ai_model_state = {
            'initialized': True,
            'learning_enabled': False,
            'current_policy': 'default',
            'episode_count': 0,
            'total_reward': 0.0,
            'last_decision_time': time.time()
        }

        # Initialize decision history
        self.ai_decision_history = []

        self.get_logger().info('AI components initialized')

    def start_background_threads(self):
        """
        Start background threads for AI processing.
        """
        # Thread for AI processing
        self.ai_thread = threading.Thread(target=self.ai_processing_thread)
        self.ai_thread.daemon = True
        self.ai_thread.start()

        # Thread for command execution
        self.command_thread = threading.Thread(target=self.command_execution_thread)
        self.command_thread.daemon = True
        self.command_thread.start()

        self.get_logger().info('Background threads started')

    def joint_state_callback(self, msg):
        """
        Callback for joint state messages.
        """
        try:
            self.current_sensor_data['joint_states'] = {
                'position': list(msg.position),
                'velocity': list(msg.velocity),
                'effort': list(msg.effort),
                'names': list(msg.name),
                'timestamp': self.get_clock().now()
            }

            # Put sensor data in queue for AI processing
            sensor_data = {
                'type': 'joint_states',
                'data': self.current_sensor_data['joint_states'],
                'timestamp': time.time()
            }
            self.sensor_queue.put_nowait(sensor_data)

        except queue.Full:
            self.get_logger().warn('Sensor queue full, dropping joint state message')

    def imu_callback(self, msg):
        """
        Callback for IMU data.
        """
        try:
            self.current_sensor_data['imu'] = {
                'orientation': [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w],
                'angular_velocity': [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                'linear_acceleration': [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z],
                'timestamp': self.get_clock().now()
            }

            # Put sensor data in queue for AI processing
            sensor_data = {
                'type': 'imu',
                'data': self.current_sensor_data['imu'],
                'timestamp': time.time()
            }
            self.sensor_queue.put_nowait(sensor_data)

        except queue.Full:
            self.get_logger().warn('Sensor queue full, dropping IMU message')

    def laser_callback(self, msg):
        """
        Callback for laser scan data.
        """
        try:
            self.current_sensor_data['laser_scan'] = {
                'ranges': list(msg.ranges),
                'intensities': list(msg.intensities),
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment,
                'time_increment': msg.time_increment,
                'scan_time': msg.scan_time,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'timestamp': self.get_clock().now()
            }

            # Put sensor data in queue for AI processing
            sensor_data = {
                'type': 'laser_scan',
                'data': self.current_sensor_data['laser_scan'],
                'timestamp': time.time()
            }
            self.sensor_queue.put_nowait(sensor_data)

        except queue.Full:
            self.get_logger().warn('Sensor queue full, dropping laser scan message')

    def odom_callback(self, msg):
        """
        Callback for odometry data.
        """
        try:
            self.current_sensor_data['odometry'] = {
                'pose': {
                    'position': [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
                    'orientation': [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
                },
                'twist': {
                    'linear': [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
                    'angular': [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
                },
                'timestamp': self.get_clock().now()
            }

            # Put sensor data in queue for AI processing
            sensor_data = {
                'type': 'odometry',
                'data': self.current_sensor_data['odometry'],
                'timestamp': time.time()
            }
            self.sensor_queue.put_nowait(sensor_data)

        except queue.Full:
            self.get_logger().warn('Sensor queue full, dropping odometry message')

    def camera_callback(self, msg):
        """
        Callback for camera data.
        """
        try:
            # For simplicity, we're not processing the full image data here
            # In a real application, you would process the image data
            self.current_sensor_data['camera'] = {
                'height': msg.height,
                'width': msg.width,
                'encoding': msg.encoding,
                'is_bigendian': msg.is_bigendian,
                'step': msg.step,
                'data_size': len(msg.data),
                'timestamp': self.get_clock().now()
            }

            # Put sensor data in queue for AI processing
            sensor_data = {
                'type': 'camera',
                'data': self.current_sensor_data['camera'],
                'timestamp': time.time()
            }
            self.sensor_queue.put_nowait(sensor_data)

        except queue.Full:
            self.get_logger().warn('Sensor queue full, dropping camera message')

    def ai_decision_timer_callback(self):
        """
        Timer callback for AI decision making.
        """
        # This is a lightweight callback that triggers AI processing
        # The heavy processing happens in the background thread
        pass

    def ai_processing_thread(self):
        """
        Background thread for AI processing.
        """
        while rclpy.ok():
            try:
                # Get sensor data from queue
                sensor_data = self.sensor_queue.get(timeout=1.0)

                # Process with AI model
                decision = self.process_sensor_data_with_ai(sensor_data)

                # Put decision in decision queue
                if decision is not None:
                    self.decision_queue.put_nowait(decision)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'AI processing error: {e}')

    def process_sensor_data_with_ai(self, sensor_data):
        """
        Process sensor data with AI model.

        Args:
            sensor_data: Sensor data from the queue

        Returns:
            decision: AI decision or action to execute
        """
        try:
            # Update internal state based on sensor type
            sensor_type = sensor_data['type']
            sensor_values = sensor_data['data']

            # Process based on sensor type
            if sensor_type == 'joint_states':
                return self.process_joint_states(sensor_values)
            elif sensor_type == 'imu':
                return self.process_imu_data(sensor_values)
            elif sensor_type == 'laser_scan':
                return self.process_laser_scan(sensor_values)
            elif sensor_type == 'odometry':
                return self.process_odometry(sensor_values)
            elif sensor_type == 'camera':
                return self.process_camera_data(sensor_values)
            else:
                self.get_logger().warn(f'Unknown sensor type: {sensor_type}')
                return None

        except Exception as e:
            self.get_logger().error(f'Error processing sensor data: {e}')
            return None

    def process_joint_states(self, joint_data):
        """
        Process joint state data with AI model.

        Args:
            joint_data: Joint state information

        Returns:
            dict: Decision based on joint data
        """
        # Example: Simple balance policy based on joint positions
        positions = np.array(joint_data['position'])
        velocities = np.array(joint_data['velocity'])

        # Simple proportional control for balance
        commands = -0.1 * positions - 0.05 * velocities

        return {
            'type': 'joint_command',
            'data': commands.tolist(),
            'timestamp': time.time(),
            'source': 'joint_states'
        }

    def process_imu_data(self, imu_data):
        """
        Process IMU data with AI model.

        Args:
            imu_data: IMU sensor information

        Returns:
            dict: Decision based on IMU data
        """
        # Example: Balance control based on orientation
        orientation = np.array(imu_data['orientation'])
        angular_velocity = np.array(imu_data['angular_velocity'])

        # Simple balance policy based on orientation error
        # (This is a simplified example)
        balance_command = -0.1 * orientation[:3] - 0.05 * angular_velocity  # Use only x,y,z parts

        return {
            'type': 'balance_command',
            'data': balance_command.tolist(),
            'timestamp': time.time(),
            'source': 'imu'
        }

    def process_laser_scan(self, laser_data):
        """
        Process laser scan data with AI model.

        Args:
            laser_data: Laser scan information

        Returns:
            dict: Decision based on laser data
        """
        # Example: Obstacle avoidance based on laser data
        ranges = np.array(laser_data['ranges'])

        # Filter out invalid ranges
        valid_ranges = ranges[np.isfinite(ranges)]

        if len(valid_ranges) == 0:
            return None

        # Find minimum distance
        min_distance = np.min(valid_ranges)

        # If obstacle is too close, generate avoidance command
        if min_distance < 1.0:  # 1 meter threshold
            # Simple avoidance: turn away from obstacles
            avoidance_command = {
                'linear': 0.0,  # Stop forward motion
                'angular': 0.5  # Turn
            }

            return {
                'type': 'avoidance_command',
                'data': avoidance_command,
                'timestamp': time.time(),
                'source': 'laser_scan'
            }

        return None

    def process_odometry(self, odom_data):
        """
        Process odometry data with AI model.

        Args:
            odom_data: Odometry information

        Returns:
            dict: Decision based on odometry data
        """
        # Example: Navigation decision based on position
        position = np.array(odom_data['pose']['position'])

        # Simple navigation policy (example: move toward origin)
        target = np.array([0.0, 0.0, 0.0])  # Move toward origin
        direction = target - position
        distance = np.linalg.norm(direction)

        if distance > 0.1:  # If not close to target
            direction_normalized = direction / distance
            navigation_command = {
                'linear': min(0.5, distance),  # Scale speed with distance
                'angular': 0.0
            }

            return {
                'type': 'navigation_command',
                'data': navigation_command,
                'timestamp': time.time(),
                'source': 'odometry'
            }

        return None

    def process_camera_data(self, camera_data):
        """
        Process camera data with AI model.

        Args:
            camera_data: Camera information

        Returns:
            dict: Decision based on camera data
        """
        # Example: Simple decision based on camera data
        # In a real application, you would process the image here
        if camera_data['data_size'] > 0:
            # Placeholder for image processing
            return {
                'type': 'camera_analysis',
                'data': {'processed': True, 'size': camera_data['data_size']},
                'timestamp': time.time(),
                'source': 'camera'
            }

        return None

    def command_execution_thread(self):
        """
        Background thread for command execution.
        """
        while rclpy.ok():
            try:
                # Get decision from queue
                decision = self.decision_queue.get(timeout=1.0)

                # Execute the decision
                self.execute_decision(decision)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Command execution error: {e}')

    def execute_decision(self, decision):
        """
        Execute an AI decision by publishing appropriate commands.

        Args:
            decision: Decision from the AI model
        """
        try:
            decision_type = decision['type']
            decision_data = decision['data']

            if decision_type == 'joint_command':
                self.execute_joint_command(decision_data)
            elif decision_type == 'balance_command':
                self.execute_balance_command(decision_data)
            elif decision_type == 'avoidance_command':
                self.execute_avoidance_command(decision_data)
            elif decision_type == 'navigation_command':
                self.execute_navigation_command(decision_data)
            elif decision_type == 'camera_analysis':
                self.execute_camera_analysis(decision_data)
            else:
                self.get_logger().warn(f'Unknown decision type: {decision_type}')

            # Add to decision history
            self.ai_decision_history.append(decision)

            # Keep history size reasonable
            if len(self.ai_decision_history) > 1000:
                self.ai_decision_history.pop(0)

        except Exception as e:
            self.get_logger().error(f'Error executing decision: {e}')

    def execute_joint_command(self, commands):
        """
        Execute joint command by publishing to joint controller.

        Args:
            commands: Joint position commands
        """
        cmd_msg = Float64MultiArray()
        cmd_msg.data = commands
        self.joint_cmd_pub.publish(cmd_msg)

        self.get_logger().debug(f'Published joint command: {commands[:3]}...')

    def execute_balance_command(self, commands):
        """
        Execute balance command (example implementation).

        Args:
            commands: Balance correction commands
        """
        # In a real implementation, this would adjust joint commands for balance
        cmd_msg = Float64MultiArray()
        cmd_msg.data = commands
        self.joint_cmd_pub.publish(cmd_msg)

        self.get_logger().debug(f'Published balance command: {commands[:3]}...')

    def execute_avoidance_command(self, command_dict):
        """
        Execute obstacle avoidance command.

        Args:
            command_dict: Dictionary with linear and angular components
        """
        cmd_msg = Twist()
        cmd_msg.linear.x = command_dict['linear']
        cmd_msg.angular.z = command_dict['angular']
        self.cmd_vel_pub.publish(cmd_msg)

        self.get_logger().debug(f'Published avoidance command: linear={command_dict["linear"]}, angular={command_dict["angular"]}')

    def execute_navigation_command(self, command_dict):
        """
        Execute navigation command.

        Args:
            command_dict: Dictionary with linear and angular components
        """
        cmd_msg = Twist()
        cmd_msg.linear.x = command_dict['linear']
        cmd_msg.angular.z = command_dict['angular']
        self.cmd_vel_pub.publish(cmd_msg)

        self.get_logger().debug(f'Published navigation command: linear={command_dict["linear"]}, angular={command_dict["angular"]}')

    def execute_camera_analysis(self, analysis_result):
        """
        Execute action based on camera analysis.

        Args:
            analysis_result: Result of camera analysis
        """
        # In a real implementation, this would trigger actions based on visual input
        self.get_logger().debug(f'Processed camera analysis: {analysis_result}')

    def plan_service_callback(self, request, response):
        """
        Service callback for planning actions.

        Args:
            request: Service request
            response: Service response to fill

        Returns:
            response: Service response
        """
        try:
            # Generate a plan using AI model
            plan = self.generate_plan()

            if plan:
                response.success = True
                response.message = f'Plan generated with {len(plan)} steps'
                self.get_logger().info(f'Plan generated: {response.message}')
            else:
                response.success = False
                response.message = 'Failed to generate plan'
                self.get_logger().warn(response.message)

        except Exception as e:
            response.success = False
            response.message = f'Error generating plan: {e}'
            self.get_logger().error(response.message)

        return response

    def generate_plan(self):
        """
        Generate a plan using AI model.

        Returns:
            list: Plan steps or None if generation failed
        """
        # Example: Simple plan generation
        # In a real implementation, this would use a planning algorithm
        current_pos = self.current_sensor_data.get('odometry', {}).get('pose', {}).get('position', [0, 0, 0])

        # Simple plan: move in a square pattern
        plan = [
            {'target': [current_pos[0] + 1, current_pos[1], current_pos[2]], 'action': 'move'},
            {'target': [current_pos[0] + 1, current_pos[1] + 1, current_pos[2]], 'action': 'move'},
            {'target': [current_pos[0], current_pos[1] + 1, current_pos[2]], 'action': 'move'},
            {'target': current_pos, 'action': 'move'}
        ]

        return plan

    def learning_service_callback(self, request, response):
        """
        Service callback for setting learning mode.

        Args:
            request: Service request with data boolean
            response: Service response to fill

        Returns:
            response: Service response
        """
        self.ai_model_state['learning_enabled'] = request.data

        response.success = True
        response.message = f'Learning mode set to {request.data}'

        self.get_logger().info(f'Learning mode changed to: {request.data}')

        return response

    def publish_status(self):
        """
        Publish status information.
        """
        status_msg = String()
        status_msg.data = f'AI Node Active - Decisions: {len(self.ai_decision_history)}, Learning: {self.ai_model_state["learning_enabled"]}'
        self.status_pub.publish(status_msg)

    def destroy_node(self):
        """
        Clean up resources when node is destroyed.
        """
        # Shutdown thread pool
        self.ai_executor.shutdown(wait=True)

        # Clean up any other resources
        super().destroy_node()


def main(args=None):
    """
    Main function to run the complete AI integration node.

    This follows the standard ROS 2 Python node pattern:
    1. Initialize ROS communication
    2. Create node instance
    3. Spin the node (process callbacks)
    4. Clean up when done
    """
    # Initialize ROS communications
    rclpy.init(args=args)

    # Create the complete AI integration node
    ai_integration = CompleteAIIntegration()

    try:
        # Use rclpy.spin() to process callbacks until the program is shut down
        rclpy.spin(ai_integration)
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        ai_integration.get_logger().info('Complete AI Integration node interrupted by user')
    finally:
        # Clean up: destroy the node and shut down ROS communications
        ai_integration.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()