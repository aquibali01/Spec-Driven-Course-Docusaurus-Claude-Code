---
sidebar_position: 3
---

# Linking Controllers

## Introduction

In ROS 2, controllers bridge the gap between the high-level robot description (URDF) and the actual hardware or simulation. Controllers manage the robot's joints and actuators, translating high-level commands into low-level actuator signals. This section covers how to connect URDF models to controllers using the ROS 2 Control framework.

## ROS 2 Control Overview

ROS 2 Control is the standard framework for controlling robots in ROS 2. It provides:
- Hardware abstraction through the hardware interface
- Controller management through the controller manager
- Standardized interfaces for different types of controllers
- Real-time capabilities for safety-critical applications

### Key Components

#### Hardware Interface
- **Purpose**: Abstracts communication with physical hardware or simulation
- **Function**: Reads from sensors and writes to actuators
- **Implementation**: Custom hardware interface for your specific robot

#### Controller Manager
- **Purpose**: Manages the lifecycle of controllers
- **Function**: Load, start, stop, and unload controllers
- **Implementation**: Provided by the controller manager package

#### Controllers
- **Purpose**: Implement specific control algorithms
- **Function**: Process commands and generate actuator signals
- **Types**: Joint trajectory controllers, forward command controllers, etc.

## URDF to Controller Connection

### Transmission Elements

The transmission element in URDF defines how joints connect to actuators:

```xml
<transmission name="transmission_joint1">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="joint1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="motor1">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

### Hardware Interface Types

#### Position Interface
For position-controlled joints:

```xml
<hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
```

#### Velocity Interface
For velocity-controlled joints:

```xml
<hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
```

#### Effort Interface
For effort-controlled joints:

```xml
<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
```

## Controller Configuration

### Controller Manager Configuration

Controllers are configured using YAML files that define their parameters:

```yaml
# Controller configuration file
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    # Define available controllers
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    # Define a position controller
    position_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    # Define a velocity controller
    velocity_controller:
      type: joint_trajectory_controller/JointTrajectoryController
```

### Joint Trajectory Controller Configuration

```yaml
position_controller:
  ros__parameters:
    joints:
      - joint1
      - joint2
      - joint3

    interface_name: position

    # Command and state interfaces
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

    # Goal and trajectory tolerances
    goal_time: 0.5
    constraints:
      stopped_velocity_tolerance: 0.01
      joint1:
        trajectory: 0.05
        goal: 0.01
      joint2:
        trajectory: 0.05
        goal: 0.01
```

## Example: Complete Robot Setup

### URDF with Transmissions

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Links and joints as defined in previous sections -->

  <!-- Transmission for joint1 -->
  <transmission name="trans_joint1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor1">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>

  <!-- Transmission for joint2 -->
  <transmission name="trans_joint2">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint2">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor2">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
</robot>
```

### Hardware Interface Implementation

A basic hardware interface implementation:

```python
import rclpy
from rclpy.node import Node
from hardware_interface import HardwareInterface, HardwareInfo
from controller_manager import ControllerManager
from builtin_interfaces.msg import Time


class MyRobotHardwareInterface(HardwareInterface):
    """
    Custom hardware interface for a robot.

    This class handles communication with the actual hardware.
    """

    def __init__(self, hardware_info: HardwareInfo):
        super().__init__()
        self.hardware_info = hardware_info

        # Initialize joint positions, velocities, and efforts
        self.joint_positions = [0.0] * len(hardware_info.joints)
        self.joint_velocities = [0.0] * len(hardware_info.joints)
        self.joint_efforts = [0.0] * len(hardware_info.joints)

        # Initialize command values
        self.joint_position_commands = [0.0] * len(hardware_info.joints)

    def configure(self, params: dict) -> bool:
        """Configure the hardware interface."""
        # Initialize communication with hardware
        return True

    def start(self) -> bool:
        """Start the hardware interface."""
        # Start hardware communication
        return True

    def stop(self) -> bool:
        """Stop the hardware interface."""
        # Stop hardware communication safely
        return True

    def read(self, time: Time, period: Duration) -> bool:
        """Read state from hardware."""
        # Read current joint positions, velocities, and efforts from hardware
        # This is called by the controller manager at the update rate
        return True

    def write(self, time: Time, period: Duration) -> bool:
        """Write commands to hardware."""
        # Send joint position commands to hardware
        # This is called by the controller manager at the update rate
        return True
```

### Controller Manager Node

```python
import rclpy
from rclpy.node import Node
from controller_manager.controller_manager import ControllerManager


class MyRobotControllerManager(Node):
    """
    Controller manager for the robot.

    This node manages the lifecycle of controllers.
    """

    def __init__(self):
        super().__init__('my_robot_controller_manager')

        # Create the controller manager
        # This would typically use a hardware interface
        self.controller_manager = ControllerManager(
            self,
            update_rate=100  # 100 Hz
        )

        # The controller manager handles loading, starting, and stopping controllers
```

## Common Controller Types

### Joint Trajectory Controller

The most common controller type that accepts trajectories (position, velocity, acceleration) for multiple joints:

```yaml
joint_trajectory_controller:
  ros__parameters:
    type: joint_trajectory_controller/JointTrajectoryController
    joints:
      - joint1
      - joint2
      - joint3
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
```

### Forward Command Controller

For controllers that receive direct commands (e.g., velocity commands):

```yaml
forward_command_controller:
  ros__parameters:
    type: forward_command_controller/ForwardCommandController
    joints:
      - joint1
      - joint2
    interface_name: velocity
```

### Joint State Broadcaster

Publishes joint states for visualization and other nodes:

```yaml
joint_state_broadcaster:
  ros__parameters:
    type: joint_state_broadcaster/JointStateBroadcaster
```

## Launch Files

Controllers are typically launched using launch files:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open('path/to/robot.urdf').read()}]
    )

    # Controller manager node
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[PathJoinSubstitution([
            FindPackageShare('my_robot_description'),
            'config',
            'controllers.yaml'
        ])],
        remappings=[
            ('/joint_states', 'joint_states'),
        ]
    )

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    # Spawn position controller
    position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['position_controller'],
    )

    # Delay controllers until controller manager is ready
    delay_controllers_after_spawner_started = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[
                joint_state_broadcaster_spawner,
                position_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager_node,
        delay_controllers_after_spawner_started,
    ])
```

## Best Practices for Controller Linking

1. **Separate URDF from Controller Configuration**: Keep robot description separate from controller configuration
2. **Use Standard Interfaces**: Stick to standard hardware interfaces when possible
3. **Validate Transmissions**: Ensure all joints have proper transmission definitions
4. **Configure Update Rates**: Set appropriate update rates for your application
5. **Plan for Safety**: Implement safety limits and emergency stops
6. **Test in Simulation**: Test controller configurations in simulation first
7. **Monitor Performance**: Monitor controller performance and update rates
8. **Use Standard Controllers**: Leverage existing standard controllers when possible

## Troubleshooting Common Issues

### Controller Not Loading
- Check that the controller type is correctly specified
- Verify that the controller package is installed
- Ensure the controller manager is running

### Joint Not Responding
- Check that the joint name matches between URDF, transmission, and controller configuration
- Verify that the hardware interface type matches
- Check that the controller is running and in the correct state

### Performance Issues
- Monitor the controller update rate
- Check for timing issues in the hardware interface
- Optimize the controller update rate for your application

Understanding how to properly link URDF models to controllers is essential for creating functional robots in ROS 2. This connection enables the bridge between the robot's mechanical design and its control system.