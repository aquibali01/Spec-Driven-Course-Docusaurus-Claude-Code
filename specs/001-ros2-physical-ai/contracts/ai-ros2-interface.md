# AI Agent to ROS 2 Interface Contract

## Overview
This contract defines the interface between AI agents and ROS 2 systems as described in the educational module. It represents the conceptual connection that AI engineers will learn to implement.

## Interface: AIController

### Purpose
The AIController interface represents how an AI agent connects to and controls a ROS 2 system. This is the primary integration point that students will learn to implement.

### Operations

#### 1. initialize_robot_connection()
- **Purpose**: Establish connection between AI agent and ROS 2 system
- **Input**: ROS 2 context, robot configuration
- **Output**: Connection status, available sensors/actuators
- **Error cases**: Connection failure, timeout, invalid configuration

#### 2. get_sensor_data()
- **Purpose**: Retrieve current sensor data from the robot
- **Input**: Sensor types to query, optional time window
- **Output**: Sensor data with timestamps, data quality indicators
- **Error cases**: Sensor unavailable, data corruption, timeout

#### 3. process_with_ai_model(input_data)
- **Purpose**: Process sensor data through AI model to generate commands
- **Input**: Sensor data, current state, goal parameters
- **Output**: Robot commands, confidence scores, execution plan
- **Error cases**: Model failure, invalid input, processing timeout

#### 4. send_robot_commands(commands)
- **Purpose**: Send commands to robot actuators via ROS 2
- **Input**: Robot commands, execution parameters
- **Output**: Command acceptance status, execution feedback
- **Error cases**: Command rejected, actuator unavailable, safety violation

#### 5. update_robot_state()
- **Purpose**: Update internal state based on robot feedback
- **Input**: Robot state feedback, execution results
- **Output**: Updated state, learning updates (if applicable)
- **Error cases**: State update failure, inconsistency detected

## Data Models

### SensorData
- **timestamp**: Time of sensor reading (ROS time)
- **sensor_type**: Type of sensor (camera, lidar, IMU, etc.)
- **data**: Raw or processed sensor reading
- **frame_id**: Coordinate frame of reference
- **quality**: Data quality indicator (0.0-1.0)

### RobotCommand
- **command_type**: Type of command (motion, configuration, etc.)
- **target**: Specific actuator or joint
- **parameters**: Command-specific parameters
- **priority**: Execution priority (0-10)
- **timeout**: Maximum execution time

### ConnectionConfig
- **ros_context**: ROS 2 context to use
- **robot_namespace**: Namespace for robot topics/services
- **qos_profile**: Quality of service settings
- **timeout**: Connection timeout value

## Quality of Service (QoS) Guidelines

### For Sensor Data Topics
- **Reliability**: Best effort for high-frequency data, reliable for critical data
- **Durability**: Volatile for streaming data, transient local for configuration
- **History**: Keep last N messages for buffering

### For Command Topics
- **Reliability**: Reliable for safety-critical commands
- **Durability**: Volatile for real-time commands
- **History**: Keep last message for recovery

## Error Handling

### Connection Errors
- Network timeouts
- Node unavailability
- Permission issues

### Data Errors
- Sensor malfunctions
- Data corruption
- Invalid command formats

### Processing Errors
- AI model failures
- State inconsistencies
- Safety violations

## Performance Expectations
- Sensor data retrieval: < 50ms for time-critical data
- AI processing: < 200ms for reactive systems
- Command execution: < 10ms for safety-critical commands
- Connection establishment: < 5s for normal operation