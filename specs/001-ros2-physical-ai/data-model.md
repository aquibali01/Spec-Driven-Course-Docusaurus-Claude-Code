# Data Model: ROS 2 for Physical AI & Humanoid Robotics

## Overview
This document defines the key data models and entities for the ROS 2 educational module, focusing on the conceptual entities identified in the feature specification.

## Entity 1: ROS 2 Communication Primitives

### Definition
The fundamental building blocks of ROS 2 communication including nodes (processes), topics (publish/subscribe), services (request/response), and actions (goal-based communication).

### Attributes
- **Node**: Process running ROS 2 client library
  - Name: Unique identifier for the node
  - Namespace: Hierarchical organization
  - Parameters: Configuration values
  - Publishers: List of topics published
  - Subscribers: List of topics subscribed
  - Services: List of services provided
  - Clients: List of services used

- **Topic**: Communication channel for publish/subscribe
  - Name: Unique topic identifier
  - Message type: Type definition for messages
  - QoS profile: Quality of service settings
  - Publishers count: Number of publishers
  - Subscribers count: Number of subscribers

- **Service**: Request/response communication
  - Name: Unique service identifier
  - Request type: Type definition for requests
  - Response type: Type definition for responses
  - Server: Node providing the service
  - Clients: List of nodes using the service

- **Action**: Goal-based communication with feedback
  - Name: Unique action identifier
  - Goal type: Type definition for goals
  - Result type: Type definition for results
  - Feedback type: Type definition for feedback
  - Server: Node providing the action server
  - Clients: List of action clients

### Relationships
- Nodes contain Publishers/Subscribers/Clients/Services
- Topics connect Publishers and Subscribers
- Services connect Servers and Clients
- Actions connect Action Servers and Action Clients

## Entity 2: URDF (Unified Robot Description Format)

### Definition
XML-based format that describes robot models including links, joints, and kinematic properties for humanoid robots.

### Attributes
- **Robot**: Top-level container
  - Name: Robot identifier
  - Version: URDF schema version
  - Links: Collection of rigid bodies
  - Joints: Collection of connections between links

- **Link**: Rigid body element
  - Name: Unique link identifier
  - Inertial: Mass, center of mass, and inertia properties
  - Visual: Visual representation for rendering
  - Collision: Collision geometry for physics simulation

- **Joint**: Connection between two links
  - Name: Unique joint identifier
  - Type: Joint type (revolute, continuous, prismatic, fixed, etc.)
  - Parent: Parent link in kinematic chain
  - Child: Child link in kinematic chain
  - Origin: Transform from parent to child
  - Limits: Joint limits (if applicable)

- **Transmission**: How joints connect to actuators
  - Type: Transmission type
  - Joint: Associated joint
  - Actuator: Associated actuator

### Relationships
- Robot contains multiple Links and Joints
- Joints connect two Links (parent and child)
- Transmissions connect Joints to Actuators

## Entity 3: rclpy (Python Client Library)

### Definition
Python client library for ROS 2 that enables Python-based AI agents to interface with the ROS 2 middleware.

### Attributes
- **Node**: Python ROS 2 node
  - Name: Node identifier
  - Context: ROS 2 context
  - Publishers: List of rclpy Publisher objects
  - Subscribers: List of rclpy Subscriber objects
  - Services: List of rclpy Service objects
  - Clients: List of rclpy Client objects
  - Timers: List of rclpy Timer objects

- **Publisher**: Message publisher
  - Topic name: Destination topic
  - Message type: Type of messages published
  - QoS profile: Quality of service settings

- **Subscriber**: Message subscriber
  - Topic name: Source topic
  - Message type: Type of messages received
  - Callback: Function to handle received messages
  - QoS profile: Quality of service settings

- **Service Server**: Service provider
  - Service name: Service identifier
  - Service type: Type definition
  - Callback: Function to handle requests

- **Service Client**: Service user
  - Service name: Service identifier
  - Service type: Type definition

### Relationships
- Node contains Publishers, Subscribers, Services, Clients, and Timers
- Publishers send to Topics
- Subscribers receive from Topics
- Service Servers provide Services
- Service Clients use Services

## Entity 4: AI Decision Loop

### Definition
The iterative process of AI agents making decisions based on sensor input and sending commands to actuators.

### Attributes
- **Loop**: Iterative decision-making process
  - Input: Sensor data from various sources
  - Processing: AI algorithm execution
  - Output: Commands to actuators
  - State: Current state of the AI system
  - Goal: Target behavior or objective
  - Frequency: Rate of iteration

- **Sensor Input**: Data from robot sensors
  - Type: Sensor type (camera, lidar, IMU, etc.)
  - Data: Raw or processed sensor readings
  - Timestamp: Time of measurement
  - Frame: Coordinate frame of reference

- **AI Processing**: Algorithm execution
  - Algorithm: Specific AI algorithm used
  - Input: Sensor data and state
  - Output: Action decisions
  - Confidence: Confidence level of decisions

- **Actuator Command**: Commands to robot actuators
  - Type: Actuator type (motor, servo, etc.)
  - Command: Specific command value
  - Target: Specific joint or actuator
  - Parameters: Additional command parameters

### Relationships
- AI Decision Loop processes Sensor Input to generate Actuator Commands
- Loop contains iterative cycles of Input → Processing → Output
- Sensor Inputs feed into Processing stage
- Processing outputs connect to Actuator Commands

## Validation Rules

### From Functional Requirements
- **FR-001**: All communication primitives must support the concept of ROS 2 as a nervous system
- **FR-002**: URDF models must support humanoid robot structures with appropriate kinematic chains
- **FR-003**: Communication patterns must support real-time requirements for robot control
- **FR-004**: rclpy integration must support Python-based AI agents
- **FR-005**: All entities must be explainable to AI engineers with no robotics background