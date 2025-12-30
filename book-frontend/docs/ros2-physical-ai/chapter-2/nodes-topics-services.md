---
sidebar_position: 2
---

# Communication Primitives Overview

## Introduction

ROS 2 provides four primary communication primitives that enable different types of interaction between nodes: **Nodes**, **Topics**, **Services**, and **Actions**. Understanding when and how to use each primitive is essential for designing effective robot systems.

## Nodes: The Basic Computational Unit

### What is a Node?

A node is a process that performs computation in ROS 2. It's the fundamental building block of a ROS 2 system.

### Key Characteristics:
- **Independent Process**: Each node runs as a separate process
- **Communication Hub**: Nodes can publish to topics, subscribe to topics, provide services, and call services
- **Identifiable**: Each node has a unique name and can be organized in namespaces
- **Lifecycle**: Nodes can be started, stopped, and managed independently

### Example Node Structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Node initialization code here
```

## Topics: Publish/Subscribe Communication

### What are Topics?

Topics enable asynchronous, one-to-many communication using a publish/subscribe pattern. Publishers send messages to a topic, and any number of subscribers can receive those messages.

### Key Characteristics:
- **Asynchronous**: Publishers and subscribers don't need to run simultaneously
- **One-to-Many**: One publisher can send to many subscribers
- **Data-Centric**: Communication is organized around data streams
- **Real-Time Friendly**: Low-latency, high-frequency communication

### Use Cases:
- **Sensor Data**: Camera feeds, LIDAR scans, IMU readings
- **Robot State**: Joint positions, battery levels, system status
- **Broadcasting**: Sharing information with multiple interested parties

### Example Topic Publisher:

```python
from std_msgs.msg import String

def create_publisher():
    publisher = self.create_publisher(String, 'topic_name', 10)
    # 10 is the queue size for messages
```

### Example Topic Subscriber:

```python
def create_subscriber(self):
    self.subscription = self.create_subscription(
        String,
        'topic_name',
        self.listener_callback,
        10)  # Queue size

def listener_callback(self, msg):
    self.get_logger().info(f'I heard: {msg.data}')
```

## Services: Request/Response Communication

### What are Services?

Services provide synchronous, one-to-one communication using a request/response pattern. A client sends a request and waits for a response from a server.

### Key Characteristics:
- **Synchronous**: Client waits for the server's response
- **One-to-One**: One client communicates with one server
- **Request/Response**: Structured exchange with defined inputs and outputs
- **Blocking**: The client is blocked until the response is received

### Use Cases:
- **Configuration**: Setting parameters or changing robot configuration
- **One-Time Actions**: Commands that execute once and return a result
- **Querying**: Requesting specific information from a service provider

### Example Service Server:

```python
from example_interfaces.srv import AddTwoInts

def create_service_server(self):
    self.srv = self.create_service(
        AddTwoInts,
        'add_two_ints',
        self.add_two_ints_callback)

def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info(f'Returning {response.sum}')
    return response
```

### Example Service Client:

```python
def create_client(self):
    self.cli = self.create_client(AddTwoInts, 'add_two_ints')

def call_service(self):
    if not self.cli.service_is_ready():
        return

    request = AddTwoInts.Request()
    request.a = 41
    request.b = 1
    self.future = self.cli.call_async(request)
```

## Actions: Goal-Based Communication

### What are Actions?

Actions provide asynchronous, goal-based communication with feedback and status. They're designed for long-running tasks that provide intermediate feedback.

### Key Characteristics:
- **Asynchronous**: Non-blocking communication
- **Goal-Based**: Tasks with defined objectives
- **Feedback**: Continuous updates during execution
- **Cancelation**: Ability to cancel long-running tasks
- **Result**: Final outcome when the task completes

### Use Cases:
- **Navigation**: Moving to a specific location with progress updates
- **Manipulation**: Grasping objects with feedback on progress
- **Calibration**: Long-running processes with status updates
- **Complex Tasks**: Multi-step operations with intermediate results

### Example Action Server:

```python
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer

def create_action_server(self):
    self._action_server = ActionServer(
        self,
        Fibonacci,
        'fibonacci',
        self.execute_callback)

def execute_callback(self, goal_handle):
    feedback_msg = Fibonacci.Feedback()
    feedback_msg.sequence = [0, 1]

    for i in range(1, goal_handle.request.order):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Fibonacci.Result()

        feedback_msg.sequence.append(
            feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
        goal_handle.publish_feedback(feedback_msg)

    goal_handle.succeed()
    result = Fibonacci.Result()
    result.sequence = feedback_msg.sequence
    return result
```

## Comparison Table

| Primitive | Pattern | Synchronous | Use Case | Example |
|-----------|---------|-------------|----------|---------|
| Topic | Pub/Sub | No | Data streams | Sensor data |
| Service | Req/Res | Yes | One-time requests | Configuration |
| Action | Goal/FB | No | Long-running tasks | Navigation |

## Choosing the Right Primitive

### For Sensor Data
- **Use Topics**: High-frequency data streams that multiple nodes might need
- **Example**: Camera images, LIDAR scans, IMU data

### For Configuration
- **Use Services**: When you need to set parameters or get specific information
- **Example**: Changing robot settings, querying system status

### For Long-Running Tasks
- **Use Actions**: When tasks take time and need to provide feedback
- **Example**: Moving to a location, executing a complex manipulation

### For System Status
- **Use Topics**: For continuous status updates that multiple nodes might monitor
- **Example**: Battery level, system health, operational mode

## Best Practices

1. **Use Topics for Data**: Choose topics for continuous data streams
2. **Use Services for Commands**: Choose services for immediate, one-time operations
3. **Use Actions for Processes**: Choose actions for long-running tasks with feedback
4. **Consider QoS**: Always consider Quality of Service settings for your use case
5. **Design Message Types**: Create appropriate message types for your specific needs
6. **Handle Errors**: Implement proper error handling for all communication types

Understanding these communication primitives is crucial for designing effective AI-robot interaction patterns. In the next sections, we'll explore how to apply these concepts with real-time considerations and AI decision loop integration.