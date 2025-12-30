---
sidebar_position: 4
---

# AI Decision Loops Mapping

## Introduction

AI agents in robotics operate in a continuous decision loop that involves sensing, processing, deciding, and acting. Mapping these decision loops to ROS 2 communication primitives is crucial for creating effective AI-robot integration. This section explores how to structure AI decision-making processes using ROS 2's communication patterns.

## The AI Decision Loop

### Basic Structure

The fundamental AI decision loop consists of four stages:

```
Perception → Planning → Action → Perception (repeat)
```

In ROS 2 terms, this translates to:

```
Sensors (Topics) → AI Processing (Node) → Actuators (Topics/Services/Actions) → Sensors (repeat)
```

### Detailed Loop Components

1. **Perception**: Gathering information from sensors via ROS 2 topics
2. **State Estimation**: Combining sensor data to understand current state
3. **Goal Evaluation**: Assessing progress toward objectives
4. **Planning**: Determining appropriate actions
5. **Action Execution**: Sending commands to actuators
6. **Monitoring**: Observing results and adjusting future decisions

## Mapping Decision Loop Stages to ROS 2

### 1. Perception Stage

#### ROS 2 Mapping: Topics (Subscribers)
- **Purpose**: Continuous intake of sensor information
- **Implementation**: Subscribe to sensor topics
- **QoS Considerations**: Best effort for high-frequency data, reliable for critical sensors

```python
class AIController(Node):
    def __init__(self):
        super().__init__('ai_controller')

        # Subscribe to various sensor streams
        self.camera_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.camera_callback,
            qos_profile=QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            'lidar/scan',
            self.lidar_callback,
            qos_profile=QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT
            )
        )

        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE  # Critical for balance
            )
        )
```

### 2. State Estimation

#### ROS 2 Mapping: Internal Node Processing
- **Purpose**: Combine sensor data into coherent state representation
- **Implementation**: Process callbacks and maintain internal state
- **Considerations**: Real-time processing, thread safety

```python
def camera_callback(self, msg):
    # Process camera data
    processed_image = self.process_camera_data(msg)

    # Update internal state
    self.current_state['image'] = processed_image
    self.update_belief_state()

def lidar_callback(self, msg):
    # Process LIDAR data
    obstacles = self.detect_obstacles(msg)

    # Update internal state
    self.current_state['obstacles'] = obstacles
    self.update_world_model()
```

### 3. Goal Evaluation and Planning

#### ROS 2 Mapping: Internal Node Processing
- **Purpose**: Determine appropriate actions based on state and goals
- **Implementation**: Planning algorithms running in the AI node
- **Considerations**: Computational complexity, timing requirements

```python
def evaluate_and_plan(self):
    # Check current goals and state
    if self.current_goal is None:
        self.set_new_goal()

    # Plan next action based on state
    if self.near_obstacle():
        avoidance_action = self.plan_avoidance()
        self.execute_action(avoidance_action)
    elif self.near_goal():
        approach_action = self.plan_approach()
        self.execute_action(approach_action)
    else:
        navigation_action = self.plan_navigation()
        self.execute_action(navigation_action)
```

### 4. Action Execution

#### ROS 2 Mapping: Topics, Services, or Actions
- **Topics**: For continuous control signals (motors, actuators)
- **Services**: For discrete commands (configuration, mode changes)
- **Actions**: For complex, long-running tasks (navigation, manipulation)

```python
# For continuous motor control
def publish_motor_commands(self, commands):
    motor_msg = JointState()
    motor_msg.position = commands.positions
    motor_msg.velocity = commands.velocities
    motor_msg.effort = commands.efforts
    self.motor_pub.publish(motor_msg)

# For discrete configuration changes
def change_robot_mode(self, new_mode):
    request = SetMode.Request()
    request.mode = new_mode
    future = self.mode_client.call_async(request)
    return future

# For complex navigation tasks
def navigate_to_goal(self, x, y, theta):
    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = PoseStamped()
    goal_msg.pose.pose.position.x = x
    goal_msg.pose.pose.position.y = y
    goal_msg.pose.pose.orientation = self.euler_to_quaternion(0, 0, theta)

    self.nav_action_client.send_goal_async(goal_msg)
```

## Common AI Decision Loop Patterns

### 1. Reactive Control Pattern

Simple, immediate responses to sensor inputs:

```python
class ReactiveController(Node):
    def __init__(self):
        super().__init__('reactive_controller')
        # Subscribe to sensors
        # Publish to actuators

    def sensor_callback(self, msg):
        # Immediate response to sensor input
        action = self.reactive_policy(msg)
        self.actuator_publisher.publish(action)
```

**Best for**: Safety-critical reflexes, immediate obstacle avoidance

### 2. Deliberative Planning Pattern

Complex planning based on world model:

```python
class DeliberativeController(Node):
    def __init__(self):
        super().__init__('deliberative_controller')
        # Subscribers, publishers, services, actions

    def decision_loop(self):
        while rclpy.ok():
            # Update world model from sensors
            self.update_world_model()

            # Plan sequence of actions
            action_sequence = self.plan_sequence()

            # Execute first action
            self.execute_action(action_sequence[0])

            # Wait and repeat
            time.sleep(0.1)  # Planning frequency
```

**Best for**: Path planning, task planning, complex navigation

### 3. Hybrid Architecture Pattern

Combination of reactive and deliberative elements:

```python
class HybridController(Node):
    def __init__(self):
        super().__init__('hybrid_controller')
        # Multiple layers of control

    def sensor_callback(self, msg):
        # Safety layer (reactive)
        if self.is_dangerous_state():
            self.execute_safety_action()
            return

        # Planning layer (deliberative)
        if self.should_replan():
            self.update_plan()

        # Execution layer
        next_action = self.get_next_action()
        self.execute_action(next_action)
```

**Best for**: Complex robots requiring both reflexes and planning

## Performance Considerations

### Loop Frequency Requirements

| Loop Type | Typical Frequency | Example |
|-----------|-------------------|---------|
| Safety Reflex | 100-1000 Hz | Balance, collision avoidance |
| Control Loop | 50-200 Hz | Motor control, stabilization |
| Planning Loop | 1-20 Hz | Path planning, behavior selection |
| High-Level Planning | 0.1-1 Hz | Task planning, mission management |

### ROS 2 Configuration for Different Loops

```python
# High-frequency safety loop
safety_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

# Medium-frequency control loop
control_qos = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST
)

# Low-frequency planning loop
planning_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST
)
```

## Example: Complete AI Decision Loop

```python
class AIDecisionLoop(Node):
    def __init__(self):
        super().__init__('ai_decision_loop')

        # Perception: Subscribe to sensors
        self.camera_sub = self.create_subscription(
            Image, 'camera/image_raw', self.camera_callback, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, 'lidar/scan', self.lidar_callback, 10
        )

        # Action: Publish to actuators
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Planning timer
        self.plan_timer = self.create_timer(0.1, self.decision_callback)  # 10 Hz planning

        # Internal state
        self.sensor_data = {}
        self.current_goal = None
        self.current_plan = []

    def camera_callback(self, msg):
        self.sensor_data['camera'] = self.process_image(msg)

    def lidar_callback(self, msg):
        self.sensor_data['lidar'] = self.process_lidar(msg)

    def decision_callback(self):
        # Perception: Update state from sensors
        current_state = self.update_state(self.sensor_data)

        # Planning: Decide on next action
        if not self.current_plan or self.is_plan_obsolete():
            self.current_plan = self.replan(current_state, self.current_goal)

        # Action: Execute next step
        if self.current_plan:
            next_action = self.current_plan[0]
            self.cmd_vel_pub.publish(next_action)

            # Remove executed action
            self.current_plan.pop(0)
```

## Best Practices for AI-ROS 2 Integration

1. **Match Loop Frequency**: Align your AI loop frequency with task requirements
2. **Use Appropriate Primitives**: Choose the right ROS 2 primitive for each interaction
3. **Consider Timing**: Account for communication delays in your decision making
4. **Handle Asynchrony**: Design for asynchronous communication patterns
5. **Maintain State**: Keep internal state consistent with ROS 2 messages
6. **Plan for Failures**: Handle communication and execution failures gracefully
7. **Monitor Performance**: Track loop timing and adjust as needed
8. **Separate Concerns**: Keep perception, planning, and action logic organized

Understanding how to map AI decision loops to ROS 2 communication primitives enables the creation of effective, responsive AI systems for robotic applications.