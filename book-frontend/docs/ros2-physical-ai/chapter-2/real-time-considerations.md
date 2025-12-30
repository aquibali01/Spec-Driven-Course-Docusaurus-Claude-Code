---
sidebar_position: 3
---

# Real-Time Considerations

## Introduction

Real-time performance is critical in humanoid robotics applications. Unlike traditional software systems, robots operate in the physical world where timing constraints can affect safety, stability, and performance. This section explores how ROS 2's communication primitives handle real-time requirements and how to configure them appropriately.

## Understanding Real-Time Requirements

### Hard vs. Soft Real-Time

#### Hard Real-Time
- **Definition**: Missing a deadline is equivalent to a failure
- **Examples**: Safety-critical commands, balance control, collision avoidance
- **Requirements**: Deterministic behavior with guaranteed response times

#### Soft Real-Time
- **Definition**: Missing a deadline degrades performance but doesn't cause failure
- **Examples**: Perception processing, path planning, non-critical feedback
- **Requirements**: Optimized average performance with acceptable deadline misses

### Timing Constraints in Robotics

| Application | Deadline | Criticality |
|-------------|----------|-------------|
| Balance Control | &lt;10ms | Hard Real-Time |
| Collision Avoidance | &lt;50ms | Hard Real-Time |
| Sensor Processing | &lt;100ms | Soft Real-Time |
| Path Planning | &lt;500ms | Soft Real-Time |
| High-Level Decision Making | &lt;2000ms | Soft Real-Time |

## Quality of Service (QoS) Settings

QoS settings in ROS 2 allow you to specify how messages should be handled, which directly impacts real-time performance.

### Reliability Policy

#### Reliable
- **Use Case**: When you must ensure every message is delivered
- **Performance**: Higher latency, more overhead
- **Example**: Configuration commands, safety-critical data

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE
)
```

#### Best Effort
- **Use Case**: When performance is more important than delivery guarantee
- **Performance**: Lower latency, less overhead
- **Example**: Sensor data, camera feeds, non-critical status

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

qos_profile = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT
)
```

### Durability Policy

#### Volatile
- **Use Case**: When only fresh data matters
- **Performance**: Lower storage overhead
- **Example**: Sensor readings, live video feeds

#### Transient Local
- **Use Case**: When late-joining subscribers need initial state
- **Performance**: Higher storage overhead
- **Example**: Configuration parameters, robot state

### History Policy

#### Keep Last
- **Use Case**: When only the most recent values matter
- **Performance**: Lower storage, faster access
- **Example**: Sensor readings, robot position

#### Keep All
- **Use Case**: When historical data is important
- **Performance**: Higher storage requirements
- **Example**: Logging, debugging, audit trails

## Communication Primitive Timing Characteristics

### Topics: Publish/Subscribe

#### Latency Considerations
- **Network Latency**: Time for message to travel between nodes
- **Processing Latency**: Time to serialize/deserialize messages
- **Queue Latency**: Time messages spend in publisher/subscriber queues

#### Configuration for Real-Time Performance
```python
# For safety-critical sensor data
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

safety_qos = QoSProfile(
    depth=1,  # Minimal queue to reduce latency
    reliability=ReliabilityPolicy.RELIABLE,  # Must not lose safety data
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

# For high-frequency sensor data
sensor_qos = QoSProfile(
    depth=5,  # Small queue to reduce latency
    reliability=ReliabilityPolicy.BEST_EFFORT,  # Allow some data loss
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)
```

### Services: Request/Response

#### Timing Considerations
- **Round-Trip Time**: Time for request to reach server and response to return
- **Processing Time**: Time server takes to process request
- **Blocking Nature**: Client is blocked until response received

#### Real-Time Service Design
```python
# For time-critical service calls
def create_time_critical_service(self):
    # Use reliable communication for critical services
    # Set appropriate timeouts in client code
    pass

# Client with timeout handling
def call_with_timeout(self):
    future = self.cli.call_async(request)
    # Use executor with timeout to avoid indefinite blocking
    rclpy.spin_until_future_complete(self, future, timeout_sec=0.1)
```

### Actions: Goal-Based

#### Timing Considerations
- **Goal Acceptance**: Time to accept/reject a goal
- **Feedback Frequency**: How often feedback is sent
- **Result Delivery**: Time to return final result

#### Real-Time Action Configuration
```python
# For time-critical actions
def configure_critical_action(self):
    # Set appropriate feedback frequency
    # Implement cancellation for safety
    # Monitor execution time and handle timeouts
    pass
```

## Performance Optimization Strategies

### Message Design
- **Minimize Message Size**: Smaller messages transmit faster
- **Use Efficient Data Types**: Choose appropriate types for your data
- **Batch Related Data**: Combine related information in single messages

### Node Architecture
- **Minimize Processing**: Keep node callbacks fast and lightweight
- **Use Separate Threads**: For heavy processing that shouldn't block callbacks
- **Optimize Callbacks**: Keep message processing callbacks minimal

### Network Configuration
- **Local Communication**: Use localhost for high-frequency communication
- **Network Quality**: Ensure reliable network for distributed systems
- **Bandwidth Management**: Prioritize critical communication channels

## Example: Real-Time Sensor Processing

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu

class RealTimeSensorProcessor(Node):
    def __init__(self):
        super().__init__('real_time_sensor_processor')

        # Configure QoS for real-time sensor data
        sensor_qos = QoSProfile(
            depth=1,  # Minimal queue for lowest latency
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Allow some loss for performance
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribe to IMU data with real-time QoS
        self.subscription = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            sensor_qos
        )

        # Ensure callback is as fast as possible
        self.processing_time = 0.0  # Keep track of processing time

    def imu_callback(self, msg):
        # Time-critical processing
        start_time = self.get_clock().now()

        # Process IMU data for balance control
        self.process_imu_data(msg)

        # Log processing time for monitoring
        end_time = self.get_clock().now()
        self.processing_time = (end_time.nanoseconds - start_time.nanoseconds) / 1e9
```

## Monitoring and Testing

### Performance Metrics
- **Message Latency**: Time from publish to receive
- **Callback Execution Time**: Time spent in message callbacks
- **System Load**: CPU and memory usage during operation
- **Deadline Compliance**: Percentage of operations meeting timing requirements

### Testing Strategies
- **Load Testing**: Test performance under expected message rates
- **Stress Testing**: Test limits and failure modes
- **Timing Validation**: Verify timing constraints are met
- **Integration Testing**: Test complete communication chains

## Best Practices for Real-Time ROS 2 Systems

1. **Profile Everything**: Measure actual performance, don't assume
2. **Design for Worst Case**: Plan for peak loads, not average loads
3. **Use Appropriate QoS**: Match QoS settings to your requirements
4. **Minimize Blocking**: Avoid blocking operations in time-critical callbacks
5. **Monitor Continuously**: Implement runtime monitoring of timing metrics
6. **Plan for Failures**: Design graceful degradation when timing fails
7. **Consider Hardware**: Account for hardware limitations in your design
8. **Test in Real Environments**: Validate performance in actual deployment conditions

Understanding and properly configuring real-time considerations is essential for developing safe and effective humanoid robotics applications with ROS 2.