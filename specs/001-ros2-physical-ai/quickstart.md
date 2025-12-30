# Quickstart Guide: ROS 2 for Physical AI & Humanoid Robotics

## Overview
This guide provides a fast path to understanding ROS 2 concepts for AI engineers transitioning to robotics. Follow these steps to get up and running with the core concepts.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended) or equivalent Linux distribution
- Python 3.10 or higher
- Node.js 18+ for Docusaurus site
- At least 8GB RAM (16GB recommended for simulation)

### Software Dependencies
```bash
# ROS 2 Humble Hawksbill installation
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop ros-humble-ros-base
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-buildifier
sudo rosdep init
rosdep update

# Python environment
pip3 install ros-humble-rclpy
pip3 install colcon-common-extensions
```

### Docusaurus Setup
```bash
# Navigate to the website directory
cd website
npm install
```

## Getting Started with ROS 2 Concepts

### 1. Understanding the Robotic Nervous System
ROS 2 acts as the "nervous system" of a robot, connecting:
- **Sensors** (like robot "senses") → publish data on **topics**
- **AI agents** (like robot "brain") → process data and make decisions
- **Actuators** (like robot "muscles") → receive commands to move

### 2. Running Your First ROS 2 Node
Create a simple Python node to understand the basics:

```python
# minimal_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it with:
```bash
python3 minimal_publisher.py
```

### 3. Understanding Communication Primitives
ROS 2 uses four main communication patterns:

#### Topics (Publish/Subscribe)
- **Use case**: Sensor data streaming, status updates
- **Pattern**: One-to-many, asynchronous
- **Example**: Camera images, IMU data

#### Services (Request/Response)
- **Use case**: Configuration, calibration, one-time actions
- **Pattern**: One-to-one, synchronous
- **Example**: Set robot parameters, get robot status

#### Actions (Goal-Based)
- **Use case**: Long-running tasks with feedback
- **Pattern**: One-to-one, with goal/cancel/feedback
- **Example**: Navigation to a goal, arm movement

## Running the Documentation Site

### Local Development
```bash
cd website
npm start
```
Open your browser to http://localhost:3000 to view the documentation.

### Build for Production
```bash
cd website
npm run build
```

## Key Concepts to Master

### 1. The Three Chapters
1. **Role of ROS 2 in Physical AI**: Understanding ROS 2 as a nervous system
2. **Communication Primitives**: Nodes, topics, services, and actions
3. **Robot Modeling**: URDF and connecting AI agents with rclpy

### 2. Essential ROS 2 Commands
```bash
# List active nodes
ros2 node list

# List active topics
ros2 topic list

# Echo messages from a topic
ros2 topic echo /topic_name std_msgs/msg/String

# List available services
ros2 service list
```

### 3. Python Integration Pattern
```python
# Standard ROS 2 Python node structure
import rclpy
from rclpy.node import Node

class MyAIController(Node):
    def __init__(self):
        super().__init__('ai_controller')
        # Create publishers, subscribers, etc.

    def sensor_callback(self, msg):
        # Process sensor data with AI algorithm
        command = self.ai_process(msg)
        self.command_publisher.publish(command)

def main(args=None):
    rclpy.init(args=args)
    controller = MyAIController()
    rclpy.spin(controller)
```

## Next Steps
1. Complete Chapter 1: Understand ROS 2 as a robotic nervous system
2. Complete Chapter 2: Master communication primitives
3. Complete Chapter 3: Connect AI agents using rclpy
4. Try the hands-on examples with real robot simulation

## Troubleshooting
- If ROS 2 commands are not found: Source the setup script: `source /opt/ros/humble/setup.bash`
- If Python modules are not found: Ensure you've installed rclpy in your Python environment
- For Docusaurus issues: Check that Node.js and npm are properly installed

## Resources
- [Official ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [rclpy API Documentation](https://docs.ros.org/en/humble/p/rclpy/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)