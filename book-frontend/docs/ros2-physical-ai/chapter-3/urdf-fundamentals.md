---
sidebar_position: 2
---

# URDF Fundamentals

## Introduction

URDF (Unified Robot Description Format) is an XML-based format used to describe robot models in ROS. It defines the physical and visual properties of a robot, including its links (rigid bodies), joints (connections between links), and other properties like inertial parameters and visual representations.

## URDF Structure Overview

A URDF file describes a robot as a collection of rigid bodies (links) connected by joints. This creates a kinematic tree structure that represents the robot's mechanical design.

### Basic URDF Elements

#### Robot Element
The root element of every URDF file:

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Robot description goes here -->
</robot>
```

#### Link Element
Represents a rigid body in the robot:

```xml
<link name="base_link">
  <!-- Link properties go here -->
</link>
```

#### Joint Element
Connects two links:

```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <!-- Joint properties go here -->
</joint>
```

## Links: The Building Blocks

### Link Definition

A link represents a rigid body with physical and visual properties:

```xml
<link name="link_name">
  <!-- Inertial properties for physics simulation -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>

  <!-- Visual properties for rendering -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>

  <!-- Collision properties for physics simulation -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.1" radius="0.05"/>
    </geometry>
  </collision>
</link>
```

### Link Components

#### Inertial Properties
- **mass**: The mass of the link in kilograms
- **inertia**: The 3x3 inertia matrix describing how mass is distributed
- **origin**: The pose of the inertial reference frame relative to the link frame

#### Visual Properties
- **geometry**: The shape of the link for visualization
- **material**: The color and appearance properties
- **origin**: The pose of the visual reference frame relative to the link frame

#### Collision Properties
- **geometry**: The shape used for collision detection
- **origin**: The pose of the collision reference frame relative to the link frame

## Joints: Connecting the Links

### Joint Types

#### Fixed Joint
A joint with no degrees of freedom (0 DOF):

```xml
<joint name="fixed_joint" type="fixed">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>
```

#### Revolute Joint
A joint with 1 rotational degree of freedom:

```xml
<joint name="revolute_joint" type="revolute">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
</joint>
```

#### Continuous Joint
Like revolute but with unlimited rotation:

```xml
<joint name="continuous_joint" type="continuous">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit effort="10.0" velocity="1.0"/>
</joint>
```

#### Prismatic Joint
A joint with 1 translational degree of freedom:

```xml
<joint name="prismatic_joint" type="prismatic">
  <parent link="parent_link"/>
  <child link="child_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0.0" upper="0.1" effort="10.0" velocity="1.0"/>
</joint>
```

### Joint Components

#### Origin
Specifies the pose of the child link relative to the parent link when the joint is at zero position.

#### Axis
Defines the axis of rotation or translation for the joint (only for revolute, continuous, prismatic, and planar joints).

#### Limits
Defines the range of motion for the joint (only for revolute and prismatic joints):
- **lower**: Lower limit of joint motion
- **upper**: Upper limit of joint motion
- **effort**: Maximum effort the joint can exert
- **velocity**: Maximum velocity of the joint

## URDF Geometry Types

### Primitive Shapes

#### Box
```xml
<geometry>
  <box size="0.1 0.2 0.3"/>  <!-- width, depth, height -->
</geometry>
```

#### Cylinder
```xml
<geometry>
  <cylinder radius="0.05" length="0.1"/>
</geometry>
```

#### Sphere
```xml
<geometry>
  <sphere radius="0.05"/>
</geometry>
```

### Mesh Geometry
For complex shapes, you can reference external mesh files:

```xml
<geometry>
  <mesh filename="package://my_robot/meshes/link_mesh.dae" scale="1 1 1"/>
</geometry>
```

## Materials and Colors

Define materials for visualization:

```xml
<material name="red">
  <color rgba="1 0 0 1"/>  <!-- Red, Green, Blue, Alpha -->
</material>

<material name="green">
  <color rgba="0 1 0 1"/>
</material>

<material name="blue">
  <color rgba="0 0 1 1"/>
</material>
```

## Complete Example: Simple Robot Arm

Here's a complete example of a simple 2-DOF robot arm:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- First link -->
  <link name="link1">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.3"/>
      </geometry>
    </collision>
  </link>

  <!-- Second link -->
  <link name="link2">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- Joint between base and first link -->
  <joint name="joint1" type="revolute">
    <parent link="base_link"/>
    <child link="link1"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>

  <!-- Joint between first and second link -->
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10.0" velocity="1.0"/>
  </joint>
</robot>
```

## Xacro: URDF Macros

For complex robots, URDF can become very verbose. Xacro is a macro language that allows you to define reusable components:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="arm_with_xacro">
  <!-- Define a macro for a simple link -->
  <xacro:macro name="simple_link" params="name length radius color">
    <link name="${name}">
      <visual>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
        <material name="${color}"/>
      </visual>
      <collision>
        <geometry>
          <cylinder length="${length}" radius="${radius}"/>
        </geometry>
      </collision>
    </link>
  </xacro:macro>

  <!-- Use the macro -->
  <xacro:simple_link name="base" length="0.1" radius="0.05" color="blue"/>
</robot>
```

## URDF Best Practices

1. **Use Meaningful Names**: Choose clear, descriptive names for links and joints
2. **Consistent Units**: Use meters for length, kilograms for mass
3. **Valid Kinematic Tree**: Ensure the robot forms a valid tree structure (no loops)
4. **Proper Inertial Properties**: Include realistic inertial properties for simulation
5. **Collision vs Visual**: Use simpler geometries for collision to improve performance
6. **Use Xacro for Complex Robots**: Simplify complex URDFs with Xacro macros
7. **Validate Your URDF**: Use tools like `check_urdf` to validate your URDF files

## Common URDF Tools

- **RViz**: Visualize URDF models in ROS
- **URDF Parser**: Tools to parse and validate URDF files
- **Robot State Publisher**: Publishes joint states to transform robot model
- **Joint State Publisher**: Publishes joint states for visualization

Understanding URDF fundamentals is crucial for working with robot models in ROS. It provides the foundation for robot simulation, visualization, and control in humanoid robotics applications.