---
sidebar_position: 5
---

# URDF: Robot Modeling and Description

## What is URDF?

URDF (Unified Robot Description Format) is an XML-based format that describes a robot's physical properties in ROS. It defines the robot's structure, joints, inertial properties, visual representation, and collision properties - everything needed to simulate or control a robot in ROS.

## URDF in Humanoid Robotics

For humanoid robots, URDF is crucial because it:
- Defines the robot's kinematic chain (how body parts connect)
- Specifies joint limits and types (revolute, prismatic, continuous)
- Provides visual and collision models for simulation
- Enables kinematic solvers to calculate forward and inverse kinematics

## Basic URDF Structure

A URDF file for a humanoid robot follows this general structure:

```xml
<?xml version="1.0"?>
<robot name="huminoid_robot">
  <!-- Define materials -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  
  <!-- Define links (rigid bodies) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.2 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Define joints connecting links -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  
  <!-- More links and joints... -->
</robot>
```

## Key Components of URDF

### Links
Links represent rigid bodies of the robot:

```xml
<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
    <material name="white">
      <color rgba="1.0 1.0 1.0 1.0"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
  </inertial>
</link>
```

### Joints
Joints connect links and define how they move relative to each other:

```xml
<!-- Revolute joint for the neck (rotates around Z-axis) -->
<joint name="neck_joint" type="revolute">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0.0 0.0 0.7" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>

<!-- Continuous joint for a wheel -->
<joint name="wheel_joint" type="continuous">
  <parent link="leg"/>
  <child link="wheel"/>
  <origin xyz="0.0 0.0 -0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

### Materials
Materials define visual appearance:

```xml
<material name="red">
  <color rgba="0.8 0.0 0.0 1.0"/>
</material>
```

## Complete Humanoid URDF Example

Here's a simplified example of a humanoid robot URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_huminoid">

  <!-- Define colors -->
  <material name="blue">
    <color rgba="0.0 0.0 0.8 1.0"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.0 0.0 1.0"/>
  </material>
  <material name="white">
    <color rgba="1.0 1.0 1.0 1.0"/>
  </material>

  <!-- Base/Root link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.2 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.416" ixy="0.0" ixz="0.0" iyy="1.041" iyz="0.0" izz="1.041"/>
    </inertial>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.208" ixy="0.0" ixz="0.0" iyy="0.312" iyz="0.0" izz="0.125"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.004" ixy="0.0" ixz="0.0" iyy="0.004" iyz="0.0" izz="0.004"/>
    </inertial>
  </link>

  <!-- Joints -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0.0 0.0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="100" velocity="1.0"/>
  </joint>

  <!-- Arms -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.0075" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0.0 0.2" rpy="0 0 1.57"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>

  <!-- Similar definitions for right arm, legs, etc. -->
</robot>
```

## Using URDF with ROS 2

To use a URDF in ROS 2, we typically publish it on a topic using a robot_state_publisher node:

```python
import rclpy
from rclpy.node import Node
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        # Load the URDF file
        with open('/path/to/huminoid.urdf', 'r') as f:
            robot_desc = f.read()
        
        # You would typically use the robot_state_publisher package
        # which handles publishing joint states to tf transforms
        self.get_logger().info('Robot description loaded')

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Xacro

For complex robots, URDF files can become very large and repetitive. Xacro (XML Macros) allows us to create more maintainable robot descriptions:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="huminoid_with_xacro">

  <!-- Define properties -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="base_width" value="0.5" />
  <xacro:property name="base_length" value="0.2" />
  <xacro:property name="base_height" value="0.1" />

  <!-- Macro for creating a wheel -->
  <xacro:macro name="wheel" params="prefix parent *origin">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="0.05" length="0.05"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.05" length="0.05"/>
        </geometry>
        <origin xyz="0 0 0" rpy="${M_PI/2} 0 0"/>
      </collision>
      <inertial>
        <mass value="0.2"/>
        <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0005" iyz="0.0" izz="0.001"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="${parent}"/>
      <child link="${prefix}_wheel"/>
      <xacro:insert_block name="origin"/>
      <axis xyz="0 1 0"/>
    </joint>
  </xacro:macro>

  <!-- Use the macro -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="${base_width} ${base_length} ${base_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <xacro:wheel prefix="front_left" parent="base_link">
    <origin xyz="${base_width/2} ${base_length/2} -${base_height/2}" rpy="0 0 0"/>
  </xacro:wheel>

</robot>
```

## Validation and Visualization

To validate your URDF:
```bash
# Check for syntax errors
check_urdf /path/to/robot.urdf

# Show robot information
urdf_to_graphiz /path/to/robot.urdf
```

To visualize your robot:
- Use RViz2 with RobotModel plugin
- Use Gazebo simulator
- Use standalone tools like `robot_state_publisher` with joint_state_publisher

## Practical Exercise

Create a URDF file for a simple 2-joint robot arm. The arm should have:
- A base link (fixed to world)
- An upper arm link connected with a revolute joint
- A lower arm link connected with another revolute joint
- Visual shapes for each link
- Proper inertial properties

## Next Steps

With our understanding of URDF, we'll now explore how to integrate AI agents with ROS, enabling intelligent behavior in our humanoid robots.