---
sidebar_position: 7
---

# Module 1 Project: Design a Basic Humanoid Nervous System in ROS 2

## Project Overview

In this capstone project for Module 1, you'll create a complete neural messaging system for a humanoid robot. You'll build a multi-node architecture that connects various robot subsystems with an AI agent, creating the "nervous system" of the humanoid.

## Learning Objectives

By completing this project, you will:
- Design a modular ROS 2 architecture for a humanoid robot
- Implement all three communication patterns (topics, services, actions)
- Connect an AI agent to control robot behavior
- Integrate sensor and actuator systems
- Test the system in simulation

## Project Requirements

### Core Architecture
Your system must include:
1. **Sensor Nodes** - Publishing data from robot sensors (at least 3 different types)
2. **Controller Nodes** - Managing robot actuators
3. **AI Decision Node** - Processing sensor data and making decisions (from previous section)
4. **Coordinator Node** - Managing high-level robot behavior
5. **Visualization Node** - Publishing data for monitoring

### Communication Requirements
- Use topics for streaming sensor data (e.g., camera feeds, joint states)
- Use services for configuration and calibration tasks
- Use actions for complex behaviors like walking or manipulation

### AI Integration
- The AI agent should process sensor data and provide high-level commands
- Implement a feedback loop where actions can be modified based on new sensor input
- Include a memory component to store and recall experiences

## System Architecture

```
     +------------------+     +------------------+
     |   AI Decision    |<--->|   Coordinator    |
     |      Node        |     |      Node        |
     +--------+---------+     +--------+---------+
              |                      |
              |                      |
              v                      v
    +--------+---------+    +--------+---------+
    |   Sensor Data    |    | Action Commands  |
    |   Processing     |    |   Processing     |
    +--------+---------+    +--------+---------+
             |                       |
             |                       |
    +--------v---------+    +--------v---------+
    |  Sensor Nodes    |    | Controller Nodes |
    |  (Cameras, IMU,  |    |  (Joints, Grips) |
    |   Joint States)  |    |                 |
    +------------------+    +------------------+
```

## Implementation Steps

### Step 1: Create the ROS 2 Workspace

First, create a workspace for your humanoid robot project:

```bash
mkdir -p ~/huminoid_ws/src
cd ~/huminoid_ws
```

### Step 2: Design Your URDF Model

Create a URDF file for your humanoid robot. Start simple and add complexity gradually:

```xml
<?xml version="1.0"?>
<robot name="huminoid">
  <!-- Add your URDF description here -->
  <!-- Include at least: base, torso, head, two arms, two legs -->
</robot>
```

### Step 3: Implement Core Nodes

Create the following node packages:

1. **Sensor Publisher Node** - Publishes sensor data:
```python
# sensor_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, Imu
import math

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu_data', 10)
        # Add more sensor publishers as needed
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_sensors)
        self.i = 0

    def publish_sensors(self):
        # Publish joint states with simulated data
        joint_msg = JointState()
        joint_msg.name = ['head_pan', 'head_tilt', 'left_shoulder', 'left_elbow', 'right_shoulder', 'right_elbow']
        joint_msg.position = [math.sin(self.i/10), math.cos(self.i/10), 0.5, 0.3, -0.5, -0.3]
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_pub.publish(joint_msg)
        
        # Publish IMU data with simulated values
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = 0.1 * math.sin(self.i/10)
        imu_msg.angular_velocity.z = 0.05 * math.cos(self.i/10)
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_pub.publish(imu_msg)
        
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

2. **Controller Node** - Receives commands and controls actuators:
```python
# controller_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

class JointGroupPositionController(Node):
    def __init__(self):
        super().__init__('joint_group_position_controller')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'joint_commands',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint commands: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    joint_group_position_controller = JointGroupPositionController()
    rclpy.spin(joint_group_position_controller)
    joint_group_position_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. **AI Decision Node** - Processes sensor data and makes decisions (you can adapt the code from the AI integration section):
```python
# ai_decision_node.py
# (Use the code from the AI-ROS integration section with modifications)
```

### Step 4: Integration and Testing

1. **Build your workspace**:
```bash
cd ~/huminoid_ws
colcon build --packages-select sensor_publisher controller_node ai_decision_node
source install/setup.bash
```

2. **Launch all nodes**:
```bash
# Terminal 1
ros2 run sensor_publisher sensor_publisher

# Terminal 2
ros2 run controller_node controller_node

# Terminal 3
ros2 run ai_decision_node ai_decision_node
```

3. **Monitor the system** with `rqt_graph` to visualize the node connections:
```bash
rqt_graph
```

### Step 5: Advanced Integration

For an additional challenge, implement:
- A behavior tree or finite state machine to coordinate robot behaviors
- An action server for complex tasks like "walk_to_location" or "pick_up_object"
- A service for emergency stop functionality

## Testing and Validation

Verify your system by:
1. Confirming all nodes are communicating properly
2. Checking that the AI agent is receiving sensor data and sending commands
3. Verifying that the controller is receiving commands from the AI
4. Testing the system's response to various scenarios

## Next Steps

After completing this project, you'll have built a complete ROS 2-based nervous system for a humanoid robot. In Module 2, we'll create simulation environments where you can test and refine this system with Gazebo and Unity.