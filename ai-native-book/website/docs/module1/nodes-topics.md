---
sidebar_position: 3
---

# Nodes and Topics

## Understanding Nodes

A node in ROS 2 is an executable that uses ROS 2 to communicate with other nodes. In a humanoid robot, nodes are the building blocks that perform specific functions, such as controlling joints, processing sensor data, or making high-level decisions.

### Creating a Basic ROS 2 Node

Let's create a simple node that could be part of our humanoid robot system. This node will publish sensor data:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = ["joint1", "joint2", "joint3"]  # Example joint names
        msg.position = [0.0, 0.5, 1.0]  # Example joint positions
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing joint states: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    rclpy.spin(sensor_publisher)
    sensor_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics: The Publish/Subscribe Pattern

Topics are the primary mechanism for data streaming in ROS 2. They use a publish/subscribe pattern where:

- Publishers send messages to a topic
- Subscribers receive messages from a topic
- Multiple publishers and subscribers can use the same topic

### Creating a Subscriber Node

Here's how we might create a subscriber that listens to joint states and performs some action:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received joint states: {msg.position}')

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    rclpy.spin(joint_state_subscriber)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Topic Patterns for Humanoid Robots

In humanoid robotics, we often need more sophisticated patterns:

### 1. Multiple Publishers, Single Subscriber
Different sensor nodes publishing to the same topic for sensor fusion (e.g., IMU, cameras, force sensors all contributing to state estimation).

### 2. Single Publisher, Multiple Subscribers
One control node sending commands to multiple actuator nodes.

### 3. Publisher-Subscriber Chains
Sensor data → Processing → Decision → Action → Feedback loop.

## Quality of Service (QoS) for Humanoid Applications

Different types of data in a humanoid robot require different QoS profiles:

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# For critical control commands where delivery is essential
control_qos = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST
)

# For sensor data where real-time delivery is more important than reliability
sensor_qos = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST
)

# Create publisher with specific QoS
self.control_publisher = self.create_publisher(
    JointState, 
    'joint_commands', 
    control_qos
)
```

## Topic Naming Conventions for Humanoid Robots

To maintain organization in complex humanoid systems, follow these conventions:

- `/robot_name/sensor_type/sensor_name` (e.g., `/huminoid/head_camera/rgb_image`)
- `/robot_name/control_type/joint_name` (e.g., `/huminoid/joint_commands/right_arm_shoulder`)
- `/robot_name/system_type/status` (e.g., `/huminoid/navigation/status`)

## Practical Exercise

Create a simple publisher-subscriber pair where:
1. A publisher node simulates reading from an IMU sensor
2. A subscriber node receives the IMU data and logs it to the console

This will give you hands-on experience with the fundamentals of ROS 2 communication that power humanoid robots.

## Next Steps

After mastering nodes and topics, we'll explore services and actions, which are essential for request/response communication in robotic systems.