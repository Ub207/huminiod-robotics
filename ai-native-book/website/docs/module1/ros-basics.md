---
sidebar_position: 2
---

# ROS 2 Basics

## What is ROS 2?

ROS 2 (Robot Operating System 2) is not an operating system but a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

## Why ROS 2 for Humanoid Robotics?

For humanoid robots, ROS 2 provides several critical capabilities:

1. **Modularity**: Different robot functions can be implemented as separate nodes and run on different machines
2. **Communication**: Standardized message passing between components
3. **Hardware Abstraction**: Same code works with different hardware through standard interfaces
4. **Rich Ecosystem**: Hundreds of packages for perception, navigation, manipulation, etc.

## Core Architecture

### Nodes
A node is an executable that uses ROS 2 to communicate with other nodes. In a humanoid robot, nodes might include:
- Camera driver node
- Joint controller node
- Path planner node
- Vision processing node
- AI decision-making node

### Communication Patterns
ROS 2 supports three main communication patterns:
1. **Topics** (publish/subscribe) - for streaming data
2. **Services** (request/response) - for single requests
3. **Actions** (goal/feedback/result) - for long-running tasks with feedback

## Key Improvements in ROS 2 over ROS 1

- **Real-time support**: Deterministic behavior for safety-critical applications
- **Multi-robot systems**: Better support for multi-robot scenarios
- **Security**: Built-in security features
- **Middleware abstraction**: Pluggable communication backends (DDS implementations)
- **Official Windows and macOS support**: No more Linux-only requirement

## Setting up Your First ROS 2 Workspace

Let's create a workspace for our humanoid robot project:

```bash
# Create workspace directory
mkdir -p ~/huminiod_ws/src
cd ~/huminiod_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Understanding the DDS Layer

ROS 2 uses Data Distribution Service (DDS) as its middleware. DDS is a proven, mature technology for real-time, high-performance, interoperable, scalable, fault-tolerant and distributed systems. Understanding DDS is important for:

- Performance optimization
- Network configuration
- Security setup
- Multi-robot communication

## Quality of Service (QoS) Settings

In ROS 2, Quality of Service (QoS) settings allow you to specify policies for message delivery. For humanoid robots, important QoS settings include:

- **Reliability**: Whether messages should be guaranteed delivery or best-effort
- **Durability**: Whether late-joining nodes should receive previous messages
- **History**: How many messages to store for retransmission

Example QoS configuration for sensor data (where older readings are not relevant):
```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# For sensor data where you only want the most recent values
sensor_qos = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST
)
```

## Next Steps

With a basic understanding of ROS 2 concepts, we'll now explore how to create nodes and establish communication through topics in the next section.