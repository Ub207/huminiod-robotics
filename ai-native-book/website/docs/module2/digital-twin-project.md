---
sidebar_position: 5
---

# Module 2 Project: Create a Digital Twin of a Humanoid Robot in a Virtual Space

## Project Overview

In this capstone project for Module 2, you'll build a comprehensive digital twin environment that combines Gazebo's physics accuracy with Unity's visualization capabilities. This system will mirror the physical reality of a humanoid robot and its environment.

## Learning Objectives

By completing this project, you will:
- Build a complete virtual testing room with realistic physics
- Simulate robot walking and sensing in both Gazebo and Unity
- Create a synchronization system between simulation environments
- Observe real-time AI + physics interactions
- Validate simulation results against physical principles

## Project Requirements

### Core Components
Your digital twin system must include:
1. **Gazebo Environment** - Physics-accurate simulation with gravity, friction, and collision
2. **Unity Environment** - Visually rich environment with realistic rendering
3. **Synchronization Layer** - System to keep both environments consistent
4. **Sensor Simulation** - Implementation of various sensors (LiDAR, cameras, IMU, etc.)
5. **Control Interface** - Ability to send commands and receive feedback from both systems

### Validation Requirements
- Demonstrate stable robot locomotion in both environments
- Show consistent sensor readings between simulators
- Validate physics behavior against real-world principles
- Demonstrate AI agent interaction with simulated sensors

## System Architecture

```
                    +------------------+
                    |   AI Agent       |
                    |   (Decision)     |
                    +--------+---------+
                             |
                    +--------v---------+
                    |  Command Bridge  |
                    |  (ROS + TCP)     |
                    +--------+---------+
                             |
        +--------------------+--------------------+
        |                                       |
+-------v--------+                   +----------v---------+
|   Gazebo       |                   |     Unity          |
|   Simulation   |<----------------->|   Visualization    |
|   (Physics)    |   Sync Layer      |   (Graphics)       |
+----------------+                   +--------------------+
| - Accurate     |                   | - Realistic        |
|   physics      |                   |   rendering        |
| - Gravity,     |                   | - Interactive UI   |
|   collision    |                   | - VR/AR support    |
| - Sensor       |                   | - Enhanced         |
|   simulation   |                   |   visualization    |
+----------------+                   +--------------------+
```

## Implementation Steps

### Step 1: Set Up Gazebo Environment

1. Create a world file with your humanoid robot and testing environment:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="digital_twin_world">
    <!-- Physics properties -->
    <physics name="default_physics" type="ode">
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.000001</cfm>
          <erp>0.2</erp>
        </constraints>
      </ode>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Your humanoid robot -->
    <include>
      <uri>model://huminoid</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>

    <!-- Testing objects -->
    <model name="test_table">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="table_link">
        <collision name="collision">
          <geometry>
            <box><size>1.5 0.8 0.8</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>1.5 0.8 0.8</size></box>
          </geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient></material>
        </visual>
        <inertial>
          <mass>50.0</mass>
          <inertia><ixx>1.0</ixx><iyy>1.0</iyy><izz>1.0</izz></inertia>
        </inertial>
      </link>
    </model>

    <!-- Walking path markers -->
    <model name="path_marker_1">
      <pose>-1 0 0.05 0 0 0</pose>
      <link name="marker_link">
        <visual name="visual">
          <geometry><cylinder><radius>0.1</radius><length>0.1</length></cylinder></geometry>
          <material><ambient>0 1 0 1</ambient></material>
        </visual>
      </link>
    </model>

    <model name="path_marker_2">
      <pose>1 1 0.05 0 0 0</pose>
      <link name="marker_link">
        <visual name="visual">
          <geometry><cylinder><radius>0.1</radius><length>0.1</length></cylinder></geometry>
          <material><ambient>1 1 0 1</ambient></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

2. Ensure your robot URDF has proper Gazebo plugins for simulation:

```xml
<!-- In your robot URDF -->
<gazebo>
  <!-- Joint state publisher -->
  <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <robot_param>robot_description</robot_param>
    <robot_param_odom>robot_description</robot_param_odom>
    <joint_name>neck_joint</joint_name>
    <joint_name>left_shoulder_joint</joint_name>
    <joint_name>left_elbow_joint</joint_name>
    <joint_name>right_shoulder_joint</joint_name>
    <joint_name>right_elbow_joint</joint_name>
    <joint_name>left_hip_joint</joint_name>
    <joint_name>left_knee_joint</joint_name>
    <joint_name>left_ankle_joint</joint_name>
    <joint_name>right_hip_joint</joint_name>
    <joint_name>right_knee_joint</joint_name>
    <joint_name>right_ankle_joint</joint_name>
  </plugin>
</gazebo>

<!-- Add controllers for walking -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/huminoid</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  </plugin>
</gazebo>
```

### Step 2: Create Unity Environment

1. Set up the Unity project with the ROS-TCP-Connector
2. Recreate the environment from Gazebo with similar objects
3. Create your humanoid robot model in Unity with articulated joints
4. Implement the synchronization script:

```csharp
using System.Collections.Generic;
using ROSBridgeLib;
using ROSBridgeLib.geometry_msgs;
using ROSBridgeLib.sensor_msgs;
using UnityEngine;

public class DigitalTwinSynchronizer : MonoBehaviour
{
    public GameObject gazeboRobot;  // Visual representation of Gazebo robot
    public GameObject unityRobot;   // Visual representation in Unity
    
    public float syncRate = 30.0f;  // Sync 30 times per second
    
    private ROSBridgeWebSocketConnection ros = null;
    private Dictionary<string, Transform> gazeboLinks = new Dictionary<string, Transform>();
    private Dictionary<string, Transform> unityLinks = new Dictionary<string, Transform>();
    
    void Start()
    {
        // Initialize dictionaries with robot links
        InitializeRobotLinks();
        
        // Connect to ROS
        ros = new ROSBridgeWebSocketConnection("ws://localhost", 9090);
        ros.OnConnected += OnConnected;
        ros.Connect();
        
        // Start synchronization coroutine
        StartCoroutine(SynchronizationLoop());
    }

    void InitializeRobotLinks()
    {
        // Populate the dictionaries with references to robot parts
        foreach(Transform child in gazeboRobot.transform)
        {
            gazeboLinks.Add(child.name, child);
        }
        
        foreach(Transform child in unityRobot.transform)
        {
            unityLinks.Add(child.name, child);
        }
    }

    void OnConnected()
    {
        // Subscribe to necessary topics for sync
        ros.Subscribe("/huminoid/joint_states", JointStateCallback);
    }

    void JointStateCallback(ROSBridgeMsg msg)
    {
        // Update Unity robot based on Gazebo joint states
        JointStateMsg jointState = (JointStateMsg)msg;
        List<string> names = jointState.GetName();
        List<float> positions = jointState.GetPosition();
        
        for(int i = 0; i < names.Count; i++)
        {
            string jointName = names[i];
            float position = positions[i];
            
            // Update corresponding Unity joint
            if(unityLinks.ContainsKey(jointName))
            {
                // Apply rotation based on joint position
                // (This is a simplified example - real implementation depends on joint type)
                unityLinks[jointName].localRotation = Quaternion.Euler(0, position * Mathf.Rad2Deg, 0);
            }
        }
    }

    System.Collections.IEnumerator SynchronizationLoop()
    {
        while(true)
        {
            // Send Unity robot state to Gazebo
            SyncUnityToGazebo();
            
            // Wait for next sync interval
            yield return new WaitForSeconds(1.0f / syncRate);
        }
    }

    void SyncUnityToGazebo()
    {
        // Publish Unity robot state to ROS for Gazebo
        JointStateMsg jointState = new JointStateMsg();
        jointState.SetName(new List<string>());
        jointState.SetPosition(new List<float>());
        jointState.SetHeader(new HeaderMsg(0, new TimeMsg(), "unity_frame"));
        
        // Collect joint states from Unity robot
        foreach(KeyValuePair<string, Transform> pair in unityLinks)
        {
            jointState.GetName().Add(pair.Key);
            // Convert rotation to angle (simplified)
            float angle = pair.Value.localRotation.eulerAngles.y * Mathf.Deg2Rad;
            jointState.GetPosition().Add(angle);
        }
        
        ros.Publish("/unity_joint_states", jointState);
    }

    void OnDestroy()
    {
        if (ros != null)
            ros.Disconnect();
    }
}
```

### Step 3: Implement Sensor Simulation

Add sensor simulation in both environments:

Gazebo sensor plugins:
```xml
<!-- LiDAR on robot head -->
<gazebo reference="head">
  <sensor type="ray" name="lidar_sensor">
    <pose>0.1 0 0 0 0 0</pose>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>30.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/huminoid</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
    </plugin>
  </sensor>
</gazebo>

<!-- RGB-D camera -->
<gazebo reference="head_camera_frame">
  <sensor type="depth" name="head_camera">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.089</horizontal_fov>
      <image>
        <format>R8G8B8</format>
        <width>640</width>
        <height>480</height>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_openni_kinect.so">
      <baseline>0.2</baseline>
      <alwaysOn>true</alwaysOn>
      <updateRate>30.0</updateRate>
      <cameraName>head_camera</cameraName>
      <imageTopicName>/huminoid/head_camera/rgb/image_raw</imageTopicName>
      <depthImageTopicName>/huminoid/head_camera/depth/image_raw</depthImageTopicName>
      <pointCloudTopicName>/huminoid/head_camera/depth/points</pointCloudTopicName>
      <cameraInfoTopicName>/huminoid/head_camera/rgb/camera_info</cameraInfoTopicName>
      <depthImageCameraInfoTopicName>/huminoid/head_camera/depth/camera_info</depthImageCameraInfoTopicName>
      <frameName>head_camera_optical_frame</frameName>
      <pointCloudCutoff>0.5</pointCloudCutoff>
      <pointCloudCutoffMax>3.0</pointCloudCutoffMax>
      <distortion_k1>0.0</distortion_k1>
      <distortion_k2>0.0</distortion_k2>
      <distortion_k3>0.0</distortion_k3>
      <distortion_t1>0.0</distortion_t1>
      <distortion_t2>0.0</distortion_t2>
    </plugin>
  </sensor>
</gazebo>
```

### Step 4: AI Integration in Simulation

Create a test script that demonstrates AI interacting with the simulated sensors:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import cv2
from cv2 import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DigitalTwinAIAgent(Node):
    def __init__(self):
        super().__init__('digital_twin_ai_agent')
        
        # Create subscriber for simulated sensors
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/huminoid/scan',
            self.lidar_callback,
            10
        )
        
        self.camera_subscriber = self.create_subscription(
            Image,
            '/huminoid/head_camera/rgb/image_raw',
            self.camera_callback,
            10
        )
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/huminoid/imu/data',
            self.imu_callback,
            10
        )
        
        # Create publisher for robot commands
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/huminoid/cmd_vel',
            10
        )
        
        # Timer for decision making
        self.timer = self.create_timer(0.5, self.make_decision)
        
        # Sensor data storage
        self.lidar_data = None
        self.camera_data = None
        self.imu_data = None
        
        self.bridge = CvBridge()
        
        self.get_logger().info('Digital Twin AI Agent initialized')

    def lidar_callback(self, msg):
        self.lidar_data = np.array(msg.ranges)
        self.get_logger().info(f'Received LiDAR data with {len(msg.ranges)} points')

    def camera_callback(self, msg):
        # Convert ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.camera_data = cv_image
            self.get_logger().info(f'Received camera image: {cv_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def imu_callback(self, msg):
        self.imu_data = msg
        self.get_logger().info(f'Received IMU data')

    def make_decision(self):
        cmd = Twist()
        
        if self.lidar_data is not None:
            # Simple obstacle avoidance based on LiDAR
            front_distances = self.lidar_data[340:380]  # Front 40 degrees
            min_distance = np.min(front_distances)
            
            if min_distance < 1.0:  # Obstacle within 1 meter
                cmd.linear.x = 0.0  # Stop
                cmd.angular.z = 0.5  # Turn right
                self.get_logger().info('Obstacle detected, turning right')
            else:
                cmd.linear.x = 0.5  # Move forward
                cmd.angular.z = 0.0  # No turn
                self.get_logger().info('Path clear, moving forward')
        else:
            # Default behavior if no sensor data
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('No sensor data available')
        
        # Publish command
        self.cmd_publisher.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    ai_agent = DigitalTwinAIAgent()
    
    try:
        rclpy.spin(ai_agent)
    except KeyboardInterrupt:
        pass
    
    ai_agent.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 5: Launch and Test the System

1. Launch Gazebo with your world:
```bash
ros2 launch gazebo_ros empty_world.launch.py world:=/path/to/your/digital_twin_world.sdf
```

2. Launch your robot state publisher:
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:='$(cat /path/to/your/huminoid.urdf)'
```

3. Run the AI agent:
```bash
ros2 run your_package digital_twin_ai_agent
```

4. Start the Unity simulation with ROS connection
5. Observe the synchronized behavior between both environments

## Testing and Validation

### 1. Physics Validation
- Verify that gravity affects both robots similarly
- Test collision responses in both environments
- Confirm that friction and contact forces behave consistently

### 2. Sensor Validation
- Compare LiDAR readings between simulators
- Validate camera images show similar content
- Ensure IMU readings reflect the same physical state

### 3. Control Validation
- Send the same commands to both systems
- Verify that responses are similar
- Test complex behaviors like walking or manipulation

## Advanced Challenges

For students seeking additional challenges:
1. Implement a more complex walking gait using trajectory planning
2. Add haptic feedback in Unity based on Gazebo contact sensors
3. Create AI training scenarios that leverage both simulation environments
4. Implement safety systems that work across both simulators

## Next Steps

After completing this digital twin project, you'll have built a comprehensive simulation system that bridges physics accuracy with visual realism. This foundation enables safe testing of complex robotic behaviors before deployment to physical hardware.

In the next module, we'll explore AI agent integration in more depth, connecting cognitive capabilities with both simulated and real robotic systems.