---
sidebar_position: 3
---

# Unity Integration

## Why Unity for Robotics?

While Gazebo excels at physics simulation, Unity provides:

1. **High-quality graphics**: Realistic rendering for better visualization
2. **Interactive environments**: More engaging human-robot interaction
3. **VR/AR support**: Immersive robot programming and monitoring
4. **Large asset library**: Pre-built environments and objects
5. **User-friendly interface**: Easier for non-roboticists to interact with

## Unity-Ros Bridge

The Unity-Ros bridge enables communication between Unity and ROS 2. This allows:
- Publishing Unity sensor data to ROS topics
- Subscribing to ROS topics to control Unity robots
- Synchronizing simulation state between both environments

## Setting Up Unity for Robotics

### 1. Install Unity
- Download Unity Hub from unity.com
- Install Unity 2022.3 LTS
- Install Visual Studio or Rider for scripting

### 2. Install ROS-TCP-Connector
The ROS-TCP-Connector package allows Unity to communicate with ROS:
- Open Unity Package Manager (Window > Package Manager)
- Add package from git URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git`

### 3. Create a Robot in Unity

Here's how to create a simple humanoid robot in Unity:

1. Create a new 3D project
2. Create a hierarchy of GameObjects that represent your robot parts:
   - Robot (Root)
     - Torso
     - Head
     - Left Arm
       - Upper Arm
       - Lower Arm
     - Right Arm
       - Upper Arm
       - Lower Arm
     - Left Leg
       - Upper Leg
       - Lower Leg
     - Right Leg
       - Upper Leg
       - Lower Leg

3. Add joints to constrain movement:
   ```csharp
   // Example: Adding a hinge joint for a robot arm
   public class RobotArmController : MonoBehaviour
   {
       public HingeJoint joint;
       public float targetAngle = 0f;
       
       void Start()
       {
           joint = GetComponent<HingeJoint>();
       }
       
       void Update()
       {
           JointSpring spring = joint.spring;
           spring.targetPosition = targetAngle;
           joint.spring = spring;
       }
   }
   ```

## Unity-Ros Communication

### Publishing Data from Unity to ROS

To publish sensor data from Unity to ROS:

```csharp
using System.Collections;
using ROSBridgeLib;
using ROSBridgeLib.std_msgs;
using ROSBridgeLib.sensor_msgs;
using ROSBridgeLib.geometry_msgs;
using UnityEngine;

public class UnitySensorPublisher : MonoBehaviour 
{
    private ROSBridgeWebSocketConnection ros = null;
    
    void Start () 
    {
        // Connect to ROS
        ros = new ROSBridgeWebSocketConnection ("ws://192.168.1.1", 9090); // Replace with your ROS IP
        ros.OnConnected += OnConnected;
        ros.Connect ();
    }
    
    void Update () 
    {
        if (ros != null && ros.IsConnected ()) 
        {
            // Publish position
            Vector3 pos = transform.position;
            PoseMsg pose = new PoseMsg(
                new geometry_msgs.PointMsg(pos.x, pos.y, pos.z),
                new geometry_msgs.QuaternionMsg(0, 0, 0, 1)  // Simplified
            );
            
            ros.Publish(UnityPublisher.GetName(), new PoseMsg(pose));
        }
    }
    
    void OnConnected()
    {
        Debug.Log("Connected to ROS");
    }
    
    void OnApplicationQuit() 
    {
        if (ros != null)
            ros.Disconnect();
    }
}
```

### Subscribing to ROS Topics in Unity

To receive commands from ROS in Unity:

```csharp
using ROSBridgeLib;
using ROSBridgeLib.std_msgs;

public class UnityCommandSubscriber : MonoBehaviour 
{
    private ROSBridgeWebSocketConnection ros = null;
    
    void Start () 
    {
        ros = new ROSBridgeWebSocketConnection ("ws://192.168.1.1", 9090);
        ros.OnConnected += OnConnected;
        ros.Connect ();
    }
    
    void OnConnected()
    {
        // Subscribe to joint commands
        ros.Subscribe(UnitySubscriber.GetName(), JointStateCallback);
    }
    
    void JointStateCallback(ROSBridgeMsg msg)
    {
        // Process joint state message
        // Update robot joint positions accordingly
        Debug.Log("Received joint state: " + msg.ToString());
    }
    
    void OnApplicationQuit() 
    {
        if (ros != null)
            ros.Disconnect();
    }
}
```

## Creating a Digital Twin Environment

### 1. Environment Setup
Create a Unity scene that mirrors your Gazebo environment:
- Recreate key objects and obstacles
- Match lighting conditions
- Ensure similar scale and proportions

### 2. Synchronization Layer
Develop a synchronization system to keep both environments consistent:
- Object positions
- Robot states
- Sensor readings
- Physics properties

### 3. Visualization Enhancements
Take advantage of Unity's capabilities:
- Enhanced graphics and lighting
- Particle effects for visualization
- Custom UI overlays
- Virtual reality support

## Practical Unity-Ros Implementation

Let's create a simple Unity script that implements a basic robot controller:

```csharp
using System.Collections.Generic;
using ROSBridgeLib;
using ROSBridgeLib.geometry_msgs;
using ROSBridgeLib.sensor_msgs;
using UnityEngine;

public class HumanoidController : MonoBehaviour
{
    public GameObject head;
    public GameObject leftArm;
    public GameObject rightArm;
    public GameObject leftLeg;
    public GameObject rightLeg;
    
    public float moveSpeed = 2.0f;
    public float rotateSpeed = 50.0f;

    private ROSBridgeWebSocketConnection ros = null;
    
    void Start()
    {
        ros = new ROSBridgeWebSocketConnection("ws://localhost", 9090);
        ros.OnConnected += OnConnected;
        ros.Connect();
    }

    void OnConnected()
    {
        // Subscribe to ROS topics
        ros.Subscribe("cmd_vel", CmdVelCallback);
        ros.Subscribe("joint_commands", JointCommandCallback);
    }

    void CmdVelCallback(ROSBridgeMsg msg)
    {
        // Parse velocity command
        TwistMsg twist = (TwistMsg)msg;
        Vector3 linear = twist.GetLinear();
        Vector3 angular = twist.GetAngular();
        
        // Apply movement to robot
        transform.Translate(new Vector3(linear.x, 0, linear.z) * Time.deltaTime * moveSpeed);
        transform.Rotate(Vector3.up, angular.y * Time.deltaTime * rotateSpeed);
    }

    void JointCommandCallback(ROSBridgeMsg msg)
    {
        // Process joint commands
        // Update joint positions accordingly
        Debug.Log("Processing joint commands");
    }

    void Update()
    {
        if (ros != null && ros.IsConnected())
        {
            // Publish sensor data periodically
            PublishSensorData();
        }
    }

    void PublishSensorData()
    {
        // Create and publish sensor data
        // Example: Publish robot position
        PoseMsg pose = new PoseMsg(
            new PointMsg(transform.position.x, transform.position.y, transform.position.z),
            new QuaternionMsg(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        );
        
        ros.Publish("robot_pose", pose);
    }

    void OnApplicationQuit()
    {
        if (ros != null)
            ros.Disconnect();
    }
}
```

## Best Practices for Unity Integration

### 1. Performance Optimization
- Use efficient collision detection
- Optimize graphics settings for real-time simulation
- Use object pooling for frequently created/destroyed objects

### 2. Safety Considerations
- Implement bounds checking to prevent robots from leaving safe areas
- Add emergency stop functionality
- Visualize safety zones clearly

### 3. Consistency with Gazebo
- Ensure similar physics parameters between both simulators
- Maintain consistent coordinate systems
- Synchronize scene elements when possible

## Practical Exercise

1. Create a simple humanoid robot in Unity with articulated joints
2. Set up the ROS-TCP-Connector to communicate with ROS 2
3. Implement basic movement controls from ROS topics
4. Add a camera sensor that publishes images to ROS
5. Create a simple environment to test robot navigation

## Next Steps

After implementing Unity integration, we'll dive deeper into physics simulation concepts to ensure accurate behavior in both Gazebo and Unity environments.