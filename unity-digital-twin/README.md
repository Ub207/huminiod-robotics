# Unity Digital Twin for Huminoid Robot

This Unity project implements the digital twin for the Huminoid humanoid robot, providing high-quality visualization and simulation capabilities that complement the Gazebo physics simulation.

## Project Structure

- `Assets/` - Unity assets (models, materials, scripts, scenes)
  - `HuminoidRobot/` - Robot model and components
  - `Scenes/` - Unity scenes (environments)
  - `Scripts/` - C# scripts for robot control and ROS communication
  - `Materials/` - Visual materials for robot and environment
  - `Prefabs/` - Reusable robot and environment components

## Prerequisites

- Unity 2022.3 LTS or later
- ROS-TCP-Connector package
- Running ROS 2 Humble system with Huminoid packages

## Setup Instructions

### 1. Install Required Packages

1. Open this project in Unity 2022.3 LTS
2. Open Package Manager (Window > Package Manager)
3. Install ROS-TCP-Connector from Git URL:
   - Click "+" > "Add package from git URL..."
   - Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git`

### 2. Robot Model Setup

The huminoid robot model should be imported with proper joint configurations to match the URDF model from the ROS system:

```
Huminoid (Root)
├── Torso
├── Head
├── Left Arm
│   ├── Upper Arm
│   └── Lower Arm
├── Right Arm
│   ├── Upper Arm
│   └── Lower Arm
├── Left Leg
│   ├── Upper Leg
│   └── Lower Leg
└── Right Leg
    ├── Upper Leg
    └── Lower Leg
```

### 3. ROS Communication Scripts

Key scripts for ROS communication:

- `ROSMasterConnector.cs` - Main connection to ROS master
- `JointStatePublisher.cs` - Publish joint states to ROS
- `JointStateSubscriber.cs` - Subscribe to joint commands from ROS
- `SensorPublisher.cs` - Publish simulated sensor data to ROS
- `DigitalTwinSynchronizer.cs` - Keep Unity and Gazebo in sync

## Key Features

### 1. Real-time Synchronization
- Unity robot position and state synchronized with Gazebo simulation
- Latency-optimized communication for real-time control

### 2. Enhanced Visualization
- High-quality rendering with realistic lighting
- Material simulation for realistic appearance
- Particle effects for interaction feedback

### 3. Human-Robot Interaction
- VR/AR support for immersive interaction
- Gesture recognition simulation
- Natural interaction points visualization

## Running the Simulation

1. Make sure ROS 2 system is running with huminoid packages:
   ```bash
   # In terminal 1
   ros2 launch huminoid_bringup gazebo_simulation.launch.py

   # In terminal 2
   ros2 run huminoid_ai ai_decision_node
   ```

2. Start the ROS TCP server bridge:
   ```bash
   ros2 run rosbridge_server rosbridge_websocket
   ```

3. Open this Unity project and run the main scene
4. Configure the IP address in `ROSMasterConnector.cs` to match your ROS system
5. Press Play to start the Unity simulation

## Configuration

### Network Settings
Update `ROSMasterConnector.cs` with the correct IP address:
```csharp
public string rosIpAddress = "127.0.0.1";  // Change to your ROS system IP
public int rosPort = 9090;  // Default rosbridge port
```

### Robot Parameters
The Unity robot should match the physical parameters from the URDF:
- Link masses and inertias
- Joint limits and dynamics
- Visual and collision geometries

## Safety and Ethics Considerations

This digital twin implements the same safety protocols as the physical system:
- Emergency stop functionality
- Collision detection and avoidance
- Safe human interaction zones
- Privacy-preserving perception

## Development Notes

### Adding New Sensors
To add new sensor simulation:
1. Create a new sensor publisher script extending the base class
2. Implement sensor data generation logic
3. Register the sensor with the ROS connector
4. Add to the scene and configure parameters

### Performance Optimization
- Use object pooling for frequently created/destroyed objects
- Implement Level of Detail (LOD) for distant objects
- Optimize materials and shaders for real-time rendering
- Use occlusion culling where appropriate

## Troubleshooting

### Common Issues
- Network connection problems: Check firewall settings and IP addresses
- Synchronization issues: Verify that both systems use the same time source
- Visual artifacts: Check that materials and scales match between systems

## Integration with Documentation

This Unity project is documented in the Physical AI & Humanoid Robotics textbook, specifically in Module 2: The Digital Twin (Gazebo & Unity). Refer to the documentation for implementation details and educational context.