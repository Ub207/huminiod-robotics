---
sidebar_position: 2
---

# Gazebo Basics

## What is Gazebo?

Gazebo is a 3D dynamic simulator with accurate physics, realistic rendering, and robust plugin architecture. It's widely used in robotics research and development for testing algorithms, robot designs, and scenarios in a safe, reproducible environment.

## Why Gazebo for Humanoid Robotics?

For humanoid robots, Gazebo provides:

1. **Realistic Physics Simulation**: Accurate modeling of forces, gravity, collisions, and friction
2. **Sensor Simulation**: Emulation of various robot sensors (cameras, LiDAR, IMU, force/torque)
3. **Environment Creation**: Ability to build complex worlds for testing robot behaviors
4. **Hardware Integration**: Seamless connection with ROS through Gazebo ROS packages
5. **Safety**: Test dangerous scenarios without risk to hardware or humans

## Installing Gazebo

Gazebo typically comes with ROS 2 installation. If you need to install it separately:

```bash
sudo apt-get install ros-humble-gazebo-*
sudo apt-get install gazebo
```

## Basic Gazebo Concepts

### Worlds
A Gazebo "world" is an environment containing:
- Physics properties (gravity, magnetic field, etc.)
- Models (robots, objects, obstacles)
- Plugins (control, sensors, simulation enhancements)
- Lighting and visual effects

### Models
Gazebo models represent physical objects in the simulation. These can be:
- Robots (your humanoid)
- Obstacles and furniture
- Sensors
- Specialized objects (balls, blocks, etc.)

### Plugins
Gazebo uses plugins to customize behavior:
- **Sensor plugins**: Simulate real sensors
- **Controller plugins**: Interface with ROS topics/services
- **Physics plugins**: Modify physical behavior
- **GUI plugins**: Add custom interfaces

## Launching Gazebo with ROS 2

To work with Gazebo and ROS 2, use the `gazebo_ros` packages:

```bash
# Launch Gazebo with ROS 2 interface
ros2 launch gazebo_ros empty_world.launch.py

# Or with a specific world file
ros2 launch gazebo_ros empty_world.launch.py world:=/path/to/world.sdf
```

## Creating Your First Simulation

Let's create a simple world file that includes your humanoid robot:

```xml
<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="huminoid_world">
    <!-- Include the default ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Include the default sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Add your humanoid robot model -->
    <!-- Replace with your actual URDF/robot model -->
    <include>
      <uri>model://huminoid</uri>
      <pose>0 0 1 0 0 0</pose>
    </include>
    
    <!-- Add some obstacles for testing -->
    <model name="box1">
      <pose>2 2 0.5 0 0 0</pose>
      <link name="box1_link">
        <collision name="box1_collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="box1_visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.3 0.3 0.3 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.166667</iyy>
            <iyz>0</iyz>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Physics engine configuration -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
  </world>
</sdf>
```

## Integrating Your Robot URDF with Gazebo

To use your URDF robot model in Gazebo, you need to add Gazebo-specific tags. Here's an example of how to extend your URDF:

```xml
<!-- Inside your URDF file -->
<robot name="huminoid">
  <!-- Your existing URDF content -->
  
  <!-- Gazebo-specific extensions -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
  </gazebo>
  
  <!-- For each joint you want to control -->
  <gazebo>
    <plugin name="joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <joint_name>neck_joint</joint_name>
      <joint_name>left_shoulder_joint</joint_name>
      <joint_name>left_elbow_joint</joint_name>
      <!-- Add all your joint names -->
    </plugin>
  </gazebo>
  
  <!-- Controller plugin for joint position control -->
  <gazebo>
    <plugin name="position_controllers" filename="libgazebo_ros_joint_trajectory.so">
      <robot_namespace>/huminoid</robot_namespace>
    </plugin>
  </gazebo>
</robot>
```

## Launching Your Robot in Gazebo

Create a launch file to spawn your robot in Gazebo:

```python
# launch/huminoid_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
    )
    
    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'huminoid',
            '-x', '0',
            '-y', '0', 
            '-z', '1'
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[PathJoinSubstitution([
            FindPackageShare('huminoid_description'),
            'urdf',
            'huminoid.urdf'
        ])]
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
```

## Working with Sensors in Gazebo

To simulate sensors on your humanoid robot, add sensor plugins to your URDF:

```xml
<!-- Camera sensor -->
<gazebo reference="head_camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.089</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>head_camera_optical_frame</frame_name>
      <min_depth>0.1</min_depth>
      <max_depth>100.0</max_depth>
      <update_rate>30.0</update_rate>
    </plugin>
  </sensor>
</gazebo>

<!-- IMU sensor -->
<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <visualize>true</visualize>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>2e-4</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>1.7e-2</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/huminoid</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Practical Exercise

1. Create a simple world file with your humanoid robot
2. Add at least one sensor to your robot model
3. Launch the simulation and verify that you can see your robot in Gazebo
4. Verify that sensor data is being published to ROS topics

## Next Steps

After mastering Gazebo basics, we'll explore Unity integration for more advanced visualization and human-robot interaction scenarios.