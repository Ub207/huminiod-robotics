---
sidebar_position: 4
---

# Physics Simulation

## Understanding Physics in Robotics Simulation

Physics simulation is the backbone of realistic robotic simulation. It enables accurate modeling of forces, gravity, collisions, and material properties that determine how robots interact with their environment.

## Physics Concepts for Humanoid Robots

### 1. Gravity and Mass
Gravity is a constant downward force that affects all objects. For humanoid robots:
- Proper mass distribution is critical for balance and stability
- The center of mass must be controlled for stable locomotion
- Joint torques must counteract gravitational forces

### 2. Collision Detection and Response
Collision systems determine when and how objects interact:
- **Static collisions**: Robot with environment (walls, floors)
- **Dynamic collisions**: Robot with moving objects or other robots
- **Self-collision avoidance**: Preventing robot parts from colliding

### 3. Friction and Contact Forces
Friction is essential for humanoid locomotion:
- Static friction allows the robot to push off surfaces without slipping
- Kinetic friction affects sliding motions
- Surface properties affect grip and mobility

## Gazebo Physics

### Physics Engine Configuration
Gazebo uses ODE (Open Dynamics Engine) by default, though other engines like Bullet are available:

```xml
<physics name="default" type="ode">
  <gravity>0 0 -9.8</gravity>
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  <ode>
    <solver>
      <type>quick</type>
      <iters>10</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.000001</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

### Accurate Inertial Properties
For realistic simulation, every link must have accurate inertial properties:

```xml
<link name="thigh">
  <inertial>
    <!-- Mass in kg -->
    <mass value="2.5" />
    <!-- Inertia tensor -->
    <inertia 
      ixx="0.0131" 
      ixy="0.0" 
      ixz="0.0" 
      iyy="0.0131" 
      iyz="0.0" 
      izz="0.0026" />
  </inertial>
  <visual>
    <geometry>
      <cylinder length="0.4" radius="0.06"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.4" radius="0.06"/>
    </geometry>
  </collision>
</link>
```

### Joint Dynamics
For realistic joint behavior, include dynamic properties:

```xml
<joint name="knee_joint" type="revolute">
  <parent link="thigh"/>
  <child link="shin"/>
  <origin xyz="0 0 -0.2" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <!-- Joint limits -->
  <limit lower="-0.1" upper="2.0" effort="50.0" velocity="2.0"/>
  <!-- Dynamics for more realistic behavior -->
  <dynamics damping="1.0" friction="0.1"/>
</joint>
```

## Tuning Physics Parameters for Humanoid Robots

### 1. Stability vs. Responsiveness
- Higher damping values increase stability but reduce responsiveness
- Lower damping values make the robot more responsive but potentially unstable
- Find the right balance for your specific humanoid design

### 2. Simulation Accuracy
- Smaller step sizes increase accuracy but reduce performance
- Typical values: 0.001s for high accuracy, 0.01s for performance

### 3. Real-time Factor
- Real-time factor of 1.0 means simulation runs at the same speed as real-time
- Values less than 1.0 mean simulation runs slower than real-time
- Values greater than 1.0 mean simulation runs faster than real-time

## Collision Checking

### Self-Collision Avoidance
In URDF, you can disable collisions between links that are expected to be near each other:

```xml
<collision_checking>
  <!-- Disable collision between adjacent links -->
  <disable_collisions link1="torso" link2="head"/>
  <disable_collisions link1="upper_arm" link2="torso"/>
  <!-- But enable collision with environment -->
</collision_checking>
```

## Sensor Physics

### Force/Torque Sensors
For accurate manipulation, simulate sensor noise and dynamics:

```xml
<gazebo reference="wrist">
  <sensor name="ft_sensor" type="force_torque">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
    </force_torque>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </sensor>
</gazebo>
```

### Contact Sensors
To detect when the robot makes contact with objects:

```xml
<gazebo reference="foot">
  <sensor name="foot_contact" type="contact">
    <always_on>true</always_on>
    <update_rate>1000</update_rate>
    <contact>
      <collision>foot_collision</collision>
    </contact>
  </sensor>
</gazebo>
```

## Advanced Physics Concepts

### 1. Center of Mass (CoM)
Critical for humanoid balance:
- CoM must remain within the support polygon for stability
- Control strategies must account for CoM dynamics
- Changing payloads affect CoM position

### 2. Zero Moment Point (ZMP)
ZMP is crucial for walking stability in humanoid robots:
- Points where the net moment of ground reaction forces is zero
- Walking patterns are often planned around ZMP constraints
- Maintaining ZMP within the support foot is essential for stability

### 3. Impulse-based Contact
For more accurate collision handling, especially during walking:
- Models impacts as instantaneous momentum transfers
- Better for scenarios with high impact forces
- Critical for realistic walking and running simulation

## Unity Physics Integration

Unity's physics engine (NVIDIA PhysX) can be configured for robotics:

```csharp
public class HumanoidPhysicsController : MonoBehaviour
{
    public float gravityScale = 1.0f;
    public float mass = 70.0f; // Approximate humanoid mass
    
    private Rigidbody[] rigidbodies;
    
    void Start()
    {
        // Get all rigidbodies in the humanoid
        rigidbodies = GetComponentsInChildren<Rigidbody>();
        
        // Configure physics properties
        foreach (Rigidbody rb in rigidbodies)
        {
            rb.mass *= mass / 100; // Scale masses appropriately
            rb.drag = 0.1f; // Air resistance
            rb.angularDrag = 0.05f; // Rotational resistance
        }
    }
    
    void FixedUpdate()
    {
        // Apply gravity consistently
        foreach (Rigidbody rb in rigidbodies)
        {
            rb.AddForce(Physics.gravity * gravityScale * rb.mass);
        }
    }
}
```

## Calibration and Validation

### 1. Comparing Simulation to Reality
- Record motion capture data of real robot
- Compare to simulation with same commands
- Adjust physics parameters until behavior matches

### 2. Sensitivity Analysis
- Test how parameter changes affect behavior
- Identify which parameters most affect performance
- Focus calibration efforts on critical parameters

## Practical Exercise

1. Create a simple bipedal robot in Gazebo with proper inertial properties
2. Implement a basic balance controller using center of mass information
3. Test the robot's response to external forces
4. Validate the simulation by comparing to basic physics principles
5. Adjust parameters to achieve stable standing behavior

## Next Steps

With a strong understanding of physics simulation, we'll now put everything together in the final project for Module 2: creating a complete digital twin environment that leverages both Gazebo's physics accuracy and Unity's visualization capabilities.