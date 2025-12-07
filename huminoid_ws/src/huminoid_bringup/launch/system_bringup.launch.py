from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package names
    pkg_huminoid_description = FindPackageShare(package='huminoid_description')
    pkg_huminoid_ai = FindPackageShare(package='huminoid_ai')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Use Gazebo
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_gazebo_arg = DeclareLaunchArgument(
        'use_gazebo',
        default_value='false',
        description='Use Gazebo simulator if true'
    )
    
    # Robot description command
    robot_description_cmd = Command([
        'xacro ', 
        PathJoinSubstitution([pkg_huminoid_description, 'urdf', 'huminoid.urdf'])
    ])
    
    # Launch Gazebo if requested
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ]),
        condition=IfCondition(use_gazebo)
    )
    
    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': robot_description_cmd}
        ],
        remappings=[
            ('/joint_states', 'huminoid/joint_states')
        ]
    )
    
    # Spawn the robot in Gazebo if using simulation
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'huminoid',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.5'
        ],
        output='screen',
        condition=IfCondition(use_gazebo)
    )
    
    # AI Decision Node
    ai_decision_node = Node(
        package='huminoid_ai',
        executable='ai_decision_node',
        name='ai_decision_node',
        parameters=[
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/huminoid/joint_states', 'huminoid/joint_states'),
            ('/huminoid/scan', 'huminoid/scan'),
            ('/huminoid/head_camera/image_raw', 'huminoid/head_camera/image_raw'),
            ('/huminoid/cmd_vel', 'huminoid/cmd_vel')
        ]
    )
    
    # Joint State Publisher (for non-simulated runs)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_sim_time)
    )
    
    # Create launch description and add actions
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_gazebo_arg)
    
    # Add actions
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(ai_decision_node)
    ld.add_action(joint_state_publisher)
    
    return ld