from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package names
    pkg_huminoid_description = FindPackageShare(package='huminoid_description')
    pkg_huminoid_ai = FindPackageShare(package='huminoid_ai')
    pkg_huminoid_bringup = FindPackageShare(package='huminoid_bringup')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    world_file = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_huminoid_bringup, 'worlds', 'huminoid_world.sdf']),
        description='Path to the world file'
    )
    
    # Launch Gazebo with the custom world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # Robot State Publisher node (loads the URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': PathJoinSubstitution([pkg_huminoid_description, 'urdf', 'huminoid.urdf'])}
        ]
    )
    
    # Spawn the robot in Gazebo
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
        output='screen'
    )
    
    # AI Decision Node
    ai_decision_node = Node(
        package='huminoid_ai',
        executable='ai_decision_node',
        name='ai_decision_node',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Create launch description and add actions
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_arg)
    
    # Add actions
    ld.add_action(gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    ld.add_action(ai_decision_node)
    
    return ld