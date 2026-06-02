import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_path = get_package_share_directory('ackermann_demo')
    
    # Process the Xacro file into clean URDF string
    xacro_file = os.path.join(pkg_path, 'urdf', 'car.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = {'robot_description': robot_description_config.toxml()}
    ps4_config_path = os.path.join(pkg_path, 'config', 'ps4_rc.yaml')

    
    controller_ps4_arg = DeclareLaunchArgument(
        'controller_ps4',
        default_value='false',
        description='Whether to start the SDL2 game controller teleop stack'
    )
    
    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_desc]
    )
    
    # Include Gazebo Classic Launch Script
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )
    
    # Spawn the Robot Entity into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'four_wheeled_car'],
        output='screen'
    )
    
    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    ackermann_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ackermann_steering_controller"],
    )
    
    game_controller_node = Node(
        package='joy',
        executable='game_controller_node',
        name='game_controller_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('controller_ps4')),
        parameters=[{'use_sim_time': True}]
    )

    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('controller_ps4')),
        parameters=[ps4_config_path, {'use_sim_time': True}]
    )

    return LaunchDescription([
        controller_ps4_arg,
        robot_state_publisher,
        gazebo,
        spawn_entity,
        joint_state_broadcaster_spawner,
        ackermann_steering_controller_spawner,
        game_controller_node,
        teleop_twist_joy_node
    ])