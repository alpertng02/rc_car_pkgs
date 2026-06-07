import os
import xacro
from typing import Any
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('ackermann_demo')
    
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'rc_car.rviz')
    slam_config_path = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')
    world_path = os.path.join(pkg_path, 'worlds', 'museum.world')

    # Parse URDF Model
    xacro_file = os.path.join(pkg_path, 'urdf', 'car.urdf.xacro')
    robot_description_config: Any = xacro.process_file(xacro_file)
    robot_desc = {'robot_description': robot_description_config.toxml()}
    ps4_config_path = os.path.join(pkg_path, 'config', 'ps4_rc.yaml')

    # Arguments Tracker
    controller_ps4_arg = DeclareLaunchArgument('controller_ps4', default_value='false')
    use_slam_arg = DeclareLaunchArgument('use_slam', default_value='false')
    use_nav_arg = DeclareLaunchArgument('use_nav', default_value='false')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    
    # Core Global TF Node Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[robot_desc]
    )
    
    # Include Gazebo Classic 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )
    
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'four_wheeled_car'], output='screen'
    )
    
    # Controller Spawners
    joint_state_broadcaster_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    ackermann_steering_controller_spawner = Node(package="controller_manager", executable="spawner", arguments=["ackermann_steering_controller"])
    
    # Teleop Nodes
    game_controller_node = Node(
        package='joy', executable='game_controller_node', name='game_controller_node',
        condition=IfCondition(LaunchConfiguration('controller_ps4')), parameters=[{'use_sim_time': True}]
    )
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
        condition=IfCondition(LaunchConfiguration('controller_ps4')), parameters=[ps4_config_path, {'use_sim_time': True}]
    )
    
    # Mapping/Localization Node
    slam_toolbox_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
        condition=IfCondition(LaunchConfiguration('use_slam')), parameters=[slam_config_path, {'use_sim_time': True}]
    )

    # ✅ RUNNING C++ TARGET: Points directly to the compiled binary inside install/lib/
    astar_planner_node = Node(
        package='ackermann_demo',
        executable='astar_planner',  
        name='astar_planner',
        output='screen',
        parameters=[{'use_sim_time': True}], 
        condition=IfCondition(LaunchConfiguration('use_nav')) 
    )
    
    costmap_node = Node(
        package='ackermann_demo',
        executable='costmap_node',
        name='costmap_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_radius': 0.22,    
            'safety_margin': 0.25    
        }],
        condition=IfCondition(LaunchConfiguration('use_nav'))
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')), parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        use_rviz_arg, use_slam_arg, use_nav_arg, controller_ps4_arg,
        robot_state_publisher, gazebo, spawn_entity,
        joint_state_broadcaster_spawner, ackermann_steering_controller_spawner,
        game_controller_node, teleop_twist_joy_node, slam_toolbox_node,
        astar_planner_node, costmap_node,
        rviz_node
    ])