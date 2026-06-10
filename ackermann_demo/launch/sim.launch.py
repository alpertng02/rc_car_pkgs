import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('ackermann_demo')
    
    # Configuration Paths
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'rc_car.rviz')
    slam_config_path = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')
    world_path = os.path.join(pkg_path, 'worlds', 'museum.world')
    ps4_config_path = os.path.join(pkg_path, 'config', 'ps4_rc.yaml')

    # Feature Toggles
    controller_ps4_arg = DeclareLaunchArgument('controller_ps4', default_value='false')
    use_slam_arg = DeclareLaunchArgument('use_slam', default_value='false')
    use_nav_arg = DeclareLaunchArgument('use_nav', default_value='false')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    
    # Compile URDF with sim_mode enabled
    xacro_file = os.path.join(pkg_path, 'urdf', 'car.urdf.xacro')
    robot_desc = {
        'robot_description': Command(['xacro ', xacro_file, ' sim_mode:=true'])
    }
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[robot_desc, {'use_sim_time': True}]
    )
    
    # Gazebo World & Spawner
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_path}.items()
    )
    
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'four_wheeled_car', '-timeout', '60'], 
        output='screen'
    )
    
    # ros2_control Controller Spawners
    joint_state_broadcaster_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    ackermann_steering_controller_spawner = Node(package="controller_manager", executable="spawner", arguments=["ackermann_steering_controller"])
    
    # Optional Simulation Mapping & Navigation Nodes
    slam_toolbox_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
        condition=IfCondition(LaunchConfiguration('use_slam')), parameters=[slam_config_path, {'use_sim_time': True}]
    )

    astar_planner_node = Node(
        package='ackermann_demo',
        executable='astar_planner',
        name='astar_planner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'default_tolerance': 0.25,       
            'turning_radius': 0.5,          
            'step_size': 0.1,               
            'max_iterations': 500000,          
            'deviation_threshold': 0.20      
        }],
        condition=IfCondition(LaunchConfiguration('use_nav')) 
    )
    
    costmap_node = Node(
        package='ackermann_demo', executable='costmap_node', name='costmap_node', output='screen',
        condition=IfCondition(LaunchConfiguration('use_nav')), 
        parameters=[{'use_sim_time': True, 'robot_radius': 0.18, 'safety_margin': 0.22}]
    )
    
    # 🏎️ OVERHAULED: Parameters aligned exactly with the production Stanley node
    stanley_controller_node = Node(
        package='ackermann_demo',
        executable='stanley_controller',
        name='stanley_controller',
        condition=IfCondition(LaunchConfiguration('use_nav')), 
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Mechanical Dimensions
            'wheelbase': 0.1688,                  # Calibrated rear-to-front axle distance [m]
            'max_steering_angle': 0.523,          # Calibrated max steering lock [rad] (~30 deg)
            
            # Tracking Gains
            'stanley_k': 1.6,                     # Cross-track error tracking gain
            'stanley_k_soft': 0.18,               # Velocity softening dampener [m/s]
            
            # Operational Constraints
            'max_linear_velocity': 1.0,          # Forward cruise profile limit [m/s]
            'max_reverse_velocity': 0.30,         # Reverse profile limit [m/s]
            'goal_tolerance': 0.10,               # Goal arrival radius bubble [m]
            
            # Acceleration Ramp Profiles
            'max_acceleration': 0.50,             # Longitudinal acceleration limits [m/s²]
            'max_deceleration': 0.90,             # Braking/Deceleration profile limits [m/s²]
            'steer_speed_reduction': 0.70,        # Aggressiveness of cornering velocity dampening
            'path_timeout_sec': 30.0               # Safety fallback data-drop window timeout [s]
        }]
    )
    
    # Optional Gamepad & Visualization
    game_controller_node = Node(
        package='joy', executable='game_controller_node', name='game_controller_node',
        condition=IfCondition(LaunchConfiguration('controller_ps4')), parameters=[{'use_sim_time': True}]
    )
    
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy', executable='teleop_node', name='teleop_twist_joy_node',
        condition=IfCondition(LaunchConfiguration('controller_ps4')), 
        parameters=[ps4_config_path, {'use_sim_time': True, 'stamped': True}],
        remappings=[('/cmd_vel', '/cmd_vel_teleop')] 
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')), parameters=[{'use_sim_time': True}]
    )

    cmd_vel_mux_config = os.path.join(pkg_path, 'config', 'cmd_vel_mux.yaml')
    cmd_vel_mux_node = Node(
        package='cmd_vel_mux',
        executable='cmd_vel_mux_node',
        name='cmd_vel_mux',
        output='screen',
        parameters=[cmd_vel_mux_config, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/cmd_vel')] 
    )

    return LaunchDescription([
        controller_ps4_arg, use_slam_arg, use_nav_arg, use_rviz_arg,
        robot_state_publisher, gazebo, spawn_entity,
        joint_state_broadcaster_spawner, ackermann_steering_controller_spawner,
        game_controller_node, teleop_twist_joy_node, cmd_vel_mux_node,
        slam_toolbox_node,
        astar_planner_node, costmap_node, stanley_controller_node, 
        rviz_node
    ])