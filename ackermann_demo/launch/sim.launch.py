import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_path = get_package_share_directory('ackermann_demo')
    
    # Configuration Paths
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'rc_car.rviz')
    slam_config_path = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')
    ekf_config_path = os.path.join(pkg_path, 'config', 'ekf.yaml')
    gazebo_params_path = os.path.join(pkg_path, 'config', 'gazebo_params.yaml')
    world_path = os.path.join(pkg_path, 'worlds', 'museum.world')
    ps4_config_path = os.path.join(pkg_path, 'config', 'ps4_rc.yaml')

    # Feature Toggles
    controller_ps4_arg = DeclareLaunchArgument('controller_ps4', default_value='false')
    use_slam_arg = DeclareLaunchArgument('use_slam', default_value='false')
    use_nav_arg = DeclareLaunchArgument('use_nav', default_value='false')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    use_cam_fuse_arg = DeclareLaunchArgument('use_cam_fuse', default_value='false')
    
    # Compile URDF with sim_mode enabled
    xacro_file = os.path.join(pkg_path, 'urdf', 'car.urdf.xacro')
    # ParameterValue(value_type=str) stops launch_ros from YAML-parsing the
    # URDF, which fails on colons inside XML comments.
    robot_desc = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' sim_mode:=true']), value_type=str)
    }
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[robot_desc, {'use_sim_time': True}]
    )
    
    # Gazebo World & Spawner
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        # params_file raises the /clock publish rate (default 10 Hz throttles
        # every sim-time timer outside gazebo, including the EKF's TF output).
        launch_arguments={'world': world_path, 'params_file': gazebo_params_path}.items()
    )
    
    spawn_entity = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'four_wheeled_car', '-timeout', '60'], 
        output='screen'
    )
    
    # ros2_control Controller Spawners
    joint_state_broadcaster_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    ackermann_steering_controller_spawner = Node(package="controller_manager", executable="spawner", arguments=["ackermann_steering_controller"])
    
    # Sensor fusion: wheel odometry + IMU -> odom->base_footprint TF.
    # Runs unconditionally because everything above it (slam, nav, teleop
    # visualization) depends on the odom frame.
    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node',
        output='screen', parameters=[ekf_config_path, {'use_sim_time': True}]
    )

    # Optional Simulation Mapping & Navigation Nodes
    slam_toolbox_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
        condition=IfCondition(LaunchConfiguration('use_slam')), parameters=[slam_config_path, {'use_sim_time': True}]
    )

    # 🏁 UPDATED: Expanded with full structural parameters matching the upgraded Hybrid A* planner
    astar_planner_node = Node(
        package='ackermann_demo',
        executable='astar_planner',
        name='astar_planner',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'default_tolerance': 0.25,       
            'turning_radius': 0.35,          # Kept tightly calibrated to your physical RC bounds
            'step_size': 0.15,               
            'max_iterations': 500000,        # High exploration budget for long-distance searches
            'deviation_threshold': 0.20,     
            'lethal_cost_threshold': 85,     # Obstacle density boundary
            'unknown_cost_penalty': 5.0,    # Metric cost weight multiplier for unmapped space
            'theta_bins': 72                 # 5-degree discrete heading state grid resolution
        }],
        condition=IfCondition(LaunchConfiguration('use_nav')) 
    )
    
    costmap_node = Node(
        package='ackermann_demo', executable='costmap_node', name='costmap_node', output='screen',
        condition=IfCondition(LaunchConfiguration('use_nav')), 
        parameters=[{'use_sim_time': True, 'robot_radius': 0.18, 'safety_margin': 0.22}]
    )
    
    # 🏎️ UPDATED: Added structural parameters for proprioceptive stall monitoring
    stanley_controller_node = Node(
        package='ackermann_demo',
        executable='stanley_controller',
        name='stanley_controller',
        condition=IfCondition(LaunchConfiguration('use_nav')), 
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'control_hz': 100.0,
            
            # Mechanical Dimensions
            'wheelbase': 0.1688,                  
            'max_steering_angle': 0.523,          
            
            # Tracking Gains
            'stanley_k': 1.8,                     
            'stanley_k_soft': 0.18,               
            
            # Operational Constraints
            'max_linear_velocity': 1.0,          
            'max_reverse_velocity': 1.0,         
            'goal_tolerance': 0.10,               
            
            # Acceleration Ramp Profiles
            'max_acceleration': 0.50,             
            'max_deceleration': 0.90,             
            'steer_speed_reduction': 0.70,        
            'path_timeout_sec': 20.0,             
            'ultrasonic_safety_dist': 0.20,       

            # 🟢 ADDED: Proprioceptive Stall & Wheel Slip Monitoring Configurations
            'stall_velocity_threshold': 0.15,     # Minimum speed command to begin evaluating [m/s]
            'stall_odom_threshold': 0.02,         # Maximum speed baseline below which the car is considered stuck [m/s]
            'stall_accel_threshold': 0.05,        # Noise ceiling for real IMU linear acceleration frames [m/s²]
            'stall_timeout_sec': 1.0,             # Allowed time window of zero acceleration/movement before path dropout
            'collision_accel_threshold': 3.0      # IMU forward-acceleration spike that is treated as an impact [m/s²]
        }]
    )
    
    # LiDAR-Camera Fusion: colors the laser scan with camera RGB -> /colored_scan
    lidar_camera_fusion_node = Node(
        package='ackermann_demo', executable='lidar_camera_fusion', name='lidar_camera_fusion', output='screen',
        condition=IfCondition(LaunchConfiguration('use_cam_fuse')),
        parameters=[{'use_sim_time': True}]
    )

    # Accumulates /colored_scan into a persistent voxel map (/colored_map) in the
    # SLAM map frame — needs use_slam:=true for the map->odom TF
    cloud_accumulator_node = Node(
        package='ackermann_demo', executable='cloud_accumulator', name='cloud_accumulator', output='screen',
        condition=IfCondition(LaunchConfiguration('use_cam_fuse')),
        parameters=[{'use_sim_time': True, 'target_frame': 'map', 'voxel_size': 0.05}]
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
        controller_ps4_arg, use_slam_arg, use_nav_arg, use_rviz_arg, use_cam_fuse_arg,
        robot_state_publisher, gazebo, spawn_entity,
        joint_state_broadcaster_spawner, ackermann_steering_controller_spawner,
        ekf_node,
        game_controller_node, teleop_twist_joy_node, cmd_vel_mux_node,
        slam_toolbox_node,
        astar_planner_node, costmap_node, stanley_controller_node,
        lidar_camera_fusion_node, cloud_accumulator_node,
        rviz_node
    ])