import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_path = get_package_share_directory('ackermann_demo')
    
    # Configuration Paths
    slam_config_path = os.path.join(pkg_path, 'config', 'mapper_params_online_async.yaml')
    controllers_yaml_path = os.path.join(pkg_path, 'config', 'controllers.yaml')

    # Hardware Feature Toggles
    use_slam_arg = DeclareLaunchArgument('use_slam', default_value='true')
    use_nav_arg = DeclareLaunchArgument('use_nav', default_value='true')
    # Planner selection: 'ackermann' = paper-faithful Hybrid A* (RS analytic
    # expansion + dual heuristic + Voronoi-aware smoother); 'legacy' = the
    # original astar_planner. Defaults to the closed-loop ackermann planner.
    planner_arg = DeclareLaunchArgument(
        'planner', default_value='ackermann',
        description="Global planner to launch: 'ackermann' or 'legacy'")

    # condition helpers: nav enabled AND the selected planner matches
    nav_and_ackermann = IfCondition(PythonExpression([
        "'", LaunchConfiguration('use_nav'), "' == 'true' and '",
        LaunchConfiguration('planner'), "' == 'ackermann'"]))
    nav_and_legacy = IfCondition(PythonExpression([
        "'", LaunchConfiguration('use_nav'), "' == 'true' and '",
        LaunchConfiguration('planner'), "' != 'ackermann'"]))
    
    # Compile URDF with sim_mode disabled to invoke C++ Serial Plugins
    xacro_file = os.path.join(pkg_path, 'urdf', 'car.urdf.xacro')
    # ParameterValue(value_type=str) stops launch_ros from YAML-parsing the
    # URDF, which fails on colons inside XML comments.
    robot_desc = {
        'robot_description': ParameterValue(Command(['xacro ', xacro_file, ' sim_mode:=false']), value_type=str)
    }
    
    # Robot State Publisher (Calculates TF frames using Jetson system time)
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[robot_desc, {'use_sim_time': False}]
    )
    
    # Standalone ros2_control Node (Replaces Gazebo backend manager)
    # real_car_control_node = Node(
    #     package='controller_manager', executable='ros2_control_node',
    #     parameters=[robot_desc, controllers_yaml_path, {'use_sim_time': False}],
    #     output='screen'
    # )
    
    # Hardware Driver: RPLidar A1 Driver Node
    lidar_driver_node = Node(
        package='sllidar_ros2', executable='sllidar_node', name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'use_sim_time': False
        }],
        output='screen'
    )

    # Spawners (Once real_car_control_node establishes the pipeline, inject controllers)
    joint_state_broadcaster_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    ackermann_steering_controller_spawner = Node(package="controller_manager", executable="spawner", arguments=["ackermann_steering_controller"])
    
    # Mapping & Navigation Stack Nodes
    slam_toolbox_node = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
        condition=IfCondition(LaunchConfiguration('use_slam')), parameters=[slam_config_path, {'use_sim_time': False}]
    )

    # Paper-faithful planner (default); consumes /voronoi_field in its smoother.
    ackermann_astar_planner_node = Node(
        package='ackermann_demo', executable='ackermann_astar_planner', name='ackermann_astar_planner',
        output='screen', condition=nav_and_ackermann, parameters=[{'use_sim_time': False}]
    )

    # Legacy planner, selected with planner:=legacy.
    astar_planner_node = Node(
        package='ackermann_demo', executable='astar_planner', name='astar_planner', output='screen',
        condition=nav_and_legacy, parameters=[{'use_sim_time': False}]
    )

    costmap_node = Node(
        package='ackermann_demo', executable='costmap_node', name='costmap_node', output='screen',
        condition=IfCondition(LaunchConfiguration('use_nav')),
        parameters=[{'use_sim_time': False, 'robot_radius': 0.22, 'safety_margin': 0.15}]
    )

    # Voronoi field for the closed-loop smoother (harmless when planner:=legacy).
    voronoi_costmap_node = Node(
        package='ackermann_demo', executable='voronoi_costmap', name='voronoi_costmap', output='screen',
        condition=IfCondition(LaunchConfiguration('use_nav')),
        parameters=[{'use_sim_time': False, 'd_o_max': 1.0, 'alpha': 0.5}]
    )

    return LaunchDescription([
        use_slam_arg, use_nav_arg, planner_arg,
        robot_state_publisher,
        #real_car_control_node,
        lidar_driver_node,
        joint_state_broadcaster_spawner,
        ackermann_steering_controller_spawner,
        slam_toolbox_node,
        ackermann_astar_planner_node,
        astar_planner_node,
        costmap_node,
        voronoi_costmap_node
    ])