import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 1. Define package paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_rc_car = get_package_share_directory('rc_car_description')

    urdf_file = os.path.join(pkg_rc_car, 'urdf', 'rc_car.xacro')
    museum_world_path = os.path.join(pkg_rc_car, 'worlds', 'museum.world')
    rviz_config_path = os.path.join(pkg_rc_car, 'rviz', 'rc_car.rviz')
    slam_config_path = os.path.join(pkg_rc_car, 'config', 'mapper_params_online_async.yaml')
    ps4_config_path = os.path.join(pkg_rc_car, 'config', 'ps4_rc.yaml')
    
    # 2. Declare Launch Arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Whether to start slam_toolbox'
    )

    controller_ps4_arg = DeclareLaunchArgument(
        'controller_ps4',
        default_value='false',
        description='Whether to start the SDL2 game controller teleop stack'
    )

    # 3. Process the URDF with Xacro
    robot_description_content = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # 4. Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # 5. Launch Gazebo Server (gzserver)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': museum_world_path}.items()
    )

    # 6. Launch Gazebo Client (gzclient)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 7. Spawn the Car in Gazebo (Fixed: Runs on system time to prevent immediate service timeout)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rc_car',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '-1.57',
            '-timeout', '120.0'
         ],
        output='screen'
    )

    # 8. RViz Node (Added: use_sim_time parameter for smooth UI transform updates)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # 9. SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_slam')),
        parameters=[
            slam_config_path,
            {'use_sim_time': True}
        ]
    )
    
    # 10. ros2_control Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_steering_controller', '--controller-manager', '/controller_manager'],
    )
    
    # 11. PS4 Controller Teleop Stack
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
        use_rviz_arg,
        use_slam_arg,
        controller_ps4_arg,
        robot_state_publisher_node,
        gzserver,
        gzclient,
        spawn_entity_node,
        rviz_node,
        slam_toolbox_node,
        joint_state_broadcaster_spawner,
        ackermann_steering_controller_spawner,
        game_controller_node,
        teleop_twist_joy_node
    ])