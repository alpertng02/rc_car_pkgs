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
    
    # NEW: Path to your RViz config file (update the filename if yours is different)
    rviz_config_path = os.path.join(pkg_rc_car, 'rviz', 'rc_car.rviz')
    
    slam_config_path = os.path.join(pkg_rc_car, 'config', 'mapper_params_online_async.yaml')
    
    # 2. Declare Launch Arguments
    # This allows you to run: ros2 launch rc_car_description launch.py use_rviz:=false
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    use_slam_arg = DeclareLaunchArgument(
        'use_slam',
        default_value='false', # Set to false by default so it doesn't run unless requested
        description='Whether to start slam_toolbox'
    )

    # 3. Process the URDF with Xacro
    robot_description_content = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # 4. Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 5. Launch Gazebo Server (gzserver)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': museum_world_path}.items()
    )

    # 6. Launch Gazebo Client (gzclient) for the GUI
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 7. Spawn the Car in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rc_car',
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '-1.57',
            '-timeout', '120.0' # <-- Tell the script to wait up to 120 seconds
         ],
        output='screen'
    )

    # 8. NEW: RViz Node with Condition
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_slam')),
        parameters=[
            slam_config_path,          # Load the YAML file first
            {'use_sim_time': True}     # Then override with use_sim_time for Gazebo
        ]
    )

    return LaunchDescription([
        use_rviz_arg,
        use_slam_arg,
        robot_state_publisher_node,
        gzserver,
        gzclient,
        spawn_entity_node,
        rviz_node,
        slam_toolbox_node
    ])