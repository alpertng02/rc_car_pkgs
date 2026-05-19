import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'rc_car_description'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Process the URDF file
    xacro_file = os.path.join(pkg_share, 'urdf', 'rc_car.xacro')
    gazebo_params_file = os.path.join(get_package_share_directory(pkg_name),'config','gazebo_params.yaml')
    
    # CRITICAL: We must pass 'use_sim_time': True so ROS syncs its clock with Gazebo
    robot_description_params = {
        'robot_description': Command(['xacro ', xacro_file]),
        'use_sim_time': True
    }

    # 1. Include the standard Gazebo launch file
    gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
            )


    # 2. Start robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_params]
    )

    # 3. Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'rc_car',
                   '-z', '0.05'], # Drops the car slightly from the air so it settles on the ground
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])