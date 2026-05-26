import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

# This import is the magic fix for the YAML parsing crash!
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 1. Define package paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_rc_car = get_package_share_directory('rc_car_description')

    # IMPORTANT: Change 'rc_car.urdf' to whatever your file is actually named 
    # (e.g., 'rc_car.urdf.xacro' or 'robot.urdf')
    urdf_file = os.path.join(pkg_rc_car, 'urdf', 'rc_car.xacro')

    museum_world_path = pkg_rc_car = os.path.join(
        get_package_share_directory('rc_car_description'), 'worlds', 'museum.world')


    # 3. Process the URDF with Xacro AND wrap it as a pure string to prevent YAML crashes
    robot_description_content = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    # 4. Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 5. Launch Gazebo Server (gzserver) with the museum World
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

    # 7. Spawn the Car safely in the museum (x=0, y=1.0, z=0.2)
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'rc_car',
            '-x', '0.0',
            '-y', '1.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gzserver,
        gzclient,
        spawn_entity_node
    ])