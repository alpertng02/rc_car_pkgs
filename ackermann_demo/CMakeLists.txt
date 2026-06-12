cmake_minimum_required(VERSION 3.8)
project(ackermann_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(astar_planner src/astar_planner.cpp)
ament_target_dependencies(astar_planner rclcpp nav_msgs geometry_msgs std_msgs tf2_ros)

add_executable(costmap_node src/costmap_node.cpp)
ament_target_dependencies(costmap_node rclcpp nav_msgs)

add_executable(stanley_controller src/stanley_controller.cpp)
ament_target_dependencies(stanley_controller rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2_ros)

add_executable(lidar_camera_fusion src/lidar_camera_fusion.cpp)
ament_target_dependencies(lidar_camera_fusion rclcpp sensor_msgs tf2_ros tf2_geometry_msgs cv_bridge image_geometry)

add_executable(cloud_accumulator src/cloud_accumulator.cpp)
ament_target_dependencies(cloud_accumulator rclcpp sensor_msgs tf2_ros tf2_geometry_msgs std_srvs)

add_library(ackermann_demo_hardware SHARED src/ackermann_hardware_interface.cpp)

# 🟢 ALTERNATIVE: Pass it directly to ament if find_package is declared at the top
ament_target_dependencies(ackermann_demo_hardware rclcpp pluginlib hardware_interface)

# Register the pluginlib descriptor map file
pluginlib_export_plugin_description_file(hardware_interface ackermann_demo_hardware.xml)

# Install rules configuration
install(TARGETS ackermann_demo_hardware DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config launch urdf rviz meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  astar_planner
  costmap_node
  stanley_controller
  lidar_camera_fusion
  cloud_accumulator
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/odom_drift_eval.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
