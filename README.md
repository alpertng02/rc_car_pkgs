# Ackermann Car Demo (`ackermann_demo`)

An advanced ROS 2 simulation and autonomy stack for an Ackermann-steered vehicle. This package features a fully custom navigation pipeline, to utilize a paper-faithful Hybrid A* planner, a Stanley controller with stall monitoring, and a LiDAR-Camera sensor fusion.

## Core Features

* **Custom Planner:** A specialized Ackermann Hybrid A* planner featuring Reed-Shepp analytic expansion, dual heuristics, and a Voronoi-field-aware smoother.
* **Advanced Path Tracking:** Implements a Stanley controller tailored for Ackermann kinematics (`ackermann_steering_controller`). It includes operational constraints (acceleration ramp profiles) and built-in stall/collision monitoring using IMU data.
* **Sensor Fusion & Mapping:** * Integrates `robot_localization` (EKF) for Wheel Odometry + IMU fusion.
    * Full support for `slam_toolbox` (async online) for 2D occupancy grid mapping.
* **LiDAR-Camera Voxel Fusion:** Optional nodes to colorize laser scans with camera RGB data (`/colored_scan`) and accumulate them into a persistent voxel map (`/colored_map`) aligned with the SLAM frame.
* **Autonomous Exploration:** A frontier exploration node allows the car to map its environment and drive completely unattended.
* **Hardware-Ready Teleop:** Multiplexed command velocities (`cmd_vel_mux`) supporting seamless switching between the autonomy stack and a PS4 controller (`joy` + `teleop_twist_joy`).

## Prerequisites

This package is built for ROS 2 (tested on Linux, including Ubuntu/Kubuntu) and relies on standard simulation and control frameworks. 

**Dependencies:**
```bash
sudo apt install ros-<distro>-gazebo-ros-pkgs ros-<distro>-ros2-control ros-<distro>-ros2-controllers ros-<distro>-slam-toolbox ros-<distro>-robot-localization ros-<distro>-joy ros-<distro>-teleop-twist-joy