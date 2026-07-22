# ROS 2 Driver & Application Nodes Directory

This directory contains the production ROS 2 nodes and components for `libbno055-linux`.

## Included ROS 2 Nodes & Components

| Source File | Executable / Node | Description |
| :--- | :--- | :--- |
| **`ros2_publisher_node.cpp`** | `bno055_publisher_node` | High-frequency ROS 2 Imu publisher (Standard & Composable Node) with DiagnosticArray support. |
| **`ros2_heading_control_node.cpp`** | `bno055_heading_control_node` | Production Composable Heading PID Corrector Node (`cmd_vel_in` -> `cmd_vel`) with Dynamic Parameters & Trigger Service. |
| **`ros2_lifecycle_publisher_node.cpp`** | `bno055_lifecycle_publisher_node` | ROS 2 Managed Lifecycle Node with state machine transitions (`configure`, `activate`, `deactivate`, `cleanup`). |
| **`bno055_ros2_common.hpp`** | Header-only helper | Shared ROS 2 parameter declaration, covariance setup, and diagnostic array builders. |

---

## How to Build & Run with ROS 2

```bash
cd ~/ros2_ws
colcon build --packages-select libbno055_linux
source install/setup.bash

# 1. One-Command Launch (Core Driver + Heading PID Corrector Node in a Zero-Copy Composable Container)
ros2 launch libbno055_linux heading_control_launch.py

# 2. Launch Core IMU Publisher Node only
ros2 launch libbno055_linux bno055_launch.py
```
