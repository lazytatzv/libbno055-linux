# ROS 2 Integration Source Directory

This directory contains the ROS 2 integration nodes and components for `libbno055-linux`.

## Included ROS 2 Components

| Source File | Executable / Node | Description |
| :--- | :--- | :--- |
| **`ros2_publisher_node.cpp`** | `bno055_publisher_node` | High-frequency ROS 2 Imu publisher (Standard & Composable Node) with DiagnosticArray support. |
| **`ros2_lifecycle_publisher_node.cpp`** | `bno055_lifecycle_publisher_node` | ROS 2 Managed Lifecycle Node with state machine transitions (`configure`, `activate`, `deactivate`, `cleanup`). |
| **`ros2_heading_control_node.cpp`** | `bno055_heading_control_node` | High-performance Heading PID Controller publishing `/cmd_vel` (`geometry_msgs/msg/Twist`) from `/imu/data`. |
| **`bno055_ros2_common.hpp`** | Header-only helper | Shared ROS 2 parameter declaration, covariance setup, and diagnostic array builders. |

---

## How to Build & Run with ROS 2

```bash
cd ~/ros2_ws
colcon build --packages-select libbno055_linux
source install/setup.bash

# Launch the publisher node
ros2 launch libbno055_linux bno055_launch.py

# Launch the heading PID control node
ros2 run libbno055_linux bno055_heading_control_node --ros-args -p kp:=0.05 -p base_linear_speed:=0.3
```
