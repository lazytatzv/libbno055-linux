# ROS 2 Driver Core Directory

This directory contains the core ROS 2 sensor driver nodes and components for `libbno055-linux`.

## Included Core ROS 2 Driver Nodes

| Source File | Executable / Node | Description |
| :--- | :--- | :--- |
| **`ros2_publisher_node.cpp`** | `bno055_publisher_node` | High-frequency ROS 2 Imu publisher (Standard & Composable Node) with DiagnosticArray support. |
| **`ros2_lifecycle_publisher_node.cpp`** | `bno055_lifecycle_publisher_node` | ROS 2 Managed Lifecycle Node with state machine transitions (`configure`, `activate`, `deactivate`, `cleanup`). |
| **`bno055_ros2_common.hpp`** | Header-only helper | Shared ROS 2 parameter declaration, covariance setup, and diagnostic array builders. |

> **Note**: For ROS 2 application nodes (such as the PID Heading Controller), see [`examples/ros2/ros2_heading_control_node.cpp`](../../examples/ros2/ros2_heading_control_node.cpp).

---

## How to Build & Run Core Driver

```bash
cd ~/ros2_ws
colcon build --packages-select libbno055_linux
source install/setup.bash

# Launch the core driver node
ros2 launch libbno055_linux bno055_launch.py
```
