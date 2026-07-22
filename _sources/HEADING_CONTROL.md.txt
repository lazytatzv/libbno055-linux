# 🚀 Robot Straight-Line & Heading PID Control Guide

This document provides a comprehensive guide on using `libbno055-linux` for **straight-line drive stability, heading lock, and anti-drift control** in mobile robotics (diff-drive, omni-wheel, and Mecanum chassis).

---

## Table of Contents

1. [Overview](#overview)
2. [Core Control Theory & Features](#core-control-theory--features)
3. [C++ Production API (`HeadingController`)](#c-production-api-headingcontroller)
4. [ROS 2 Production Node (`bno055_heading_control_node`)](#ros-2-production-node-bno055_heading_control_node)
5. [ROS 2 Launch File & One-Command Setup](#ros-2-launch-file--one-command-setup)
6. [Tuning Guide & Best Practices](#tuning-guide--best-practices)

---

## Overview

When a mobile robot travels straight, physical factors such as **wheel diameter discrepancies, surface friction differences, and slip** cause the robot to drift off-course.

`libbno055-linux` provides a production-grade, zero-allocation **Heading PID Controller** that integrates BNO055 high-precision sensor fusion orientation and gyro rate to keep your robot driving straight.

```text
 [Upper System]                  [BNO055 IMU]
 Nav2 / Teleop                     /imu/data
       │                               │
       ▼ (cmd_vel_in)                  ▼
┌────────────────────────────────────────────┐
│      bno055_heading_control_node           │
├────────────────────────────────────────────┤
│ • Turning Command (angular.z != 0):        │
│   → Passthrough without interference        │
│ • Straight Driving (angular.z == 0):       │
│   → Automatically lock heading & apply     │
│     IMU PID correction to angular.z        │
└─────────────────────┬──────────────────────┘
                      │ (cmd_vel)
                      ▼
           [Motor Driver / Controller]
```

---

## Core Control Theory & Features

### 1. Shortest-Path Angle Normalization
Heading angles wrapped between $-180^\circ$ and $+180^\circ$ can cause naive controllers to spin $340^\circ$ in reverse when crossing boundary angles. `HeadingController` uses `std::remainder` for instant shortest-path error calculation:

$$\text{Error} = \text{normalizeAngleDeg}(\text{Target} - \text{Current})$$

### 2. Gyro-Based Derivative Control (Noise-Free D-Term)
Instead of computing numerical finite differences $(\Delta e / \Delta t)$, which amplifies high-frequency noise, the D-term directly utilizes BNO055's low-noise hardware gyro angular velocity ($\omega_z$). This eliminates derivative kick and provides instant anti-slip braking.

### 3. Anti-Windup Integral Clamping
To prevent integrator saturation during large disturbances or stuck wheels, the integral accumulator is strictly clamped to `max_i_term`:

$$I(t) = \text{clamp}\left(I(t-1) + K_i \cdot e(t) \cdot \Delta t, -I_{max}, +I_{max}\right)$$

---

## C++ Production API (`HeadingController`)

Header Location: `include/libbno055-linux/controllers/heading_controller.hpp`

```cpp
#include "libbno055-linux/bno055.hpp"
#include "libbno055-linux/controllers/heading_controller.hpp"

// 1. Configure PID parameters
bno055lib::HeadingController::Config cfg;
cfg.kp = 0.04;
cfg.ki = 0.001;
cfg.kd = 0.008;
cfg.max_output = 0.5; // Max correction in rad/s

bno055lib::HeadingController controller(cfg);

// 2. Control Loop (100Hz)
double target_heading = 0.0; // Locked orientation in degrees
double current_heading = 10.0; // Current IMU orientation in degrees
double gyro_z_deg = 1.5; // Gyro yaw rate in deg/s

auto out = controller.update(target_heading, current_heading, 0.01, gyro_z_deg, /*base_velocity=*/0.5);

std::cout << "Correction (u): " << out.correction << "\n";
std::cout << "Left Wheel Speed: " << out.left_motor << "\n";
std::cout << "Right Wheel Speed: " << out.right_motor << "\n";
```

---

## ROS 2 Production Node (`bno055_heading_control_node`)

The ROS 2 node is built as a **Composable Node Component (`rclcpp_components`)** supporting **Zero-Copy Intra-Process Transport**, **Dynamic Parameters**, and **Live Diagnostics**.

### Topics & Services

| Name | Type | Direction | Description |
| :--- | :--- | :---: | :--- |
| `imu/data` | `sensor_msgs/msg/Imu` | Sub | BNO055 IMU orientation and angular velocity. |
| `cmd_vel_in` | `geometry_msgs/msg/Twist` | Sub | Raw input velocity command from Teleop or Nav2. |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Pub | IMU-corrected output velocity command for motors. |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Pub | Real-time PID error, correction output, and lock status. |
| `~/reset_heading` | `std_srvs/srv/Trigger` | Service | Resets target heading to current robot orientation. |

### Dynamic ROS 2 Parameters

| Parameter | Type | Default | Description |
| :--- | :---: | :---: | :--- |
| `kp` | `double` | `0.05` | Proportional gain. |
| `ki` | `double` | `0.001` | Integral gain. |
| `kd` | `double` | `0.01` | Derivative gain (gyro-based). |
| `max_i_term` | `double` | `0.2` | Anti-windup integral saturation limit. |
| `angular_deadband` | `double` | `0.01` | Turning threshold to disengage heading lock. |

---

## ROS 2 Launch File & One-Command Setup

Launch the core BNO055 driver and Heading PID Controller in a single zero-copy composable container:

```bash
ros2 launch libbno055_linux heading_control_launch.py
```

### Live Parameter Tuning & Service Trigger

```bash
# Dynamically adjust Proportional Gain on the fly
ros2 param set /bno055_heading_control_node kp 0.08

# Trigger Manual Target Heading Reset
ros2 service call /bno055_heading_control_node/reset_heading std_srvs/srv/Trigger
```

---

## Tuning Guide & Best Practices

1. **Start with $K_p$**: Set $K_i = 0$ and $K_d = 0$. Increase $K_p$ until the robot firmly corrects straight-line drift without oscillating.
2. **Add $K_d$ for Damping**: If the robot overshoots or wiggles when returning to course, increase $K_d$ to apply smooth gyro-based braking.
3. **Add Small $K_i$ for Bias**: If persistent friction pulls the robot slightly to one side over long distances, add a small $K_i$ (e.g., $0.001$).
