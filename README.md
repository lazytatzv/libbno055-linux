# libbno055-linux

[![Build & Test](https://github.com/lazytatzv/libbno055-linux/actions/workflows/ci.yml/badge.svg)](https://github.com/lazytatzv/libbno055-linux/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++ Standard](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7F%20Jazzy-orange.svg)](https://docs.ros.org/)
[![Version](https://img.shields.io/badge/version-1.7.1-green.svg)](CHANGELOG.md)

**A high-performance C++17 library, ROS 2 integration, and multi-language binding (Python, C, Rust) for the Bosch BNO055 9-axis IMU on Linux.**

Features an integrated **Straight-Line & Heading PID Controller** for mobile robots, zero-copy transport, safety watchdog protection, and IMU fail-safe passthrough out of the box.

---

## ⚡ Quick Start

### 1. ROS 2 Users (One-Command Launch)

Launch the **IMU Driver** and **Heading PID Controller** in a single command (Zero-Copy Composable Container):

```bash
ros2 launch libbno055_linux heading_control_launch.py
```

> **Need Nav2 Lifecycle Management?** Pass `node_type:=lifecycle`:
> ```bash
> ros2 launch libbno055_linux heading_control_launch.py node_type:=lifecycle
> ```

### 2. Standalone C++ Users (Non-ROS)

Build and run the interactive 100Hz Heading PID Control Demo:

```bash
mkdir -p build && cd build
cmake .. && make -j$(nproc)

# Run the 100Hz PID Heading Control Demo
./heading_control_demo /dev/i2c-1
```

### 3. Python Users

```bash
pip install libbno055-linux
python3 -c "import libbno055; print('Installed successfully!')"
```

---

## ✨ Features

- **🚀 Low Latency & High Rate**: Supports up to 200Hz sensor updates, 1kHz/2kHz hardware ODR, and FTDI UART low-latency mode (`ASYNC_LOW_LATENCY`).
- **🎯 Integrated Heading PID Controller**: Zero-allocation C++17 controller featuring:
  - **Shortest-Path Angle Normalization**: Automatic $\pm 180^\circ$ boundary wrapping.
  - **Filtered Gyro D-Term**: 1st-order Low-Pass Filtered gyro rate to eliminate motor vibration noise.
  - **Trapezoidal Integration**: Jitter-free integration with anti-windup clamping.
  - **Slew-Rate Limiter**: Constrains angular acceleration to prevent wheel slip and motor gear shock.
- **🛡️ Safety & Fail-Safe Protection**:
  - **Command-Loss Watchdog**: Automatically stops the robot (`zero velocity`) if input commands time out.
  - **IMU Fail-Safe Passthrough**: Seamlessly passes velocity commands through if IMU is offline or disconnected.
  - **Outlier Rejection**: Rejects corrupted or `NaN`/`Inf` quaternion data automatically.
- **📡 Advanced ROS 2 Architecture**:
  - **Zero-Copy Composable Components** (`rclcpp_components`).
  - **Managed Lifecycle Nodes** (`rclcpp_lifecycle`) for Nav2 integration.
  - **Isolated CallbackGroups** (`MutuallyExclusive`) & `MultiThreadedExecutor`.
  - **Linux `SCHED_FIFO` Realtime Priority** support.
- **🌐 Polyglot Bindings**: Native support for **C++17**, **C99 FFI**, **Python 3**, and **Rust**.

---

## 🛠️ Installation & Building

### Prerequisites

- Linux (Ubuntu 20.04+, Raspberry Pi OS, etc.)
- C++17 compatible compiler (`gcc` 8+ or `clang` 7+)
- CMake 3.10+
- (Optional) ROS 2 Humble / Iron / Jazzy

### Building from Source

```bash
git clone https://github.com/lazytatzv/libbno055-linux.git
cd libbno055-linux
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### ROS 2 Workspace Build (`colcon`)

```bash
cd ~/ros2_ws/src
git clone https://github.com/lazytatzv/libbno055-linux.git
cd ~/ros2_ws
colcon build --packages-select libbno055_linux
source install/setup.bash
```

---

## ⚙️ Configuration & Parameter Tuning

Edit [`config/heading_control_params.yaml`](config/heading_control_params.yaml) to tune PID gains or timeouts:

```yaml
bno055_heading_control_node:
  ros__parameters:
    imu_topic: "imu/data"             # Input IMU topic
    cmd_vel_in_topic: "cmd_vel_in"     # Raw input velocity command
    cmd_vel_out_topic: "cmd_vel"       # Corrected output velocity command

    # PID & Control Gains
    kp: 0.05                           # Proportional Gain
    ki: 0.001                          # Integral Gain (Trapezoidal Rule)
    kd: 0.01                           # Derivative Gain (Filtered Gyro)
    kff: 0.0                           # Feedforward Gain
    max_i_term: 0.2                    # Anti-windup saturation limit
    max_slew_rate: 2.0                 # Max output change rate (rad/s^2)

    # Safety Timeouts
    cmd_vel_timeout: 0.5               # Command loss Watchdog timeout (seconds)
    imu_timeout: 1.0                   # IMU loss Fail-Safe timeout (seconds)
```

---

## 📚 Documentation & Guides

- 📘 [Heading Control Architecture & Tuning Guide](docs/HEADING_CONTROL.md)
- 📙 [C++ / C / Python / Rust API Reference](docs/API_REFERENCE.md)
- 📗 [ROS 2 Nodes & Component Architecture](src/ros2/README.md)
- 📕 [Code Examples Overview](examples/README.md)
- 📓 [Hardware Calibration Guide](docs/CALIBRATION.md)
- 📒 [Troubleshooting & FAQ](docs/TROUBLESHOOTING.md)

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).
