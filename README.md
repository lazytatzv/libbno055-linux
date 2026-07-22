# libbno055-linux

[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](include/libbno055-linux/bno055.hpp)
[![PyPI](https://img.shields.io/pypi/v/libbno055-linux.svg)](https://pypi.org/project/libbno055-linux/)
[![crates.io](https://img.shields.io/crates/v/libbno055.svg)](https://crates.io/crates/libbno055)
[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB.svg)](src/python/bindings.cpp)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy%20%7C%20Kilted-22314E.svg)](docs/INTEGRATION.md)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20%7C%2024.04-E95420.svg)](docs/INTEGRATION.md)
[![CI](https://github.com/lazytatzv/libbno055-linux/actions/workflows/ci.yml/badge.svg)](https://github.com/lazytatzv/libbno055-linux/actions/workflows/ci.yml)
[![Coverage](https://codecov.io/gh/lazytatzv/libbno055-linux/graph/badge.svg)](https://codecov.io/gh/lazytatzv/libbno055-linux)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue.svg)](https://lazytatzv.github.io/libbno055-linux/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

> **A robust, low-latency (270µs) BNO055 IMU driver for Linux, supporting I2C/UART with auto-recovery.**

A C++17 driver with C, Python, Rust, and ROS 2 bindings for the Bosch BNO055 9-DOF IMU sensor.

## Demonstration

```console
$ ros2 topic echo /imu/data
header:
  stamp:
    sec: 1718920102
    nanosec: 432100000
  frame_id: 'imu_link'
orientation:
  x: 0.001
  y: 0.002
  z: 0.707
  w: 0.707
angular_velocity:
  x: 0.012
  y: -0.004
  z: 0.001
linear_acceleration:
  x: 0.02
  y: -0.01
  z: 9.81
```

---

## Features

| Feature | `libbno055-linux` | Adafruit | Kernel IIO |
| :--- | :---: | :---: | :---: |
| **ROS 2** | ✅ | ❌ | △ |
| **Python** | ✅ | ✅ | ❌ |
| **Rust** | ✅ | ❌ | ❌ |
| **UART (921600 bps)** | ✅ | ❌ | △ |
| **Auto Recovery** | ✅ | ❌ | ❌ |
| **Zero-CPU Interrupts**| ✅ | ❌ | ❌ |

---

## Highlights

- **Native I2C & UART Support**: Fast POSIX drivers with hardware auto-recovery for I2C bus lockups.
- **ROS 2 Lifecycle Integration**: High-performance intra-process publishing and diagnostic streams.
- **Multi-Language APIs**: C++17 core engine with native C, Python (`pip`), and safe Rust (`crates.io`).

---

## Documentation

This README is a quick overview. Please see the `docs/` directory for detailed information:

- **[Sphinx API Documentation (Web)](https://lazytatzv.github.io/libbno055-linux/)**: Auto-generated web documentation for C++ and Python APIs.
- **[Features & Comparison](docs/FEATURES_AND_COMPARISON.md)**: Detailed feature list and why you should choose this over Kernel IIO drivers.
- **[Calibration Best Practices](docs/CALIBRATION.md)**: The "Pro Workflow" for managing calibration in real-time robotic control loops.
- **[API Reference (Markdown)](docs/API_REFERENCE.md)**: Full class and function reference for C++, C, Python, and Rust.
- **[Integration & Tuning Guide](docs/INTEGRATION.md)**: ROS 2 YAML parameters, EKF setup, 400kHz I2C, UART 921600 bps tuning.
- **[Architecture & Design](docs/ARCHITECTURE.md)**: PIMPL design, zero-copy transport, FFI layers, and state machines.
- **[Troubleshooting & FAQ](docs/TROUBLESHOOTING.md)**: Hardware wiring, permissions, and clock-stretching fixes.

---

## Installation

### Ubuntu (APT / ROS 2)
```bash
sudo apt update && sudo apt install ros-${ROS_DISTRO}-libbno055-linux
```

### Python (pip)
```bash
pip install libbno055-linux
```

### Rust (Cargo)
```bash
cargo add libbno055
```

### Build from Source
```bash
git clone https://github.com/lazytatzv/libbno055-linux.git
cd libbno055-linux
mkdir build && cd build
cmake .. && make
sudo make install
```

---

## Quick Start

### Rust (`crates.io`)

> **Tip:** See [`rust/examples/demo.rs`](rust/examples/demo.rs) for a complete, runnable example. You can run it instantly with `cargo run --example demo`.

```bash
cargo add libbno055
```

```rust
use libbno055::{BNO055, OpMode};

fn main() -> Result<(), &'static str> {
    let mut imu = BNO055::new_i2c(0x28, "/dev/i2c-1")?;
    if imu.begin(OpMode::NDOF) {
        if let Some(q) = imu.get_quaternion() {
            let euler = BNO055::to_euler_degrees(&q);
            println!("Roll: {:.2}, Pitch: {:.2}, Yaw: {:.2}", euler.x, euler.y, euler.z);
        }
    }
    Ok(())
}
```

---

### Python (`pip`)

> **Tip:** See [`examples/python_demo.py`](examples/python_demo.py) for a complete, runnable script featuring diagnostics, error handling, and formatting.

```bash
pip install libbno055-linux  # Or: pip install . (from source)
```

```python
import libbno055

imu = libbno055.BNO055(address=0x28, device="/dev/i2c-1")
if imu.begin(libbno055.OpMode.NDOF):
    q = imu.get_quaternion()
    if q:
        euler = libbno055.to_euler_degrees(q)
        print(f"Roll: {euler.x:.2f}, Pitch: {euler.y:.2f}, Yaw: {euler.z:.2f}")
```

---

### C++17 (Native CMake)

```cpp
#include <libbno055-linux/bno055.hpp>
#include <iostream>

int main() {
    bno055lib::BNO055 imu(0x28, "/dev/i2c-1");
    if (imu.begin(bno055lib::OpMode::NDOF)) {
        if (auto q = imu.getQuaternionNoexcept()) {
            auto euler = bno055lib::toEulerDegrees(*q);
            std::cout << "Roll: " << euler.x << " Pitch: " << euler.y << " Yaw: " << euler.z << "\n";
        }
    }
}
```

---

### ROS 2 Node (Binary Installation)

```bash
sudo apt update && sudo apt install ros-${ROS_DISTRO}-libbno055-linux
ros2 launch libbno055_linux bno055_launch.py
```

---

### Robot Straight-Line & Heading PID Control Demo

Want to prevent your robot from drifting during straight-line driving or maintain target heading? Try the included ROS 2-style PID heading controller demo:

```bash
# C++ Standalone Demo (Visual ASCII Motor Output Dashboard)
./build/heading_control_demo /dev/i2c-1

# ROS 2 Heading Control Node (publishes cmd_vel from /imu/data)
ros2 run libbno055_linux bno055_heading_control_node --ros-args -p kp:=0.05 -p base_linear_speed:=0.3
```

---

## License

This project is released under the [MIT License](LICENSE).
