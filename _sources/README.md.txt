# libbno055-linux

[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](include/libbno055-linux/bno055.hpp)
[![crates.io](https://img.shields.io/crates/v/libbno055.svg)](https://crates.io/crates/libbno055)
[![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB.svg)](src/python/bindings.cpp)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Jazzy%20%7C%20Kilted-22314E.svg)](docs/INTEGRATION.md)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-22.04%20%7C%2024.04-E95420.svg)](docs/INTEGRATION.md)
[![CI](https://github.com/lazytatzv/libbno055-linux/actions/workflows/ci.yml/badge.svg)](https://github.com/lazytatzv/libbno055-linux/actions/workflows/ci.yml)
[![Coverage](https://codecov.io/gh/lazytatzv/libbno055-linux/graph/badge.svg)](https://codecov.io/gh/lazytatzv/libbno055-linux)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue.svg)](https://lazytatzv.github.io/libbno055-linux/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
> **A robust, low-latency BNO055 IMU driver for Linux, supporting I2C/UART with auto-recovery.**

A polyglot C++17, ROS 2, Python, C, and Rust driver for the Bosch BNO055 9-DOF IMU sensor.

## Demonstration

<!-- TODO: Replace this placeholder with a GIF of Rviz2 or terminal output! -->
<p align="center">
  <img src="https://via.placeholder.com/800x400?text=Drop+a+GIF+of+Rviz2+or+Terminal+Output+Here" alt="Demo GIF Placeholder">
</p>

---

## Highlights

- **Native I2C & UART Support**: Fast POSIX drivers with hardware auto-recovery for I2C bus lockups and UART overrun errors.
- **ROS 2 Lifecycle & Diagnostics**: Production ROS 2 lifecycle nodes with zero-copy publishing and diagnostic streams.
- **Multi-Language FFI Bindings**: C++17 core engine with native C, Python (`pip`), and safe Rust (`crates.io`) interfaces.

---

## Why libbno055-linux?

| Feature | `libbno055-linux` | Adafruit BNO055 | Bosch Reference |
| :--- | :---: | :---: | :---: |
| **Linux I2C & UART** | ✅ | ⚠️ | ❌ |
| **Automatic Recovery** | ✅ | ❌ | ❌ |
| **ROS 2 & Lifecycle** | ✅ | ❌ | ❌ |
| **Zero-Allocation Hot Path** | ✅ | ❌ | ❌ |
| **Rust & Python Bindings** | ✅ | ⚠️ | ❌ |
| **Burst Read (18-Byte)** | ✅ | ❌ | ❌ |
| **GPIO IRQ Latency** | ✅ | ❌ | ❌ |

### Userspace vs. Kernel IIO Driver

While the mainline Linux kernel includes an IIO driver (`drivers/iio/imu/bno055`) starting from Linux 6.1, `libbno055-linux` is implemented as a **Userspace Driver** to address specific integration requirements in robotics (e.g., ROS 2) and rapid prototyping:

1. **Zero-Configuration Setup**: No need to compile kernel modules or write Device Tree Overlays (DTO). Installable via `pip install`, `cargo add`, or `apt install`, and operates on any standard I2C/UART port.
2. **`I2C_RDWR` Utilization**: Uses `ioctl(I2C_RDWR)` for Repeated Start burst-reading. It achieves low latency (~450µs for 18-bytes) in userspace by reducing system calls.
3. **Interrupt Driven**: Uses Linux `poll()` on sysfs GPIOs to wait for hardware INT pin edges in a background thread, achieving CPU-free idling similar to kernel-triggered buffers.
4. **Native ROS 2 Integration**: Directly publishes Zero-Copy `sensor_msgs/Imu`, avoiding the overhead of parsing multiple kernel sysfs text files (`in_accel_x_raw`, etc.) in a polling loop.

---

## Features

- **Polyglot Bindings**: First-class support for C++17, ROS 2, Python (`pip`), Rust (`crates.io`), and native C.
- **Zero-Allocation Hot Path**: Memory-optimized, no-heap allocations during high-rate sensor readouts.
- **Robust Hardware Recovery**: Auto-reconnects on I2C `EIO` bus lockups, clock-stretching glitches, and UART overrun errors.
- **High-Throughput Burst Reads**: 18-Byte sequential I2C/UART burst reads (~450µs at 400kHz I2C).
- **Linux GPIO Interrupt (IRQ)**: Sub-millisecond latency via hardware INT pin edge detection using POSIX `poll()`.

---

## Best Practices for Calibration in Production (Robotics)

In real-world robotics (e.g., drones, autonomous rovers), blocking a high-speed control loop to read calibration status or save files to a disk can cause severe jitter and crashes. To avoid this "deadlock" and ensure real-time performance, `libbno055-linux` recommends the following **Pro Workflow**:

1. **Disable Auto-Save in Flight**: Do not use `enableAutoCalibration()` during actual autonomous operation. Saving to the Linux filesystem can block the I2C bus and the reading thread for milliseconds.
2. **Use a Master Calibration File**: 
   - **Offline (Maintenance)**: During maintenance, physically calibrate the robot (figure-8 motion). Once fully calibrated (`isFullyCalibrated() == true`), manually trigger `saveCalibrationFile("master_calib.bin")`. In ROS 2, you can do this by calling the `~/save_calibration` service.
   - **Online (Production)**: On startup, simply load the master file using `loadCalibrationFile("master_calib.bin")`. The sensor will use this as a highly accurate baseline.
3. **Never Block on Calibration**: After loading the file, wait a few seconds for the gyroscope to stabilize (which happens automatically while stationary), and immediately start your control loop using `startInterruptDrivenReading`. Do not wait for the magnetometer to reach a status of `3`, as local magnetic interference might prevent it.

---

## Quick Start by Language

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

## Documentation

- **[Sphinx API Documentation (Web)](https://lazytatzv.github.io/libbno055-linux/)**: Comprehensive, auto-generated web documentation for C++ and Python APIs.
- **[API Reference (Markdown)](docs/API_REFERENCE.md)**: Full class and function reference for C++, C, Python, and Rust.
- **[Integration & Tuning Guide](docs/INTEGRATION.md)**: ROS 2 YAML parameters, EKF setup, 400kHz I2C, UART 921600 bps tuning, and Rust integration.
- **[Architecture & Design](docs/ARCHITECTURE.md)**: PIMPL design, zero-copy transport, FFI layers, and state machines.
- **[Troubleshooting & FAQ](docs/TROUBLESHOOTING.md)**: Hardware wiring, permissions, and clock-stretching fixes.

---

## License

This project is released under the [MIT License](LICENSE).
