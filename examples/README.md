# Examples Overview & Directory Guide

This directory contains application examples categorized by language and framework, clearly decoupled from the core driver engine.

## Directory Structure

```text
examples/
├── cpp/       # Native C++17 application examples
├── ros2/      # ROS 2 application & control nodes
├── c/         # C99 API examples
└── python/    # Python 3 binding examples
```

## Summary of Examples

| File | Subdirectory | Language | Purpose & Features |
| :--- | :--- | :---: | :--- |
| **`heading_control_demo.cpp`** | `cpp/` | C++17 | **[Recommended]** 100Hz Ultra-Performance Straight-Line PID Heading Controller with zero-allocation ASCII dashboard. |
| **`read_all_data.cpp`** | `cpp/` | C++17 | Interactive dashboard reading all physical vectors (Accel, Gyro, Mag, Euler, Linear Accel, Gravity, Quaternion, Temp). |
| **`read_data_noexcept.cpp`** | `cpp/` | C++17 | Non-throwing `noexcept` API usage pattern suitable for hard real-time / safety-critical systems. |
| **`calibrate.cpp`** | `cpp/` | C++17 | Interactive sensor calibration helper & status logger. |
| **`benchmark_imu.cpp`** | `cpp/` | C++17 | Low-latency I2C/UART bus throughput and read timing benchmark tool. |
| **`ros2_heading_control_node.cpp`** | `ros2/` | ROS 2 C++ | ROS 2 node subscribing to `/imu/data` and publishing `/cmd_vel` for heading alignment. |
| **`c_demo.c`** | `c/` | C99 | C FFI API binding usage example for legacy C codebases. |
| **`python_demo.py`** | `python/` | Python 3 | High-level Python binding (`libbno055`) example with exception handling. |

---

## How to Build & Run Examples

### C++ / C Examples (CMake)

```bash
mkdir -p build && cd build
cmake -DENABLE_CLANG_TIDY=OFF ..
make -j$(nproc)

# Run the 100Hz Heading PID Control Demo
./heading_control_demo /dev/i2c-1

# Run the full sensor dashboard
./read_all_data /dev/i2c-1
```

### Python Example

```bash
pip install .
python3 examples/python/python_demo.py
```
