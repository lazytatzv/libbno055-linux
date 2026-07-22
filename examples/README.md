# Examples Overview & Directory Guide

This directory contains executable code examples showcasing how to use `libbno055-linux` across various use cases and languages.

## Summary of Examples

| File | Language | Purpose & Features |
| :--- | :---: | :--- |
| **`heading_control_demo.cpp`** | C++17 | **[Recommended]** 100Hz Ultra-Performance Straight-Line PID Heading Controller with zero-allocation ASCII dashboard. |
| **`read_all_data.cpp`** | C++17 | Interactive dashboard reading all physical vectors (Accel, Gyro, Mag, Euler, Linear Accel, Gravity, Quaternion, Temp). |
| **`read_data_noexcept.cpp`** | C++17 | Non-throwing `noexcept` API usage pattern suitable for hard real-time / safety-critical systems. |
| **`calibrate.cpp`** | C++17 | Interactive sensor calibration helper & status logger. |
| **`benchmark_imu.cpp`** | C++17 | Low-latency I2C/UART bus throughput and read timing benchmark tool. |
| **`c_demo.c`** | C99 | C FFI API binding usage example for legacy C codebases. |
| **`python_demo.py`** | Python 3 | High-level Python binding (`libbno055`) example with exception handling. |

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

# Run latency benchmark
./benchmark_imu /dev/i2c-1
```

### Python Example

```bash
pip install .
python3 examples/python_demo.py
```
