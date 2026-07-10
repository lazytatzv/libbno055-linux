# Architecture and Design Decisions

The `libbno055-linux` library was built from the ground up for use in production-grade robotics. This document outlines the core technical decisions that differentiate this library from hobbyist Arduino ports.

## 1. Zero-Allocation & Exception-Free Hot Paths
In hard real-time systems (such as robot balancers or drone flight controllers), latency spikes caused by heap allocation or exception unwinding are unacceptable.
- **Fixed-size Stack Buffers**: All I2C read/write operations utilize pre-allocated `uint8_t` stack arrays (e.g., `uint8_t[32]`). We strictly avoid `std::vector` inside the sensor read loops.
- **Noexcept API**: The hot-path retrieval functions (e.g., `getQuaternionNoexcept()`) are marked `noexcept` and return `std::optional<T>` rather than throwing `std::runtime_error`. This guarantees deterministic execution time.

## 2. The PIMPL Idiom (Pointer to Implementation)
To guarantee Application Binary Interface (ABI) stability across different ROS 2 workspaces and compiler versions, the library utilizes the PIMPL idiom.
- **Fast Compilation**: Consumers of `bno055.hpp` do not need to include `<linux/i2c-dev.h>`, `<mutex>`, or any other system headers. 
- **Encapsulation**: All internal file descriptors, mutexes, and mock states are entirely hidden within `bno055.cpp`.

## 3. I2C Clock Stretching & Auto-Recovery
The Bosch BNO055 has a known hardware quirk: it heavily utilizes I2C clock stretching, which Linux I2C drivers (especially on the Broadcom SoC used in Raspberry Pi) occasionally fail to handle correctly, leading to bus lockups.
- **Diagnostic Telemetry**: The library tracks `read_failures`, `write_failures`, and `reconnect_attempts`.
- **Transparent Reconnection**: If the kernel returns `EIO` (Input/Output Error), the library safely closes the file descriptor, flushes the bus, re-opens the connection, and re-initializes the sensor mode. This happens transparently within milliseconds, preventing node crashes.

## 4. Thread Safety
A single `BNO055` instance might be read by a high-frequency control thread while simultaneously being polled by a low-frequency telemetry/diagnostics thread.
- All internal I2C operations are protected by a `std::mutex` ensuring atomic register accesses.

## 5. Cross-Platform Mocking
For Continuous Integration (CI) and offline development, the library automatically falls back to a Mock Mode when compiled on non-Linux platforms (Windows, macOS) or when the target I2C device does not exist. This allows developers to compile and test their ROS 2 nodes on their MacBooks without physical hardware.
