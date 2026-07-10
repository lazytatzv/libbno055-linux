# libbno055-linux

![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)
![ROS 2](https://img.shields.io/badge/ROS%202-Compatible-22314E.svg)
![Linux](https://img.shields.io/badge/OS-Linux-FCC624.svg)
![CMake](https://img.shields.io/badge/CMake-3.10%2B-064F8C.svg)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue.svg)](https://lazytatzv.github.io/libbno055-linux/)
![CI](https://github.com/lazytatzv/libbno055-linux/actions/workflows/ci.yml/badge.svg)
![Repo Size](https://img.shields.io/github/repo-size/lazytatzv/libbno055-linux.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

A robust, thread-safe, and dependency-free C++17 library for the BNO055 sensor over I2C on Linux.

**[View the Official Web Documentation (API, Architecture, Integration Guides)](https://lazytatzv.github.io/libbno055-linux/)**

Designed for robotic control systems, autonomous vehicles, and ROS 2 deployments that demand high reliability, automatic error recovery, and deterministic (noexcept) execution.

<details>
<summary><strong>Table of Contents</strong></summary>

- [Why libbno055-linux?](#why-libbno055-linux)
- [Compatibility](#compatibility)
- [Quick Start](#quick-start)
- [Sensor Overview and Operation Modes](#sensor-overview-and-operation-modes)
- [Prerequisites (Linux / Raspberry Pi Setup)](#prerequisites-linux--raspberry-pi-setup)
- [Build & Install (System-wide)](#build--install-system-wide)
- [Integration (CMake & ROS 2)](#integration-cmake--ros-2)
- [Running the Examples](#running-the-examples)
- [Usage Examples](#usage-examples)
- [API Reference](#api-reference)
- [License](#license)

</details>

## Compatibility

*   **C++ Version**: C++17 or newer.
*   **ROS 2 Distributions**: Compatible with all ROS 2 active and LTS distributions (including Foxy, Humble, Iron, Jazzy, and Rolling) as a pure CMake package built via colcon.
    *   *Note on Older ROS 2 Distributions (e.g., Foxy, Galactic)*: These older distributions default to C++14. You must explicitly enable C++17 in your consuming ROS 2 package's `CMakeLists.txt` by adding `set(CMAKE_CXX_STANDARD 17)` to avoid compilation failures.
*   **Operating Systems**: Linux (e.g., Ubuntu, Raspberry Pi OS) for hardware execution. macOS and Windows are supported for compilation and simulation via the built-in I2C mock mode.

---

## Why libbno055-linux?

Most existing BNO055 libraries for Linux are either simple ports of Arduino code or lack the robustness required for production robotics. This library is designed differently:

*   **Self-Healing (Auto-Recovery)**: The BNO055 on Raspberry Pi is notorious for I2C lockups due to hardware clock stretching. Existing libraries crash; this library transparently catches `EIO` faults, flushes the bus, and automatically initiates a recovery sequence to restore sensor state.
*   **Real-Time Safe**: Zero heap allocations (`std::vector` is avoided in hot paths) and exception-free (`noexcept`) APIs guarantee bounded, zero-jitter execution for ROS 2 hardware controllers.
*   **Modern & Native**: Written in pure C++17. No Arduino `Wire.h` hacks, no heavy frameworks, and zero external dependencies.
*   **CI/CD Ready**: Includes an automatic Mock Mode. You can compile your ROS 2 nodes on macOS or Windows laptops without physical I2C hardware.

---

## Quick Start

Whether you are building a standard C++ application or a ROS 2 robot controller, here is how to get the IMU data streaming in less than 5 minutes.

### A. No-ROS (Pure C++ / Raspberry Pi)

Follow these steps to run a standalone C++ program using a system installation of the library.

#### 1. Build and Install the Library
```bash
sudo apt update && sudo apt install -y build-essential cmake
git clone https://github.com/lazytatzv/libbno055-linux.git
cd libbno055-linux && mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF
make -j$(nproc)
sudo make install
```

#### 2. Write your code (`main.cpp`)
```cpp
#include <libbno055-linux/bno055.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // 0x28 is default I2C address, /dev/i2c-1 is default RPi I2C port
    bno055lib::BNO055 imu(0x28, "/dev/i2c-1");

    // Initialize in NDOF fusion mode (accelerometer + gyroscope + magnetometer)
    if (!imu.begin(bno055lib::OpMode::NDOF)) {
        std::cerr << "Failed to find BNO055 sensor over I2C!" << std::endl;
        return 1;
    }

    std::cout << "Streaming BNO055 Quaternion orientation (w, x, y, z)..." << std::endl;
    for (int i = 0; i < 20; ++i) {
        auto q = imu.getQuaternionOrDefault();
        std::cout << "w: " << q.w << " | x: " << q.x << " | y: " << q.y << " | z: " << q.z << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
```

#### 3. Compile and Run
```bash
g++ -std=c++17 main.cpp -lbno055-linux -o imu_demo
./imu_demo
```

---

### B. ROS 2 (colcon workspace)

Follow these steps to integrate and run the BNO055 publisher nodes directly inside your robot's ROS 2 workspace.

#### 1. Setup and Build Workspace
Make sure your ROS 2 environment is sourced (e.g., Humble, Jazzy):
```bash
# 1. Navigate to your ROS 2 workspace src directory
cd ~/ros2_ws/src

# 2. Clone this library directly inside src
git clone https://github.com/lazytatzv/libbno055-linux.git

# 3. Build using colcon
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select libbno055_linux
source install/setup.bash
```

#### 2. Launch the Nodes

You can launch and configure the nodes using the provided launch file and parameters YAML file.

##### Method A: Launching via Launch File (Recommended)
You can launch any of the three node types (`standard`, `perf`, or `lifecycle`) and customize their parameters using the ROS 2 Launch system.

*   **Standard Node (Default)**:
    ```bash
    ros2 launch libbno055_linux bno055_launch.py node_type:=standard
    ```
*   **High-Performance Node**:
    ```bash
    ros2 launch libbno055_linux bno055_launch.py node_type:=perf
    ```
*   **Lifecycle Node**:
    ```bash
    ros2 launch libbno055_linux bno055_launch.py node_type:=lifecycle
    ```

To customize parameters, copy and edit the installed template `config/bno055_params.yaml`, then load it via launch:
```bash
ros2 launch libbno055_linux bno055_launch.py params_file:=/path/to/your/custom_params.yaml
```

##### Method B: Direct Command Line (Alternative)
*   **Standard Standalone Node**:
    ```bash
    ros2 run libbno055_linux bno055_publisher_node
    ```
*   **High-Performance Zero-Copy Node**:
    ```bash
    ros2 run libbno055_linux bno055_perf_publisher_node
    ```
*   **Lifecycle Managed Node**:
    ```bash
    # Terminal 1: Run the node
    ros2 run libbno055_linux bno055_lifecycle_publisher_node
    
    # Terminal 2: Trigger state transitions to start publishing
    ros2 lifecycle set /bno055_lifecycle_publisher_node configure
    ros2 lifecycle set /bno055_lifecycle_publisher_node activate
    ```

#### 3. Verify Topic Stream
```bash
ros2 topic echo /imu/data
```

---

## Sensor Overview and Operation Modes

Detailed information about the BNO055's fusion modes (NDOF, IMUPlus, etc.), orientation formats (Quaternions vs Euler Angles), and calibration procedures has been moved to a separate document.

Please see the [Sensor Overview & Calibration Guide](docs/SENSOR_OVERVIEW.md) for full details.

---

## Prerequisites (Linux / Raspberry Pi Setup)

Before using the library, you must wire the sensor, enable the I2C interface, and ensure your user has permissions.

### 1. Hardware Wiring (Raspberry Pi)

Connect the BNO055 sensor to the Raspberry Pi's standard I2C pins:

| BNO055 Pin | Raspberry Pi Pin | Description |
| :--- | :--- | :--- |
| **Vin** | `3.3V` (Pin 1 or 17) | Power supply (do not use 5V if the breakout lacks a regulator) |
| **GND** | `GND` (Pin 6 or 9) | Ground |
| **SDA** | `GPIO 2` (Pin 3) | I2C Data |
| **SCL** | `GPIO 3` (Pin 5) | I2C Clock |
| **ADR** (Optional) | `GND` (or open) | Sets I2C Address to `0x28` (Default). Tie to `3.3V` for `0x29`. |

### 2. Enable I2C
On Raspberry Pi OS:
1. Run sudo raspi-config.
2. Navigate to Interface Options -> I2C and select Yes to enable it.
3. Reboot your Raspberry Pi.

Alternatively, add/uncomment the following line in /boot/config.txt (or /boot/firmware/config.txt on newer OS versions) and reboot:
```text
dtparam=i2c_arm=on
```

### 2. Set Permissions
By default, access to I2C devices (/dev/i2c-*) requires root privileges. To run your program as a non-root user, add your user to the i2c group:
```bash
sudo usermod -aG i2c $USER
```
*Note: You must log out and log back in for the group changes to take effect.*

---

## Build & Install (System-wide)

```bash
mkdir build && cd build
cmake ..
make
sudo make install
```

---

## Integration (CMake & ROS 2)

### Standard CMake Integration (With System Installation)
If installed globally on your system, find and link the library in your CMakeLists.txt:

```cmake
find_package(libbno055-linux REQUIRED)
target_link_libraries(your_target PRIVATE libbno055-linux::libbno055-linux)
```

Include in your C++ code:
```cpp
#include <libbno055-linux/bno055.hpp>
```

### Local CMake Integration (Without System Installation)
If you do not want to install the library system-wide, integrate it locally:

#### Method A: add_subdirectory
Place the libbno055-linux directory inside your project (e.g., under third_party/) and add it in your CMakeLists.txt:
```cmake
add_subdirectory(third_party/libbno055-linux)
target_link_libraries(your_target PRIVATE libbno055-linux)
```

#### Method B: FetchContent (CMake 3.11+)
```cmake
include(FetchContent)

FetchContent_Declare(
  libbno055-linux
  GIT_REPOSITORY https://github.com/lazytatzv/libbno055-linux.git
  GIT_TAG        main
)
FetchContent_MakeAvailable(libbno055-linux)

target_link_libraries(your_target PRIVATE libbno055-linux)
```

### ROS 2 (colcon) Integration
Place the libbno055-linux directory directly inside your ROS 2 workspace's src folder. colcon will build it as a pure CMake package.

**Workspace Directory Structure:**
```text
your_ros2_ws/
└── src/
    ├── libbno055-linux/   # <--- Place this library here (Pure CMake package)
    │   ├── CMakeLists.txt
    │   └── ...
    └── your_ros_package/  # Your ROS 2 node package (Ament CMake package)
        ├── CMakeLists.txt
        ├── package.xml
        └── ...
```

To use it from another ROS 2 package:
1. Add dependency to package.xml:
   ```xml
   <depend>libbno055-linux</depend>
   ```
2. Find and link in CMakeLists.txt:
   ```cmake
   find_package(libbno055-linux REQUIRED)
   ament_target_dependencies(your_node_target libbno055-linux)
   ```

---

## Running the Examples

After building the project, you can run the compiled example binaries directly from the build directory.

### 1. Read All Sensor Data (Standard API)
Displays all 8 physical data types and calibration status at 10Hz.
```bash
./build/read_all_data
```
To specify a different I2C device (e.g., /dev/i2c-0) and/or operation mode (e.g., imu, amg, gyro):
```bash
./build/read_all_data /dev/i2c-0 imu
```
(Supported modes: ndof, imu, amg, gyro. Default is ndof. Selecting "imu" enables IMUPlus mode, which runs 6-axis sensor fusion without the magnetometer, avoiding orientation drift and distortion in indoor environments with magnetic interference.)

### 2. High-Frequency Real-time Loop (Exception-free API)
Reads orientation, angular velocity, and linear acceleration at 20Hz, displays telemetry diagnostics, and safely suspends the sensor on Ctrl+C.
```bash
./build/read_data_noexcept
```

### 3. Calibration Utility
Runs the interactive calibration utility to align the accelerometer, gyroscope, and magnetometer, and saves the calibrated offsets to a binary file.
```bash
./build/calibrate_imu /dev/i2c-1 bno055_calib.bin
```

### 4. ROS 2 Publisher Nodes (Optional)
If a ROS 2 environment is detected during compilation, the library compiles three distinct ROS 2 publisher nodes to support different system integration designs:

*   **Standard Standalone Node (`bno055_publisher_node`)**:
    ```bash
    ros2 run libbno055_linux bno055_publisher_node --ros-args -p device:="/dev/i2c-1" -p publish_rate:=50.0
    ```
*   **High-Performance Zero-Copy Node (`bno055_perf_publisher_node`)**: Optimized using `std::unique_ptr` and `std::move` to allow zero-copy intra-process message passing when composed.
    ```bash
    ros2 run libbno055_linux bno055_perf_publisher_node
    ```
*   **Managed Lifecycle Node (`bno055_lifecycle_publisher_node`)**: Integrates with ROS 2 Lifecycle management to handle state transitions (configure, activate, deactivate, cleanup) and syncs hardware low-power states.
    ```bash
    ros2 run libbno055_linux bno055_lifecycle_publisher_node
    ```

### Troubleshooting Permission Denied
If you see "Failed to open I2C device" or permission errors, run with sudo or ensure your user belongs to the i2c group as described in the Prerequisites section:
```bash
sudo ./build/read_all_data
```

---

## Usage Examples

### 1. Robust Real-time Loop (Exceptions-Free & Telemetry)
Highly recommended for real-time controllers (like ROS 2 control nodes) where exceptions are prohibited.

```cpp
#include <libbno055-linux/bno055.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // 1. Initialize sensor (uses Mock mode automatically on non-Linux platforms)
    bno055lib::BNO055 imu(0x28, "/dev/i2c-1");

    if (!imu.begin(bno055lib::OpMode::NDOF)) {
        std::cerr << "Initialization failed!" << std::endl;
        return 1;
    }

    std::cout << "IMU started successfully in exception-free mode." << std::endl;

    while (true) {
        // 2. Fetch data without exceptions using Noexcept API
        if (auto gyro = imu.getGyroscopeNoexcept()) {
            std::cout << "\rGyro: X=" << gyro->x << " Y=" << gyro->y << " Z=" << gyro->z << std::flush;
        } else {
            std::cerr << "\n[Warning] Temporary communication dropout." << std::endl;
        }

        // 3. Monitor Telemetry / Diagnostics periodically
        static int loops = 0;
        if (++loops % 20 == 0) {
            auto diag = imu.getDiagnostics();
            if (diag.reconnect_attempts > 0) {
                std::cout << "\n[DIAG] I2C errors: RxErr=" << diag.read_failures 
                          << ", TxErr=" << diag.write_failures 
                          << ", Reconnects=" << diag.reconnect_attempts << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return 0;
}
```

### 2. Quaternion Retrieval (With Exception Handling)
```cpp
#include <libbno055-linux/bno055.hpp>
#include <iostream>

int main() {
    bno055lib::BNO055 imu(0x28, "/dev/i2c-1");

    // Standard 6-axis IMU mode for indoor robotics
    if (!imu.begin(bno055lib::OpMode::IMUPlus)) {
        std::cerr << "Initialization failed!" << std::endl;
        return 1;
    }

    imu.loadCalibrationFile("bno055_calib.bin");

    try {
        // Read unit quaternion (w, x, y, z) for 3D orientation.
        // Directly compatible with ROS 2 geometry_msgs/msg/Quaternion.
        auto quat = imu.getQuaternion(); // Throws bno055lib::IMUError on permanent I2C loss
        
        std::cout << "Quaternion orientation: "
                  << "w=" << quat.w 
                  << " x=" << quat.x 
                  << " y=" << quat.y 
                  << " z=" << quat.z << std::endl;
                  
    } catch (const bno055lib::IMUError& e) {
        std::cerr << "Sensor read failed: " << e.what() << std::endl;
    }

    return 0;
}
```

### 3. Beginner-Friendly Integration (No Optionals, No Exceptions)
Designed for hobbyists, students, or rapid prototyping where you want to read data with minimal code.

```cpp
#include <libbno055-linux/bno055.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    bno055lib::BNO055 imu(0x28, "/dev/i2c-1");

    // Simple initialization
    if (!imu.begin(bno055lib::OpMode::IMUPlus)) {
        std::cerr << "Sensor not found." << std::endl;
        return 1;
    }

    while (true) {
        // Read directly without try-catch or std::optional handling.
        // Returns the last cached valid value (or 0) if a temporary I2C dropout occurs.
        bno055lib::Quaternion quat = imu.getQuaternionOrDefault();
        
        // Convert to human-readable Euler angles in degrees
        bno055lib::Vector3 euler = bno055lib::toEulerDegrees(quat);
        
        std::cout << "Roll: " << euler.x 
                  << " | Pitch: " << euler.y 
                  << " | Yaw: " << euler.z << " degrees" << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return 0;
}
```

---

## API Reference

The full API reference has been moved to a separate document. Please see [API_REFERENCE.md](docs/API_REFERENCE.md) for complete details on classes, structs, and available functions.


## License

MIT License.
