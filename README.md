# libbno055-linux

A robust, thread-safe, and dependency-free C++17 library for the BNO055 sensor over I2C on Linux.

Designed for robotic control systems, autonomous vehicles, and ROS 2 deployments that demand high reliability, automatic error recovery, and zero-latency (noexcept) capabilities.

## Compatibility

*   **C++ Version**: C++17 or newer.
*   **ROS 2 Distributions**: Compatible with all ROS 2 active and LTS distributions (including Foxy, Humble, Iron, Jazzy, and Rolling) as a pure CMake package built via colcon.
    *   *Note on Older ROS 2 Distributions (e.g., Foxy, Galactic)*: These older distributions default to C++14. You must explicitly enable C++17 in your consuming ROS 2 package's `CMakeLists.txt` by adding `set(CMAKE_CXX_STANDARD 17)` to avoid compilation failures.
*   **Operating Systems**: Linux (e.g., Ubuntu, Raspberry Pi OS) for hardware execution. macOS and Windows are supported for compilation and simulation via the built-in I2C mock mode.

---

## Key Features

*   **Thread-Safe**: Safe concurrent access to the IMU from multiple threads or asynchronous control loops.
*   **State-Preserving Auto-Recovery**: Detects I2C drops, automatically reconnects, and restores all configuration (Axis Remaps, Calibration Offsets, Ext-Crystal selection, and Unit setup).
*   **Performance Optimized**: Uses sequential burst-writes (writeLen) for uploading calibration offsets in a single batch, reducing bus overhead.
*   **Zero-Latency API (noexcept)**: Companion non-throwing APIs returning std::optional to avoid memory/CPU overhead of C++ exceptions in real-time execution loops.
*   **Hardware Diagnostics**: Real-time telemetry tracking cumulative read failures, write failures, and hardware reconnect attempts.
*   **Cross-Platform Compatibility (Mock Mode)**: Automatically falls back to I2C mocks on macOS/Windows, allowing software compilation and CI/CD validation without access to physical hardware.

---

## Sensor Overview and Operation Modes

The Bosch BNO055 is a System-in-Package (SiP) integrating a triaxial accelerometer, a triaxial gyroscope, a triaxial geomagnetic sensor, and a 32-bit ARM Cortex-M0 microcontroller running Bosch Sensortec sensor fusion software. 

Selecting the appropriate Operation Mode (OpMode) is critical for the stability of your estimation loops.

### Key Fusion Modes

*   **NDOF (9-DoF Fusion)**: Uses accelerometer, gyroscope, and magnetometer. It outputs absolute orientation relative to the Earth's magnetic field (Yaw is referenced to magnetic North). This mode is suitable for outdoor navigation but highly susceptible to magnetic interference (distortion from iron structures, electric motors, or wiring).
*   **IMUPlus (6-DoF Fusion)**: Uses accelerometer and gyroscope only. It outputs relative orientation (Yaw starts at 0 on boot and will slowly drift over time). This mode is highly recommended for indoor robotics, autonomous mobile robots (AMRs), and industrial environments where magnetic disturbances are constant.
*   **AMG (Non-fusion Raw Mode)**: Bypasses the internal fusion processor and outputs raw sensor readings from the Accelerometer, Magnetometer, and Gyroscope. Use this mode if you intend to implement custom state estimation filters (such as EKF or complementary filters) on the host CPU.

### Orientation Formats (Quaternion and Euler Angles)

The library provides two formats for retrieving the 3D orientation computed by the sensor:

*   **Quaternions (De-facto Standard for Robotics)**: Highly recommended for robotics applications (such as ROS 2 navigation, TF2 transforms, and state estimation) to avoid gimbal lock. The BNO055 internal fusion coprocessor computes unit quaternions (w, x, y, z) at 100Hz. The library automatically normalizes this data to a unit quaternion format, making it directly compatible with ROS 2 geometry_msgs/msg/Quaternion and sensor_msgs/msg/Imu messages.
*   **Euler Angles**: Convenient for human-readable display or simpler projects. The library returns Roll, Pitch, and Yaw in radians via a Vector3 struct (where x = Roll, y = Pitch, and z = Yaw).

### Sensor Calibration

The BNO055 calibrates itself dynamically in the background. The calibration status for each sensor ranges from 0 (uncalibrated) to 3 (fully calibrated). To achieve full calibration:
1.  **Gyroscope**: Keep the sensor completely still in a stable position for a few seconds.
2.  **Magnetometer**: Move the sensor in a figure-8 pattern through the air.
3.  **Accelerometer**: Rotate the sensor into 6 different stable positions, holding it still for a few seconds in each orientation (similar to placing a cube on each of its 6 faces).

---

## Prerequisites (Linux / Raspberry Pi Setup)

Before using the library, you must enable the I2C interface on your Linux device (such as a Raspberry Pi) and ensure your user has permissions to access it.

### 1. Enable I2C
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

### 4. ROS 2 Publisher Node (Optional)
If a ROS 2 environment is detected during compilation, the library compiles a standalone ROS 2 publisher node that reads IMU data and publishes it to the `/imu/data` topic as a `sensor_msgs/msg/Imu` message.
```bash
./build/bno055_publisher_node --ros-args -p device:="/dev/i2c-1" -p publish_rate:=50.0
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

### Namespaces & Types

```cpp
namespace bno055lib {
    // 3D coordinate vector (used for accelerometer, gyroscope, magnetometer, euler, gravity, linear accel)
    struct Vector3 {
        double x;
        double y;
        double z;
    };

    // 3D orientation quaternion representation
    struct Quaternion {
        double w; // Real part
        double x; // Imaginary X
        double y; // Imaginary Y
        double z; // Imaginary Z
    };
    
    // Binary calibration offsets (22 bytes total) for saving/restoring sensor profile
    struct Offsets {
        int16_t accel_offset_x, accel_offset_y, accel_offset_z;
        int16_t mag_offset_x, mag_offset_y, mag_offset_z;
        int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;
        int16_t accel_radius, mag_radius;
    };

    // Dynamic calibration status of the sensor (0 = uncalibrated, 3 = fully calibrated)
    struct CalibrationStatus {
        uint8_t sys;   // Overall system calibration status [0-3]
        uint8_t gyro;  // Gyroscope calibration status [0-3]
        uint8_t accel; // Accelerometer calibration status [0-3]
        uint8_t mag;   // Magnetometer calibration status [0-3]
        
        // Returns true if gyro, accel, and mag are fully calibrated (status == 3)
        bool isFullyCalibrated() const;
    };

    // Cumulative telemetry diagnostics tracking error rates for health monitoring
    struct Diagnostics {
        uint32_t write_failures;      // Number of failed register write transactions
        uint32_t read_failures;       // Number of failed register read transactions
        uint32_t reconnect_attempts;  // Number of I2C bus auto-reconnect triggers
    };

    // Operating mode configuration
    enum class OpMode : uint8_t {
        Config = 0X00,          // Configuration mode (required to write map/sign/crystal settings)
        AccOnly = 0X01,         // Non-fusion Accelerometer only
        MagOnly = 0X02,         // Non-fusion Magnetometer only
        GyroOnly = 0X03,        // Non-fusion Gyroscope only
        AccMag = 0X04,          // Non-fusion Accelerometer + Magnetometer
        AccGyro = 0X05,         // Non-fusion Accelerometer + Gyroscope
        MagGyro = 0X06,         // Non-fusion Magnetometer + Gyroscope
        AMG = 0X07,             // Non-fusion Accelerometer + Magnetometer + Gyroscope (raw outputs)
        IMUPlus = 0X08,         // 6-axis Fusion (Acc + Gyro). Yaw relative to boot position. Recommended indoors.
        Compass = 0X09,         // 6-axis Fusion (Acc + Mag). Absolute Yaw.
        M4G = 0X0A,             // 6-axis Fusion (Mag + Gyro).
        NDOF_FMC_Off = 0X0B,    // 9-axis Fusion (Acc + Mag + Gyro) with Fast Magnetometer Calibration disabled
        NDOF = 0X0C             // 9-axis Fusion (Acc + Mag + Gyro) with FMC enabled. Absolute Yaw (North-referenced).
    };

    enum class LogLevel { Debug, Info, Warning, Error };
    using LoggerCallback = std::function<void(LogLevel level, std::string_view message)>;
}
```
### Class BNO055

All functions in the `BNO055` class are thread-safe and protect access to the underlying I2C bus using internal mutexes.

#### Lifecycle

*   **explicit BNO055(uint8_t i2c_address = 0x28, std::string_view i2c_device = "/dev/i2c-1")**
    *   *Parameters*:
        *   `i2c_address`: `uint8_t`. The I2C slave address of the BNO055 (typically `0x28` or `0x29`).
        *   `i2c_device`: `std::string_view`. The Linux file path to the I2C adapter (e.g., `/dev/i2c-1`).
    *   *Returns*: None (Constructor).
    *   *Description*: Creates a BNO055 handler. If compiled on macOS/Windows, it falls back to the mock mode and accepts any mock device name.
*   **bool begin(OpMode mode = OpMode::NDOF)**
    *   *Parameters*:
        *   `mode`: `OpMode`. The target operation mode to start in.
    *   *Returns*: `bool`. `true` if initialization, self-test verification, and mode transition succeeded; `false` on boot timeouts or I2C failures.
    *   *Description*: Power-cycles the sensor, verifies the Chip ID, configures default Axis Maps and Unit settings, and spawns the background auto-recovery thread.

#### Configuration

*   **void setMode(OpMode mode)**
    *   *Parameters*:
        *   `mode`: `OpMode`. The target operation mode to switch into.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on permanent I2C communication failures.
    *   *Description*: Switches the operating mode of the sensor. Handles transition delay requirements automatically.
*   **OpMode getMode()**
    *   *Parameters*: None.
    *   *Returns*: `OpMode`. The active operation mode currently registered on the sensor.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.
*   **void setAxisRemap(AxisMapConfig config)**
    *   *Parameters*:
        *   `config`: `AxisMapConfig`. The remap configuration matching the sensor's mounting orientation.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.
*   **void setAxisSign(AxisMapSign sign)**
    *   *Parameters*:
        *   `sign`: `AxisMapSign`. The axis sign configuration to adjust rotation/acceleration directions.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.
*   **void setExtCrystalUse(bool use_xtal)**
    *   *Parameters*:
        *   `use_xtal`: `bool`. Set to `true` to use the external 32.768kHz crystal oscillator for enhanced fusion stability.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.

#### Sensor Data (Throwing APIs)

These functions query raw data registers and convert them to SI units. They throw `bno055lib::IMUError` on permanent I2C communication loss.

*   **Vector3 getAccelerometer()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis acceleration in meters per second squared (m/s^2).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getMagnetometer()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis magnetic field strength in microteslas (uT).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getGyroscope()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis angular velocity in radians per second (rad/s).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getEulerAngles()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. Roll, Pitch, Yaw in radians (rad). Mapping: `x` = Roll, `y` = Pitch, `z` = Yaw.
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getLinearAcceleration()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis linear acceleration (accelerometer excluding gravity) in m/s^2.
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Vector3 getGravity()**
    *   *Parameters*: None.
    *   *Returns*: `Vector3`. 3-axis gravity vector in m/s^2.
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **Quaternion getQuaternion()**
    *   *Parameters*: None.
    *   *Returns*: `Quaternion`. Normalized 3D orientation unit quaternion (w, x, y, z).
    *   *Exceptions*: Throws `bno055lib::IMUError`.
*   **int8_t getTemperature()**
    *   *Parameters*: None.
    *   *Returns*: `int8_t`. Chip temperature in degrees Celsius (C).
    *   *Exceptions*: Throws `bno055lib::IMUError`.

#### Sensor Data (Exception-free / noexcept APIs)

These companion APIs perform the exact same register queries and conversions but never throw exceptions.

*   **std::optional\<Vector3\> getAccelerometerNoexcept() noexcept**
*   **std::optional\<Vector3\> getMagnetometerNoexcept() noexcept**
*   **std::optional\<Vector3\> getGyroscopeNoexcept() noexcept**
*   **std::optional\<Vector3\> getEulerAnglesNoexcept() noexcept**
*   **std::optional\<Vector3\> getLinearAccelerationNoexcept() noexcept**
*   **std::optional\<Vector3\> getGravityNoexcept() noexcept**
*   **std::optional\<Quaternion\> getQuaternionNoexcept() noexcept**
*   **std::optional\<int8_t\> getTemperatureNoexcept() noexcept**
    *   *Parameters*: None.
    *   *Returns*: `std::optional<T>`. Contains the requested struct on success; `std::nullopt` on communication failure.
    *   *Description*: Safety-hardened read path that increments I2C diagnostic counters upon failure without generating CPU exceptions.

#### Sensor Data (Beginner-Friendly / OrDefault APIs)

These functions return sensor readings directly. They never throw exceptions and never return optionals. If an I2C communication failure occurs, they automatically return the last cached valid value (or zero/identity values on startup).

*   **Vector3 getAccelerometerOrDefault() noexcept**
*   **Vector3 getMagnetometerOrDefault() noexcept**
*   **Vector3 getGyroscopeOrDefault() noexcept**
*   **Vector3 getEulerAnglesOrDefault() noexcept**
*   **Vector3 getLinearAccelerationOrDefault() noexcept**
*   **Vector3 getGravityOrDefault() noexcept**
*   **Quaternion getQuaternionOrDefault() noexcept**
*   **int8_t getTemperatureOrDefault() noexcept**
    *   *Parameters*: None.
    *   *Returns*: The requested struct directly (`Vector3`, `Quaternion`, or `int8_t`). On temporary bus drops, returns the last cached valid frame (or zero/identity).

#### Diagnostics & Calibration

*   **Diagnostics getDiagnostics() const noexcept**
    *   *Parameters*: None.
    *   *Returns*: `Diagnostics`. The telemetry diagnostic struct containing I2C read/write error counts and reconnection attempts.
*   **CalibrationStatus getCalibrationStatus()**
    *   *Parameters*: None.
    *   *Returns*: `CalibrationStatus`. Current calibration state values (0 to 3) for the system, gyro, accelerometer, and magnetometer.
    *   *Exceptions*: Throws `bno055lib::IMUError` on I2C failure.
*   **bool getSensorOffsets(Offsets& offsets)**
    *   *Parameters*:
        *   `offsets`: `Offsets&` (output). Structure to store the retrieved calibration offsets.
    *   *Returns*: `bool`. `true` if offsets were read and stored successfully; `false` on I2C failure.
*   **bool getSensorOffsets(std::array<uint8_t, 22>& calib_data)**
    *   *Parameters*:
        *   `calib_data`: `std::array<uint8_t, 22>&` (output). Raw byte array to store the 22 bytes of offsets.
    *   *Returns*: `bool`. `true` if read succeeded; `false` on failure.
*   **void setSensorOffsets(const Offsets& offsets)**
    *   *Parameters*:
        *   `offsets`: `const Offsets&`. Calibration offset parameters to load into the sensor registers.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on failure.
    *   *Description*: Writes offsets using a single-batch sequential I2C write transaction to minimize bus occupation.
*   **void setSensorOffsets(const std::array<uint8_t, 22>& calib_data)**
    *   *Parameters*:
        *   `calib_data`: `const std::array<uint8_t, 22>&`. Raw 22-byte array containing calibration offsets.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on failure.
*   **bool saveCalibrationFile(std::string_view filepath)**
    *   *Parameters*:
        *   `filepath`: `std::string_view`. Destination file path to save the 22-byte profile (e.g., `calib.bin`).
    *   *Returns*: `bool`. `true` if offsets were read and successfully written to disk; `false` on I2C or file I/O errors.
*   **bool loadCalibrationFile(std::string_view filepath)**
    *   *Parameters*:
        *   `filepath`: `std::string_view`. Source path of the 22-byte binary profile.
    *   *Returns*: `bool`. `true` if file read, register upload, and cache storage succeeded; `false` on file I/O or I2C errors.
    *   *Description*: Uploads calibration to the sensor and caches it locally so it can be automatically restored during an I2C hot-reconnect recovery.

#### Power Management

*   **void enterSuspendMode()**
    *   *Parameters*: None.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on failure.
    *   *Description*: Places the sensor into suspend mode to reduce power consumption. Suspends accelerometer, gyroscope, and magnetometer blocks.
*   **void enterNormalMode()**
    *   *Parameters*: None.
    *   *Returns*: `void`.
    *   *Exceptions*: Throws `bno055lib::IMUError` on failure.
    *   *Description*: Awakes the sensor from suspend mode back to active normal operation.

#### Logging

*   **void setLogger(LoggerCallback callback)**
    *   *Parameters*:
        *   `callback`: `LoggerCallback`. Callback function of signature `void(LogLevel level, std::string_view message)`.
    *   *Returns*: `void`.
    *   *Description*: Hooks a custom logging function (such as `std::cout` or ROS 2 logging macros) to direct library diagnostics, warnings, and reconnect traces.

### Utilities (Class-External)

*   **Vector3 toEulerDegrees(const Quaternion& q) noexcept**
    *   *Parameters*:
        *   `q`: `const Quaternion&`. The normalized unit quaternion representation of orientation.
    *   *Returns*: `Vector3`. Roll, Pitch, and Yaw in degrees (Mapping: `x` = Roll `[-180, 180]`, `y` = Pitch `[-90, 90]`, `z` = Yaw `[0, 360)`).
    *   *Description*: Utility function to convert Quaternion orientation into human-readable Euler angles in degrees.

---

## License

MIT License.
