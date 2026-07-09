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
        *   `i2c_address`: The I2C address of the BNO055 sensor (typically `0x28` or `0x29`).
        *   `i2c_device`: The path to the Linux I2C device node (e.g., `/dev/i2c-1`).
    *   *Description*: Creates a BNO055 instance. If compiled on a non-Linux platform (like macOS or Windows), it automatically activates the built-in mock simulation mode using the virtual device name (e.g., `/dev/i2c-mock`).
*   **bool begin(OpMode mode = OpMode::NDOF)**
    *   *Parameters*:
        *   `mode`: The initial operating mode of the sensor.
    *   *Returns*: `true` on successful initialization; `false` on communication failure or boot timeout.
    *   *Description*: Power-cycles the sensor to software-reset it, waits for the system to boot (timeout 1000ms), verifies the chip ID, configures units to SI metrics (m/s^2, rad/s, Celsius), sets the default axis remap, and transitions into the specified operating mode. Also spawns the background auto-recovery thread.

#### Configuration
*   **void setMode(OpMode mode)**
    *   *Description*: Switches the operating mode. Temporarily enters CONFIG mode if necessary, writes the mode register, and waits the required datasheet transition time (30ms). Throws `IMUError` on communication loss.
*   **OpMode getMode()**
    *   *Returns*: The current operating mode read from the sensor's `OPR_MODE` register. Throws `IMUError` on communication loss.
*   **void setAxisRemap(AxisMapConfig config)**
    *   *Description*: Changes the axis mapping based on the mounting orientation of the sensor. Internally enters CONFIG mode to apply. Throws `IMUError` on communication loss.
*   **void setAxisSign(AxisMapSign sign)**
    *   *Description*: Modifies the axis signs (direction of positive rotation/acceleration) for custom orientations. Throws `IMUError` on communication loss.
*   **void setExtCrystalUse(bool use_xtal)**
    *   *Description*: Tells the sensor whether to use an external crystal oscillator. Highly recommended for accurate fusion integration. Throws `IMUError` on communication loss.

#### Sensor Data (Throwing APIs)
These functions query registers, convert the raw binary data to standard physical SI units, and return the structure. They throw `bno055lib::IMUError` if the bus communication fails permanently.

*   **Vector3 getAccelerometer()**
    *   *Units*: Meters per second squared (m/s^2).
*   **Vector3 getMagnetometer()**
    *   *Units*: Microteslas (uT).
*   **Vector3 getGyroscope()**
    *   *Units*: Radians per second (rad/s).
*   **Vector3 getEulerAngles()**
    *   *Units*: Radians (rad). Return order is `x` = Roll, `y` = Pitch, `z` = Yaw (Heading).
*   **Vector3 getLinearAcceleration()**
    *   *Units*: Meters per second squared (m/s^2). Acceleration of the sensor excluding gravity.
*   **Vector3 getGravity()**
    *   *Units*: Meters per second squared (m/s^2). Gravity acceleration vector.
*   **Quaternion getQuaternion()**
    *   *Units*: Normalized Unit Quaternion ($w^2 + x^2 + y^2 + z^2 \approx 1.0$).
*   **int8_t getTemperature()**
    *   *Units*: Degrees Celsius ($^\circ C$).

#### Sensor Data (Exception-free / noexcept APIs)
These functions provide the exact same functionality as the throwing APIs above but are declared `noexcept` and will **never throw exceptions**.

*   **std::optional\<Vector3\> getAccelerometerNoexcept() noexcept**
*   **std::optional\<Vector3\> getMagnetometerNoexcept() noexcept**
*   **std::optional\<Vector3\> getGyroscopeNoexcept() noexcept**
*   **std::optional\<Vector3\> getEulerAnglesNoexcept() noexcept**
*   **std::optional\<Vector3\> getLinearAccelerationNoexcept() noexcept**
*   **std::optional\<Vector3\> getGravityNoexcept() noexcept**
*   **std::optional\<Quaternion\> getQuaternionNoexcept() noexcept**
*   **std::optional\<int8_t\> getTemperatureNoexcept() noexcept**
    *   *Returns*: `std::optional` containing the data structure on success; `std::nullopt` on communication dropout.
    *   *Description*: Safely increments the internal `Diagnostics` counter if I2C failures occur.

#### Diagnostics & Calibration
*   **Diagnostics getDiagnostics() const noexcept**
    *   *Returns*: A copy of the current error/recovery counters.
*   **CalibrationStatus getCalibrationStatus()**
    *   *Returns*: The current system calibration status structure. Throws `IMUError` on communication loss.
*   **bool getSensorOffsets(Offsets& offsets)**
*   **bool getSensorOffsets(std::array<uint8_t, 22>& calib_data)**
    *   *Returns*: `true` if offsets were read successfully; `false` otherwise.
    *   *Description*: Reads the 22-byte calibration profiles (offsets and radiuses) from the sensor's EEPROM-like register bank.
*   **void setSensorOffsets(const Offsets& offsets)**
*   **void setSensorOffsets(const std::array<uint8_t, 22>& calib_data)**
    *   *Description*: Writes the 22-byte calibration profiles into the sensor. Internally optimized using single-batch I2C sequential writes (`writeLen`) to minimize bus occupation. Throws `IMUError` on failure.
*   **bool saveCalibrationFile(std::string_view filepath)**
    *   *Description*: Reads offsets from the sensor and saves them as a 22-byte binary profile to the host filesystem. Returns `true` on success.
*   **bool loadCalibrationFile(std::string_view filepath)**
    *   *Description*: Loads a 22-byte binary profile from the filesystem, writes it to the sensor, and saves it in the memory cache for automatic reconnect state restoration. Returns `true` on success.

#### Power Management
*   **void enterSuspendMode()**
    *   *Description*: Puts the sensor into low-power sleep mode (accelerometer, gyroscope, and magnetometer suspended).
*   **void enterNormalMode()**
    *   *Description*: Resumes the sensor to active operating mode.

#### Logging
*   **void setLogger(LoggerCallback callback)**
    *   *Description*: Registers a user-defined logging callback (such as logging to ROS 2 logger `RCLCPP_INFO` or `std::cout`) to capture internal warnings, debug events, and recovery traces.

---

## License

MIT License.
