# bno055lib

A robust, thread-safe, and dependency-free C++17 library for the BNO055 sensor over I2C on Linux.

---

## Prerequisites (Linux / Raspberry Pi Setup)

Before using the library, you must enable the I2C interface on your Linux device (such as a Raspberry Pi) and ensure your user has permissions to access it.

### 1. Enable I2C
On Raspberry Pi OS:
1. Run `sudo raspi-config`.
2. Navigate to **Interface Options** -> **I2C** and select **Yes** to enable it.
3. Reboot your Raspberry Pi.

Alternatively, add/uncomment the following line in `/boot/config.txt` (or `/boot/firmware/config.txt` on newer OS versions) and reboot:
```text
dtparam=i2c_arm=on
```

### 2. Set Permissions
By default, access to I2C devices (`/dev/i2c-*`) requires root privileges. To run your program as a non-root user, add your user to the `i2c` group:
```bash
sudo usermod -aG i2c $USER
```
*Note: You must log out and log back in for the group changes to take effect.*

---

## Build & Install

```bash
mkdir build && cd build
cmake ..
make
sudo make install
```

---

## Integration (CMake & ROS 2)

### Standard CMake Integration
If installed to your system, find and link the library in your `CMakeLists.txt`:

```cmake
find_package(bno055lib REQUIRED)
target_link_libraries(your_target PRIVATE bno055lib::bno055lib)
```

Include in your C++ code:
```cpp
#include <bno055lib/bno055.hpp>
```

### ROS 2 (colcon) Integration
Place the `bno055lib` directory directly inside your ROS 2 workspace's `src` folder alongside other packages. `colcon` will automatically build it as a pure CMake package.

To use it from another ROS 2 package:
1. Add dependency to `package.xml`:
   ```xml
   <depend>bno055lib</depend>
   ```
2. Find and link in `CMakeLists.txt`:
   ```cmake
   find_package(bno055lib REQUIRED)
   ament_target_dependencies(your_node_target bno055lib)
   ```

---

## Usage

```cpp
#include <bno055lib/bno055.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // 1. Initialize sensor (default address: 0x28, device: /dev/i2c-1)
    bno055lib::BNO055 imu(0x28, "/dev/i2c-1");

    // 2. Setup custom logger (Optional)
    imu.setLogger([](bno055lib::LogLevel level, std::string_view message) {
        std::cout << "[IMU] " << message << std::endl;
    });

    // 3. Start sensor in NDOF fusion mode
    if (!imu.begin(bno055lib::OpMode::NDOF)) {
        std::cerr << "Initialization failed!" << std::endl;
        return 1;
    }

    // 4. Load calibration file if exists (Optional)
    imu.loadCalibrationFile("bno055_calib.bin");

    // 5. Main loop to read data (SI units)
    while (true) {
        try {
            auto accel = imu.getLinearAcceleration(); // m/s^2
            auto gyro  = imu.getGyroscope();           // rad/s
            auto euler = imu.getEulerAngles();         // rad (x=Roll, y=Pitch, z=Yaw)
            auto quat  = imu.getQuaternion();          // Quaternion (w, x, y, z)

            std::cout << "Euler (deg): R=" << euler.x * 180 / M_PI 
                      << " P=" << euler.y * 180 / M_PI 
                      << " Y=" << euler.z * 180 / M_PI << std::endl;
        } catch (const bno055lib::IMUError& e) {
            std::cerr << "Sensor read error: " << e.what() << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 6. Save calibration on exit (Optional)
    // imu.saveCalibrationFile("bno055_calib.bin");

    return 0;
}
```

---

## API Reference

### Namespaces & Types

```cpp
namespace bno055lib {
    struct Vector3 { double x, y, z; };
    struct Quaternion { double w, x, y, z; };
    
    struct Offsets {
        int16_t accel_offset_x, accel_offset_y, accel_offset_z;
        int16_t mag_offset_x, mag_offset_y, mag_offset_z;
        int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;
        int16_t accel_radius, mag_radius;
    };

    struct CalibrationStatus {
        uint8_t sys;   // 0 to 3
        uint8_t gyro;
        uint8_t accel;
        uint8_t mag;
        bool isFullyCalibrated() const;
    };

    enum class OpMode : uint8_t {
        Config = 0X00, AccOnly = 0X01, MagOnly = 0X02, GyroOnly = 0X03,
        AccMag = 0X04, AccGyro = 0X05, MagGyro = 0X06, AMG = 0X07,
        IMUPlus = 0X08, Compass = 0X09, M4G = 0X0A, NDOF_FMC_Off = 0X0B,
        NDOF = 0X0C
    };

    enum class LogLevel { Debug, Info, Warning, Error };
    using LoggerCallback = std::function<void(LogLevel level, std::string_view message)>;
}
```

### Class `BNO055`

#### Lifecycle
* **`explicit BNO055(uint8_t i2c_address = 0x28, std::string_view i2c_device = "/dev/i2c-1")`**
* **`bool begin(OpMode mode = OpMode::NDOF)`**

#### Configuration
* **`void setMode(OpMode mode)`**
* **`OpMode getMode()`**
* **`void setAxisRemap(AxisMapConfig config)`**
* **`void setAxisSign(AxisMapSign sign)`**
* **`void setExtCrystalUse(bool use_xtal)`**

#### Sensor Data (SI Units)
* **`Vector3 getAccelerometer()`** (returns $m/s^2$)
* **`Vector3 getMagnetometer()`** (returns $\mu T$)
* **`Vector3 getGyroscope()`** (returns $rad/s$)
* **`Vector3 getEulerAngles()`** (returns `x = Roll`, `y = Pitch`, `z = Yaw` in $rad$)
* **`Vector3 getLinearAcceleration()`** (returns $m/s^2$)
* **`Vector3 getGravity()`** (returns $m/s^2$)
* **`Quaternion getQuaternion()`**
* **`int8_t getTemperature()`** (returns Celsius)

#### Calibration & Offsets
* **`CalibrationStatus getCalibrationStatus()`**
* **`bool getSensorOffsets(Offsets& offsets)`**
* **`bool getSensorOffsets(std::array<uint8_t, 22>& calib_data)`**
* **`void setSensorOffsets(const Offsets& offsets)`**
* **`void setSensorOffsets(const std::array<uint8_t, 22>& calib_data)`**
* **`bool saveCalibrationFile(std::string_view filepath)`**
* **`bool loadCalibrationFile(std::string_view filepath)`**

#### Power Management
* **`void enterSuspendMode()`**
* **`void enterNormalMode()`**

#### Logging
* **`void setLogger(LoggerCallback callback)`**

---

## License

MIT License.
