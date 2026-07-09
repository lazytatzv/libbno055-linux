# bno055lib

A robust, thread-safe, and dependency-free C++17 library for the BNO055 orientation sensor over I2C on Linux.

Linux上のI2C経由でBNO055センサを制御する、堅牢かつスレッドセーフなC++17ライブラリです。

---

## Features (特徴)

- **No Dependencies**: Pure C++17 & CMake (No ROS dependency).
- **Thread-safe**: All I2C operations are protected by `std::mutex`.
- **Robust**: Automatic retries and reconnection logic on I2C bus errors.
- **Pimpl Idiom**: Fast compilation, clean API, and no Linux system headers exposed.
- **Auto conversion**: Sensor data is automatically converted to standard SI units ($m/s^2$, $rad/s$, $rad$, $\mu T$).

---

## Build & Install (ビルドとインストール)

```bash
mkdir build && cd build
cmake ..
make
sudo make install
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
        uint8_t sys;   // 0 (uncalibrated) to 3 (fully calibrated)
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
  Constructs the IMU instance. Alternate I2C address is `0x29`.
* **`bool begin(OpMode mode = OpMode::NDOF)`**
  Initializes the I2C bus, resets the sensor, and sets the initial operating mode. Returns `true` if successful.

#### Configuration
* **`void setMode(OpMode mode)`**
  Changes the sensor operation mode.
* **`OpMode getMode()`**
  Gets the current operation mode.
* **`void setAxisRemap(AxisMapConfig config)`**
  Configures the mapping of physical axes to coordinate axes.
* **`void setAxisSign(AxisMapSign sign)`**
  Configures the signs of the mapped axes.
* **`void setExtCrystalUse(bool use_xtal)`**
  Enables or disables the external 32.768kHz crystal.

#### Sensor Data (SI Units)
* **`Vector3 getAccelerometer()`**
  Gets raw accelerometer data in **$m/s^2$**.
* **`Vector3 getMagnetometer()`**
  Gets magnetometer data in **$\mu T$** (Microtesla).
* **`Vector3 getGyroscope()`**
  Gets gyroscope data in **$rad/s$**.
* **`Vector3 getEulerAngles()`**
  Gets euler angles. Returns `Vector3` where `x = Roll`, `y = Pitch`, `z = Yaw` in **$rad$**.
* **`Vector3 getLinearAcceleration()`**
  Gets acceleration excluding gravity in **$m/s^2$**.
* **`Vector3 getGravity()`**
  Gets gravity vector in **$m/s^2$**.
* **`Quaternion getQuaternion()`**
  Gets normalized orientation quaternion (w, x, y, z).
* **`int8_t getTemperature()`**
  Gets ambient temperature in **Celsius**.

#### Calibration & Offsets
* **`CalibrationStatus getCalibrationStatus()`**
  Gets current calibration levels (0 to 3) for all sensor parts.
* **`bool getSensorOffsets(Offsets& offsets)`**
  Populates the `offsets` structure with calibration profile. Returns `false` if read failed.
* **`bool getSensorOffsets(std::array<uint8_t, 22>& calib_data)`**
  Populates a raw 22-byte array with calibration profile.
* **`void setSensorOffsets(const Offsets& offsets)`**
  Writes calibration profile to the sensor.
* **`void setSensorOffsets(const std::array<uint8_t, 22>& calib_data)`**
  Writes raw 22-byte calibration profile.
* **`bool saveCalibrationFile(std::string_view filepath)`**
  Saves the current calibration profile to a 22-byte binary file.
* **`bool loadCalibrationFile(std::string_view filepath)`**
  Loads calibration profile from a 22-byte binary file and writes to the sensor.

#### Power Management
* **`void enterSuspendMode()`**
  Puts the sensor to low-power suspend mode.
* **`void enterNormalMode()`**
  Wakes up the sensor to normal power mode.

#### Logging
* **`void setLogger(LoggerCallback callback)`**
  Registers a callback to intercept internal debug and error logs.

---

## License

MIT License.
