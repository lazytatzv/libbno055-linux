#include "bno055lib/bno055.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>

namespace bno055lib {

namespace {

// BNO055 Constants
constexpr uint8_t BNO055_ID = 0xA0;

// Register Map
enum Register : uint8_t {
    PAGE_ID = 0x07,
    CHIP_ID = 0x00,
    ACCEL_REV_ID = 0x01,
    MAG_REV_ID = 0x02,
    GYRO_REV_ID = 0x03,
    SW_REV_ID_LSB = 0x04,
    SW_REV_ID_MSB = 0x05,
    BL_REV_ID = 0x06,
    
    // Data Registers
    ACCEL_DATA_X_LSB = 0x08,
    MAG_DATA_X_LSB = 0x0E,
    GYRO_DATA_X_LSB = 0x14,
    EULER_H_LSB = 0x1A,
    QUATERNION_DATA_W_LSB = 0x20,
    LINEAR_ACCEL_DATA_X_LSB = 0x28,
    GRAVITY_DATA_X_LSB = 0x2E,
    TEMP = 0x34,
    
    // Status
    CALIB_STAT = 0x35,
    SELFTEST_RESULT = 0x36,
    INTR_STAT = 0x37,
    SYS_CLK_STAT = 0x38,
    SYS_STAT = 0x39,
    SYS_ERR = 0x3A,
    
    // Configuration
    UNIT_SEL = 0x3B,
    OPR_MODE = 0x3D,
    PWR_MODE = 0x3E,
    SYS_TRIGGER = 0x3F,
    TEMP_SOURCE = 0x40,
    AXIS_MAP_CONFIG = 0x41,
    AXIS_MAP_SIGN = 0x42,
    
    // Offsets
    ACCEL_OFFSET_X_LSB = 0x55,
    ACCEL_OFFSET_X_MSB = 0x56,
    ACCEL_OFFSET_Y_LSB = 0x57,
    ACCEL_OFFSET_Y_MSB = 0x58,
    ACCEL_OFFSET_Z_LSB = 0x59,
    ACCEL_OFFSET_Z_MSB = 0x5A,
    
    MAG_OFFSET_X_LSB = 0x5B,
    MAG_OFFSET_X_MSB = 0x5C,
    MAG_OFFSET_Y_LSB = 0x5D,
    MAG_OFFSET_Y_MSB = 0x5E,
    MAG_OFFSET_Z_LSB = 0x5F,
    MAG_OFFSET_Z_MSB = 0x60,
    
    GYRO_OFFSET_X_LSB = 0x61,
    GYRO_OFFSET_X_MSB = 0x62,
    GYRO_OFFSET_Y_LSB = 0x63,
    GYRO_OFFSET_Y_MSB = 0x64,
    GYRO_OFFSET_Z_LSB = 0x65,
    GYRO_OFFSET_Z_MSB = 0x66,
    
    ACCEL_RADIUS_LSB = 0x67,
    ACCEL_RADIUS_MSB = 0x68,
    MAG_RADIUS_LSB = 0x69,
    MAG_RADIUS_MSB = 0x6A
};

// Power Modes
constexpr uint8_t POWER_MODE_NORMAL = 0x00;
constexpr uint8_t POWER_MODE_LOWPOWER = 0x01;
constexpr uint8_t POWER_MODE_SUSPEND = 0x02;

} // namespace

class BNO055::Impl {
public:
    uint8_t address_;
    std::string i2c_device_;
    int i2c_fd{-1};
    OpMode mode_{OpMode::Config};
    std::mutex mutex_;
    LoggerCallback logger_;

    Impl(uint8_t address, std::string_view i2c_device)
        : address_(address), i2c_device_(std::string(i2c_device)) {}

    ~Impl() {
        close_i2c();
    }

    void log(LogLevel level, std::string_view msg) {
        if (logger_) {
            logger_(level, msg);
        } else {
            std::string label;
            switch (level) {
                case LogLevel::Debug: label = "[DEBUG]"; break;
                case LogLevel::Info: label = "[INFO]"; break;
                case LogLevel::Warning: label = "[WARNING]"; break;
                case LogLevel::Error: label = "[ERROR]"; break;
            }
            std::cerr << "[bno055lib::BNO055] " << label << " " << msg << std::endl;
        }
    }

    bool open_i2c() {
        if (i2c_fd >= 0) {
            return true;
        }
        i2c_fd = open(i2c_device_.c_str(), O_RDWR);
        if (i2c_fd < 0) {
            log(LogLevel::Error, "Failed to open I2C device: " + i2c_device_);
            return false;
        }
        if (ioctl(i2c_fd, I2C_SLAVE, address_) < 0) {
            log(LogLevel::Error, "Failed to set I2C slave address");
            close(i2c_fd);
            i2c_fd = -1;
            return false;
        }
        return true;
    }

    void close_i2c() {
        if (i2c_fd >= 0) {
            close(i2c_fd);
            i2c_fd = -1;
        }
    }

    bool reconnect() {
        log(LogLevel::Warning, "Attempting to reconnect I2C and reinitialize BNO055...");
        close_i2c();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!open_i2c()) {
            return false;
        }

        // Wait boot
        int timeout = 500;
        uint8_t id = 0;
        while (timeout > 0) {
            if (read8_raw(CHIP_ID, id)) {
                if (id == BNO055_ID) break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            timeout -= 10;
        }
        if (id != BNO055_ID) {
            log(LogLevel::Error, "Reconnection failed: BNO055 not detected");
            return false;
        }

        // Reset
        if (!write8_raw(SYS_TRIGGER, 0x20)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        while (true) {
            uint8_t chip_id = 0;
            if (read8_raw(CHIP_ID, chip_id) && chip_id == BNO055_ID) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // Reapply config
        if (!write8_raw(PWR_MODE, POWER_MODE_NORMAL)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (!write8_raw(PAGE_ID, 0)) return false;
        if (!write8_raw(SYS_TRIGGER, 0x0)) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Reapply operating mode
        if (!write8_raw(OPR_MODE, static_cast<uint8_t>(mode_))) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(30));

        log(LogLevel::Info, "BNO055 reconnected successfully");
        return true;
    }

    // Low-level raw methods
    bool write8_raw(uint8_t reg, uint8_t value) {
        if (i2c_fd < 0) return false;
        uint8_t buffer[2] = {reg, value};
        return ::write(i2c_fd, buffer, 2) == 2;
    }

    bool read8_raw(uint8_t reg, uint8_t& value) {
        if (i2c_fd < 0) return false;
        uint8_t reg_buf[1] = {reg};
        if (::write(i2c_fd, reg_buf, 1) != 1) return false;
        return ::read(i2c_fd, &value, 1) == 1;
    }

    // Thread-safe methods with automatic reconnect and retries
    bool write8(uint8_t reg, uint8_t value, int retries = 3) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < retries; ++i) {
            if (i2c_fd < 0 && !open_i2c()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            uint8_t buffer[2] = {reg, value};
            if (::write(i2c_fd, buffer, 2) == 2) {
                return true;
            }
            log(LogLevel::Warning, "I2C write failed, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (reconnect()) {
            uint8_t buffer[2] = {reg, value};
            if (::write(i2c_fd, buffer, 2) == 2) {
                return true;
            }
        }
        log(LogLevel::Error, "I2C write failed permanently");
        return false;
    }

    uint8_t read8(uint8_t reg, int retries = 3) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < retries; ++i) {
            if (i2c_fd < 0 && !open_i2c()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            uint8_t reg_buf[1] = {reg};
            if (::write(i2c_fd, reg_buf, 1) == 1) {
                uint8_t value = 0;
                if (::read(i2c_fd, &value, 1) == 1) {
                    return value;
                }
            }
            log(LogLevel::Warning, "I2C read failed, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (reconnect()) {
            uint8_t reg_buf[1] = {reg};
            if (::write(i2c_fd, reg_buf, 1) == 1) {
                uint8_t value = 0;
                if (::read(i2c_fd, &value, 1) == 1) {
                    return value;
                }
            }
        }
        log(LogLevel::Error, "I2C read failed permanently");
        throw IMUError("Failed to read from BNO055 register");
    }

    bool readLen(uint8_t reg, uint8_t* buffer, uint8_t len, int retries = 3) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (int i = 0; i < retries; ++i) {
            if (i2c_fd < 0 && !open_i2c()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            uint8_t reg_buf[1] = {reg};
            if (::write(i2c_fd, reg_buf, 1) == 1) {
                if (::read(i2c_fd, buffer, len) == len) {
                    return true;
                }
            }
            log(LogLevel::Warning, "I2C readLen failed, retrying...");
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        if (reconnect()) {
            uint8_t reg_buf[1] = {reg};
            if (::write(i2c_fd, reg_buf, 1) == 1) {
                if (::read(i2c_fd, buffer, len) == len) {
                    return true;
                }
            }
        }
        log(LogLevel::Error, "I2C readLen failed permanently");
        return false;
    }
};

BNO055::BNO055(uint8_t i2c_address, std::string_view i2c_device)
    : impl_(std::make_unique<Impl>(i2c_address, i2c_device)) {}

BNO055::~BNO055() = default;

BNO055::BNO055(BNO055&&) noexcept = default;
BNO055& BNO055::operator=(BNO055&&) noexcept = default;

bool BNO055::begin(OpMode mode) {
    if (!impl_->open_i2c()) {
        return false;
    }

    // Detect device
    int timeout = 850;
    uint8_t id = 0;
    while (timeout > 0) {
        try {
            id = impl_->read8(CHIP_ID, 1);
            if (id == BNO055_ID) break;
        } catch (const IMUError&) {
            // Ignore errors during boot wait
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        timeout -= 10;
    }

    if (id != BNO055_ID) {
        impl_->log(LogLevel::Error, "BNO055 not detected. Found ID: 0x" + std::string(1, id));
        return false;
    }

    // Config Mode
    setMode(OpMode::Config);

    // Reset
    impl_->write8(SYS_TRIGGER, 0x20);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    while (true) {
        try {
            if (impl_->read8(CHIP_ID, 1) == BNO055_ID) break;
        } catch (...) {}
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Normal Power Mode
    impl_->write8(PWR_MODE, POWER_MODE_NORMAL);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    impl_->write8(PAGE_ID, 0);
    impl_->write8(SYS_TRIGGER, 0x0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    // Set Operating Mode
    setMode(mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    return true;
}

void BNO055::setMode(OpMode mode) {
    impl_->mode_ = mode;
    impl_->write8(OPR_MODE, static_cast<uint8_t>(mode));
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}

OpMode BNO055::getMode() {
    return static_cast<OpMode>(impl_->read8(OPR_MODE));
}

void BNO055::setAxisRemap(AxisMapConfig config) {
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    impl_->write8(AXIS_MAP_CONFIG, static_cast<uint8_t>(config));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    setMode(prev);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void BNO055::setAxisSign(AxisMapSign sign) {
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    impl_->write8(AXIS_MAP_SIGN, static_cast<uint8_t>(sign));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    setMode(prev);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void BNO055::setExtCrystalUse(bool use_xtal) {
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    impl_->write8(PAGE_ID, 0);
    if (use_xtal) {
        impl_->write8(SYS_TRIGGER, 0x80);
    } else {
        impl_->write8(SYS_TRIGGER, 0x00);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    setMode(prev);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

Vector3 BNO055::getAccelerometer() {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(ACCEL_DATA_X_LSB, buffer, 6)) {
        throw IMUError("Failed to read accelerometer data");
    }
    int16_t x = static_cast<int16_t>(buffer[0] | (buffer[1] << 8));
    int16_t y = static_cast<int16_t>(buffer[2] | (buffer[3] << 8));
    int16_t z = static_cast<int16_t>(buffer[4] | (buffer[5] << 8));

    // 1 m/s^2 = 100 LSB
    return Vector3{x / 100.0, y / 100.0, z / 100.0};
}

Vector3 BNO055::getMagnetometer() {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(MAG_DATA_X_LSB, buffer, 6)) {
        throw IMUError("Failed to read magnetometer data");
    }
    int16_t x = static_cast<int16_t>(buffer[0] | (buffer[1] << 8));
    int16_t y = static_cast<int16_t>(buffer[2] | (buffer[3] << 8));
    int16_t z = static_cast<int16_t>(buffer[4] | (buffer[5] << 8));

    // 1 uT = 16 LSB
    return Vector3{x / 16.0, y / 16.0, z / 16.0};
}

Vector3 BNO055::getGyroscope() {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(GYRO_DATA_X_LSB, buffer, 6)) {
        throw IMUError("Failed to read gyroscope data");
    }
    int16_t x = static_cast<int16_t>(buffer[0] | (buffer[1] << 8));
    int16_t y = static_cast<int16_t>(buffer[2] | (buffer[3] << 8));
    int16_t z = static_cast<int16_t>(buffer[4] | (buffer[5] << 8));

    // 1 dps = 16 LSB. Convert to rad/s (dps * M_PI / 180.0)
    constexpr double scale = (1.0 / 16.0) * (M_PI / 180.0);
    return Vector3{x * scale, y * scale, z * scale};
}

Vector3 BNO055::getEulerAngles() {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(EULER_H_LSB, buffer, 6)) {
        throw IMUError("Failed to read euler angles");
    }
    int16_t h = static_cast<int16_t>(buffer[0] | (buffer[1] << 8));
    int16_t r = static_cast<int16_t>(buffer[2] | (buffer[3] << 8));
    int16_t p = static_cast<int16_t>(buffer[4] | (buffer[5] << 8));

    // 1 degree = 16 LSB. Convert to rad (deg * M_PI / 180.0)
    constexpr double scale = (1.0 / 16.0) * (M_PI / 180.0);
    // h: yaw, r: roll, p: pitch
    return Vector3{r * scale, p * scale, h * scale};
}

Vector3 BNO055::getLinearAcceleration() {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(LINEAR_ACCEL_DATA_X_LSB, buffer, 6)) {
        throw IMUError("Failed to read linear acceleration data");
    }
    int16_t x = static_cast<int16_t>(buffer[0] | (buffer[1] << 8));
    int16_t y = static_cast<int16_t>(buffer[2] | (buffer[3] << 8));
    int16_t z = static_cast<int16_t>(buffer[4] | (buffer[5] << 8));

    // 1 m/s^2 = 100 LSB
    return Vector3{x / 100.0, y / 100.0, z / 100.0};
}

Vector3 BNO055::getGravity() {
    uint8_t buffer[6]{0};
    if (!impl_->readLen(GRAVITY_DATA_X_LSB, buffer, 6)) {
        throw IMUError("Failed to read gravity data");
    }
    int16_t x = static_cast<int16_t>(buffer[0] | (buffer[1] << 8));
    int16_t y = static_cast<int16_t>(buffer[2] | (buffer[3] << 8));
    int16_t z = static_cast<int16_t>(buffer[4] | (buffer[5] << 8));

    // 1 m/s^2 = 100 LSB
    return Vector3{x / 100.0, y / 100.0, z / 100.0};
}

Quaternion BNO055::getQuaternion() {
    uint8_t buffer[8]{0};
    if (!impl_->readLen(QUATERNION_DATA_W_LSB, buffer, 8)) {
        throw IMUError("Failed to read quaternion data");
    }
    int16_t w = static_cast<int16_t>(buffer[0] | (buffer[1] << 8));
    int16_t x = static_cast<int16_t>(buffer[2] | (buffer[3] << 8));
    int16_t y = static_cast<int16_t>(buffer[4] | (buffer[5] << 8));
    int16_t z = static_cast<int16_t>(buffer[6] | (buffer[7] << 8));

    // 1 = 16384 LSB (scale factor 2^14)
    constexpr double scale = 1.0 / 16384.0;
    return Quaternion{w * scale, x * scale, y * scale, z * scale};
}

int8_t BNO055::getTemperature() {
    return static_cast<int8_t>(impl_->read8(TEMP));
}

CalibrationStatus BNO055::getCalibrationStatus() {
    uint8_t stat = impl_->read8(CALIB_STAT);
    CalibrationStatus status;
    status.sys = (stat >> 6) & 0x03;
    status.gyro = (stat >> 4) & 0x03;
    status.accel = (stat >> 2) & 0x03;
    status.mag = stat & 0x03;
    return status;
}

bool BNO055::getSensorOffsets(Offsets& offsets) {
    std::array<uint8_t, 22> data;
    if (!getSensorOffsets(data)) return false;

    offsets.accel_offset_x = static_cast<int16_t>(data[0] | (data[1] << 8));
    offsets.accel_offset_y = static_cast<int16_t>(data[2] | (data[3] << 8));
    offsets.accel_offset_z = static_cast<int16_t>(data[4] | (data[5] << 8));

    offsets.mag_offset_x = static_cast<int16_t>(data[6] | (data[7] << 8));
    offsets.mag_offset_y = static_cast<int16_t>(data[8] | (data[9] << 8));
    offsets.mag_offset_z = static_cast<int16_t>(data[10] | (data[11] << 8));

    offsets.gyro_offset_x = static_cast<int16_t>(data[12] | (data[13] << 8));
    offsets.gyro_offset_y = static_cast<int16_t>(data[14] | (data[15] << 8));
    offsets.gyro_offset_z = static_cast<int16_t>(data[16] | (data[17] << 8));

    offsets.accel_radius = static_cast<int16_t>(data[18] | (data[19] << 8));
    offsets.mag_radius = static_cast<int16_t>(data[20] | (data[21] << 8));
    return true;
}

bool BNO055::getSensorOffsets(std::array<uint8_t, 22>& calib_data) {
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    bool ok = impl_->readLen(ACCEL_OFFSET_X_LSB, calib_data.data(), 22);
    setMode(prev);
    return ok;
}

void BNO055::setSensorOffsets(const Offsets& offsets) {
    std::array<uint8_t, 22> data;
    data[0] = offsets.accel_offset_x & 0xFF;
    data[1] = (offsets.accel_offset_x >> 8) & 0xFF;
    data[2] = offsets.accel_offset_y & 0xFF;
    data[3] = (offsets.accel_offset_y >> 8) & 0xFF;
    data[4] = offsets.accel_offset_z & 0xFF;
    data[5] = (offsets.accel_offset_z >> 8) & 0xFF;

    data[6] = offsets.mag_offset_x & 0xFF;
    data[7] = (offsets.mag_offset_x >> 8) & 0xFF;
    data[8] = offsets.mag_offset_y & 0xFF;
    data[9] = (offsets.mag_offset_y >> 8) & 0xFF;
    data[10] = offsets.mag_offset_z & 0xFF;
    data[11] = (offsets.mag_offset_z >> 8) & 0xFF;

    data[12] = offsets.gyro_offset_x & 0xFF;
    data[13] = (offsets.gyro_offset_x >> 8) & 0xFF;
    data[14] = offsets.gyro_offset_y & 0xFF;
    data[15] = (offsets.gyro_offset_y >> 8) & 0xFF;
    data[16] = offsets.gyro_offset_z & 0xFF;
    data[17] = (offsets.gyro_offset_z >> 8) & 0xFF;

    data[18] = offsets.accel_radius & 0xFF;
    data[19] = (offsets.accel_radius >> 8) & 0xFF;
    data[20] = offsets.mag_radius & 0xFF;
    data[21] = (offsets.mag_radius >> 8) & 0xFF;

    setSensorOffsets(data);
}

void BNO055::setSensorOffsets(const std::array<uint8_t, 22>& calib_data) {
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    
    for (size_t i = 0; i < 22; ++i) {
        impl_->write8(static_cast<uint8_t>(ACCEL_OFFSET_X_LSB + i), calib_data[i]);
    }

    setMode(prev);
}

bool BNO055::saveCalibrationFile(std::string_view filepath) {
    std::array<uint8_t, 22> data;
    if (!getSensorOffsets(data)) {
        impl_->log(LogLevel::Warning, "Failed to retrieve offsets. Sensor might not be configured.");
        return false;
    }

    std::ofstream ofs(std::string(filepath), std::ios::binary);
    if (!ofs) {
        impl_->log(LogLevel::Error, "Failed to open calibration file for writing: " + std::string(filepath));
        return false;
    }

    ofs.write(reinterpret_cast<const char*>(data.data()), 22);
    impl_->log(LogLevel::Info, "Successfully saved calibration to: " + std::string(filepath));
    return true;
}

bool BNO055::loadCalibrationFile(std::string_view filepath) {
    std::ifstream ifs(std::string(filepath), std::ios::binary);
    if (!ifs) {
        impl_->log(LogLevel::Error, "Failed to open calibration file for reading: " + std::string(filepath));
        return false;
    }

    std::array<uint8_t, 22> data;
    ifs.read(reinterpret_cast<char*>(data.data()), 22);
    if (ifs.gcount() != 22) {
        impl_->log(LogLevel::Error, "Invalid calibration file size (expected 22 bytes): " + std::string(filepath));
        return false;
    }

    setSensorOffsets(data);
    impl_->log(LogLevel::Info, "Successfully loaded calibration from: " + std::string(filepath));
    return true;
}

void BNO055::enterSuspendMode() {
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    impl_->write8(PWR_MODE, POWER_MODE_SUSPEND);
    setMode(prev);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void BNO055::enterNormalMode() {
    OpMode prev = impl_->mode_;
    setMode(OpMode::Config);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    impl_->write8(PWR_MODE, POWER_MODE_NORMAL);
    setMode(prev);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
}

void BNO055::setLogger(LoggerCallback callback) {
    impl_->logger_ = std::move(callback);
}

} // namespace bno055lib
