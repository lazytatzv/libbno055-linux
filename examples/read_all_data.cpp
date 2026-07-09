#include "libbno055-linux/bno055.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <iomanip>

int main(int argc, char* argv[]) {
    std::string device = "/dev/i2c-1";
    bno055lib::OpMode mode = bno055lib::OpMode::NDOF;
    std::string mode_str = "ndof";

    if (argc > 1) {
        device = argv[1];
    }
    if (argc > 2) {
        mode_str = argv[2];
        if (mode_str == "imu") {
            mode = bno055lib::OpMode::IMUPlus;
        } else if (mode_str == "amg") {
            mode = bno055lib::OpMode::AMG;
        } else if (mode_str == "gyro") {
            mode = bno055lib::OpMode::GyroOnly;
        } else if (mode_str == "ndof") {
            mode = bno055lib::OpMode::NDOF;
        } else {
            std::cerr << "Unknown mode: " << mode_str << ". Defaulting to ndof." << std::endl;
            mode_str = "ndof";
        }
    }

    std::cout << "Initializing BNO055 IMU on " << device << " (Mode: " << mode_str << ")..." << std::endl;
    bno055lib::BNO055 imu(0x28, device);

    // Use default logger (standard error output)
    if (!imu.begin(mode)) {
        std::cerr << "Failed to initialize BNO055!" << std::endl;
        return 1;
    }

    std::cout << "BNO055 initialized. Displaying all sensor readings (10Hz)..." << std::endl;
    std::cout << std::fixed << std::setprecision(4);

    while (true) {
        try {
            // Read all 8 physical data types provided by BNO055
            auto accel  = imu.getAccelerometer();
            auto mag    = imu.getMagnetometer();
            auto gyro   = imu.getGyroscope();
            auto euler  = imu.getEulerAngles();
            auto linear = imu.getLinearAcceleration();
            auto gravity = imu.getGravity();
            auto quat   = imu.getQuaternion();
            auto temp   = imu.getTemperature();
            auto calib  = imu.getCalibrationStatus();

            std::cout << "\n------------------ BNO055 Data Frame ------------------" << std::endl;
            std::cout << "Calibration: SYS=" << (int)calib.sys 
                      << " GYRO=" << (int)calib.gyro 
                      << " ACCEL=" << (int)calib.accel 
                      << " MAG=" << (int)calib.mag << std::endl;
            
            std::cout << "Accelerometer (m/s^2):      X=" << accel.x << " Y=" << accel.y << " Z=" << accel.z << std::endl;
            std::cout << "Magnetometer (uT):          X=" << mag.x << " Y=" << mag.y << " Z=" << mag.z << std::endl;
            std::cout << "Gyroscope (rad/s):          X=" << gyro.x << " Y=" << gyro.y << " Z=" << gyro.z << std::endl;
            std::cout << "Euler Angles (rad):         Roll=" << euler.x << " Pitch=" << euler.y << " Yaw=" << euler.z << std::endl;
            std::cout << "Linear Acceleration (m/s^2): X=" << linear.x << " Y=" << linear.y << " Z=" << linear.z << std::endl;
            std::cout << "Gravity Vector (m/s^2):      X=" << gravity.x << " Y=" << gravity.y << " Z=" << gravity.z << std::endl;
            std::cout << "Quaternion:                 W=" << quat.w << " X=" << quat.x << " Y=" << quat.y << " Z=" << quat.z << std::endl;
            std::cout << "Temperature (C):            " << (int)temp << std::endl;

        } catch (const bno055lib::IMUError& e) {
            std::cerr << "Communication error: " << e.what() << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 10Hz
    }

    return 0;
}
