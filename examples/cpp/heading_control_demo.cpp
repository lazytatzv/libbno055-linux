#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string_view>
#include <thread>

#include "libbno055-linux/bno055.hpp"

// Likelihood macros for GCC/Clang
#if defined(__GNACT__) || defined(__GNUC__) || defined(__clang__)
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#define LIKELY(x) (x)
#define UNLIKELY(x) (x)
#endif

// Compile-time constants
constexpr uint8_t DEFAULT_ADDR = 0x28;
constexpr auto LOOP_PERIOD = std::chrono::microseconds(10000);  // 100Hz ultra-fast loop (10ms)
constexpr double RAD_TO_DEG = 180.0 / M_PI;

/**
 * @brief High-performance, zero-allocation PID Controller.
 * Marked inline/noexcept for maximum compiler optimization.
 */
class HighPerfPidController {
public:
    constexpr HighPerfPidController(double kp = 0.05, double ki = 0.001, double kd = 0.01,
                                   double min_out = -0.5, double max_out = 0.5,
                                   double max_i = 0.1) noexcept
        : kp_(kp), ki_(ki), kd_(kd),
          min_out_(min_out), max_out_(max_out),
          max_i_(max_i), i_term_(0.0), prev_error_(0.0), initialized_(false) {}

    inline void setGains(double kp, double ki, double kd) noexcept {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    inline void reset() noexcept {
        i_term_ = 0.0;
        prev_error_ = 0.0;
        initialized_ = false;
    }

    [[nodiscard]] inline double compute(double error, double dt, double gyro_z = 0.0, bool use_gyro_d = true) noexcept {
        if (UNLIKELY(dt <= 0.0)) return 0.0;

        // Proportional
        const double p_term = kp_ * error;

        // Anti-windup Integral Clamping
        i_term_ = std::clamp(i_term_ + ki_ * error * dt, -max_i_, max_i_);

        // Derivative (direct gyro rate avoids noise & derivative kick)
        double d_term = 0.0;
        if (LIKELY(use_gyro_d)) {
            d_term = -kd_ * gyro_z;
        } else {
            if (LIKELY(initialized_)) {
                d_term = kd_ * ((error - prev_error_) / dt);
            } else {
                initialized_ = true;
            }
            prev_error_ = error;
        }

        return std::clamp(p_term + i_term_ + d_term, min_out_, max_out_);
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double min_out_;
    double max_out_;
    double max_i_;
    double i_term_;
    double prev_error_;
    bool initialized_;
};

/**
 * @brief Fast Yaw extraction directly from Quaternion without computing unused Roll/Pitch.
 */
[[nodiscard]] inline double fastExtractYawDeg(const bno055lib::Quaternion& q) noexcept {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

/**
 * @brief Fast angle wrap into [-180.0, 180.0] range using std::remainder for speed.
 */
[[nodiscard]] inline double normalizeAngleDeg(double angle_deg) noexcept {
    return std::remainder(angle_deg, 360.0);
}

/**
 * @brief Zero-allocation ASCII bar generator using static stack buffer.
 */
inline void renderBarToBuffer(char* dest, size_t dest_size, double val, int width = 20) noexcept {
    val = std::clamp(val, -1.0, 1.0);
    int center = width / 2;
    int pos = center + static_cast<int>(val * center);
    pos = std::clamp(pos, 0, width - 1);

    if (width >= static_cast<int>(dest_size)) width = static_cast<int>(dest_size) - 1;
    std::memset(dest, ' ', width);
    dest[center] = '|';

    if (pos >= center) {
        std::memset(dest + center + 1, '=', pos - center);
    } else {
        std::memset(dest + pos, '=', center - pos);
    }
    dest[width] = '\0';
}

int main(int argc, char* argv[]) {
    // Fast I/O
    std::ios_base::sync_with_stdio(false);
    std::cin.tie(nullptr);

    std::string device = "/dev/i2c-1";
    if (argc > 1) device = argv[1];

    std::cout << "Initializing BNO055 [Ultra Performance 100Hz Mode] on " << device << "...\n";

    bno055lib::BNO055 imu(DEFAULT_ADDR, device);
    if (!imu.begin(bno055lib::OpMode::NDOF)) {
        std::cerr << "Failed to initialize BNO055!\n";
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Lock initial heading
    double target_heading_deg = 0.0;
    try {
        target_heading_deg = fastExtractYawDeg(imu.getQuaternion());
    } catch (...) {
        target_heading_deg = 0.0;
    }

    HighPerfPidController pid(/*Kp=*/0.03, /*Ki=*/0.002, /*Kd=*/0.005, /*min=*/-0.5, /*max=*/0.5, /*max_i=*/0.1);
    constexpr double base_velocity = 0.5;

    // Buffer for ultra-fast screen redrawing without flickering or std::string allocation
    char output_buffer[2048];
    char bar_corr[32], bar_left[32], bar_right[32];

    auto next_wake = std::chrono::steady_clock::now();

    while (true) {
        next_wake += LOOP_PERIOD;

        try {
            // Read hardware IMU data
            const auto quat = imu.getQuaternion();
            const auto gyro = imu.getGyroscope();
            const auto calib = imu.getCalibrationStatus();

            const double current_heading_deg = fastExtractYawDeg(quat);
            const double gyro_z_deg = gyro.z * RAD_TO_DEG;

            // Shortest angle error & PID computation
            const double heading_error_deg = normalizeAngleDeg(target_heading_deg - current_heading_deg);
            const double turn_correction = pid.compute(heading_error_deg, 0.01, gyro_z_deg, true);

            // Calculate wheel outputs
            double left_motor = std::clamp(base_velocity - turn_correction, 0.0, 1.0);
            double right_motor = std::clamp(base_velocity + turn_correction, 0.0, 1.0);

            // Zero-allocation visual bar generation
            renderBarToBuffer(bar_corr, sizeof(bar_corr), turn_correction * 2.0);
            renderBarToBuffer(bar_left, sizeof(bar_left), left_motor);
            renderBarToBuffer(bar_right, sizeof(bar_right), right_motor);

            // Fast snprintf to fixed buffer and single syscall write
            int len = std::snprintf(
                output_buffer, sizeof(output_buffer),
                "\033[H"  // Reset cursor to top-left (no full screen clear screen flash)
                "================ ULTRA-PERFORMANCE BNO055 PID DEMO (100Hz) ================\n"
                "Calib: SYS=%d G=%d A=%d M=%d | Loop: 10ms (Zero-Alloc, Absolute Sleep)\n"
                "----------------------------------------------------------------------------\n"
                "Target Heading : %7.2f deg\n"
                "Current Heading: %7.2f deg\n"
                "Heading Error  : %7.2f deg\n"
                "Gyro Yaw Rate  : %7.2f deg/s\n"
                "----------------------------------------------------------------------------\n"
                "PID Correction (u): [%s] (%+6.3f)\n"
                "Left  Wheel Speed : [%s] (%5.1f%%)\n"
                "Right Wheel Speed : [%s] (%5.1f%%)\n"
                "----------------------------------------------------------------------------\n"
                "Press Ctrl+C to exit.\n",
                calib.sys, calib.gyro, calib.accel, calib.mag,
                target_heading_deg, current_heading_deg, heading_error_deg, gyro_z_deg,
                bar_corr, turn_correction,
                bar_left, left_motor * 100.0,
                bar_right, right_motor * 100.0
            );

            if (LIKELY(len > 0)) {
                std::cout.write(output_buffer, len);
                std::cout.flush();
            }

        } catch (const bno055lib::IMUError& e) {
            std::cerr << "Communication error: " << e.what() << "\n";
        }

        // Precise absolute time sleep to prevent loop drift
        std::this_thread::sleep_until(next_wake);
    }

    return 0;
}
