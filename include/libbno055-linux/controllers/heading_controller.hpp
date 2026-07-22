#ifndef BNO055_HEADING_CONTROLLER_HPP
#define BNO055_HEADING_CONTROLLER_HPP

#include <algorithm>
#include <cmath>

#if defined(__GNUC__) || defined(__clang__)
#define BNO055_LIKELY(x)   __builtin_expect(!!(x), 1)
#define BNO055_UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#define BNO055_LIKELY(x)   (x)
#define BNO055_UNLIKELY(x) (x)
#endif

namespace bno055lib {

constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr double DEG_TO_RAD = M_PI / 180.0;

/**
 * @brief Normalizes angle into [-180.0, +180.0] degrees range using std::remainder.
 */
[[nodiscard]] constexpr inline double normalizeAngleDeg(double angle_deg) noexcept {
    return std::remainder(angle_deg, 360.0);
}

/**
 * @brief Fast Yaw extraction directly from Quaternion (W, X, Y, Z) in degrees.
 * Avoids extra Roll & Pitch trigonometric calculations.
 */
[[nodiscard]] inline double fastExtractYawDeg(double qw, double qx, double qy, double qz) noexcept {
    const double siny_cosp = 2.0 * (qw * qz + qx * qy);
    const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

/**
 * @brief Production-grade, zero-allocation Heading PID Controller for Mobile Robots.
 * Designed for real-time Embedded & ROS 2 applications.
 */
class HeadingController {
public:
    struct Config {
        double kp{0.05};
        double ki{0.001};
        double kd{0.01};
        double min_output{-1.0};
        double max_output{1.0};
        double max_i_term{0.2};
    };

    struct Output {
        double correction{0.0};   ///< PID correction output u (rad/s or differential velocity)
        double left_motor{0.0};   ///< Left wheel speed normalized [0.0, 1.0]
        double right_motor{0.0};  ///< Right wheel speed normalized [0.0, 1.0]
        double error_deg{0.0};    ///< Shortest heading error in degrees
    };

    HeadingController() noexcept : config_(), i_term_(0.0), prev_error_(0.0), initialized_(false) {}
    explicit HeadingController(const Config& config) noexcept
        : config_(config), i_term_(0.0), prev_error_(0.0), initialized_(false) {}

    inline void setGains(double kp, double ki, double kd) noexcept {
        config_.kp = kp;
        config_.ki = ki;
        config_.kd = kd;
    }

    inline void setConfig(const Config& config) noexcept { config_ = config; }

    [[nodiscard]] inline const Config& getConfig() const noexcept { return config_; }

    inline void reset() noexcept {
        i_term_ = 0.0;
        prev_error_ = 0.0;
        initialized_ = false;
    }

    /**
     * @brief Computes PID correction given current & target heading in degrees.
     * @param target_heading_deg Desired target heading in [-180, 180] deg
     * @param current_heading_deg Current IMU heading in [-180, 180] deg
     * @param dt Time delta in seconds
     * @param gyro_z_deg Yaw angular velocity in deg/s (for noise-free derivative)
     * @param base_velocity Base forward velocity [0.0, 1.0]
     */
    [[nodiscard]] inline Output update(double target_heading_deg, double current_heading_deg, double dt,
                                       double gyro_z_deg = 0.0, double base_velocity = 0.5) noexcept {
        Output out{};
        if (BNO055_UNLIKELY(dt <= 0.0)) {
            out.left_motor = std::clamp(base_velocity, 0.0, 1.0);
            out.right_motor = std::clamp(base_velocity, 0.0, 1.0);
            return out;
        }

        // 1. Shortest path angle difference (-180 to +180)
        out.error_deg = normalizeAngleDeg(target_heading_deg - current_heading_deg);

        // 2. Proportional
        const double p_term = config_.kp * out.error_deg;

        // 3. Integral with Anti-windup clamping
        i_term_ = std::clamp(i_term_ + config_.ki * out.error_deg * dt, -config_.max_i_term, config_.max_i_term);

        // 4. Derivative (Prefer direct gyro rate to eliminate noise & derivative kick)
        double d_term = 0.0;
        if (BNO055_LIKELY(gyro_z_deg != 0.0 || !initialized_)) {
            d_term = -config_.kd * gyro_z_deg;
        } else {
            const double error_dot = (out.error_deg - prev_error_) / dt;
            d_term = config_.kd * error_dot;
        }

        prev_error_ = out.error_deg;
        initialized_ = true;

        // 5. Total Correction Output u
        out.correction = std::clamp(p_term + i_term_ + d_term, config_.min_output, config_.max_output);

        // 6. Differential motor speeds
        out.left_motor = std::clamp(base_velocity - out.correction, 0.0, 1.0);
        out.right_motor = std::clamp(base_velocity + out.correction, 0.0, 1.0);

        return out;
    }

private:
    Config config_;
    double i_term_;
    double prev_error_;
    bool initialized_;
};

}  // namespace bno055lib

#endif  // BNO055_HEADING_CONTROLLER_HPP
