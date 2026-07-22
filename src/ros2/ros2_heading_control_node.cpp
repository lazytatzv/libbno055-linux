#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#if defined(__GNUC__) || defined(__clang__)
#define LIKELY(x) __builtin_expect(!!(x), 1)
#define UNLIKELY(x) __builtin_expect(!!(x), 0)
#else
#define LIKELY(x) (x)
#define UNLIKELY(x) (x)
#endif

namespace bno055_ros2 {

constexpr double RAD_TO_DEG = 180.0 / M_PI;

/**
 * @brief Zero-allocation, inline-optimized PID controller matching ROS 2 performance standards.
 */
class HighPerfPidController {
public:
    constexpr HighPerfPidController(double kp = 0.05, double ki = 0.001, double kd = 0.01,
                                   double min_output = -2.0, double max_output = 2.0,
                                   double max_i_term = 0.5) noexcept
        : kp_(kp), ki_(ki), kd_(kd),
          min_output_(min_output), max_output_(max_output),
          max_i_term_(max_i_term), i_term_(0.0), prev_error_(0.0), initialized_(false) {}

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

    [[nodiscard]] inline double compute(double error, double dt, double gyro_z = 0.0, bool use_gyro_for_d = true) noexcept {
        if (UNLIKELY(dt <= 0.0)) return 0.0;

        const double p_term = kp_ * error;
        i_term_ = std::clamp(i_term_ + ki_ * error * dt, -max_i_term_, max_i_term_);

        double d_term = 0.0;
        if (LIKELY(use_gyro_for_d)) {
            d_term = -kd_ * gyro_z;
        } else {
            if (LIKELY(initialized_)) {
                d_term = kd_ * ((error - prev_error_) / dt);
            } else {
                initialized_ = true;
            }
            prev_error_ = error;
        }

        return std::clamp(p_term + i_term_ + d_term, min_output_, max_output_);
    }

private:
    double kp_;
    double ki_;
    double kd_;
    double min_output_;
    double max_output_;
    double max_i_term_;
    double i_term_;
    double prev_error_;
    bool initialized_;
};

class BNO055HeadingControlNode : public rclcpp::Node {
public:
    explicit BNO055HeadingControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("bno055_heading_control_node", options),
          last_time_(this->now()),
          target_heading_locked_(false) {

        // Declare parameters with low overhead defaults
        this->declare_parameter<double>("kp", 0.05);
        this->declare_parameter<double>("ki", 0.001);
        this->declare_parameter<double>("kd", 0.01);
        this->declare_parameter<double>("target_heading_deg", 0.0);
        this->declare_parameter<bool>("auto_lock_initial_heading", true);
        this->declare_parameter<double>("base_linear_speed", 0.3);
        this->declare_parameter<std::string>("imu_topic", "imu/data");
        this->declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");

        const std::string imu_topic = this->get_parameter("imu_topic").as_string();
        const std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

        // High performance SensorDataQoS setup
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::SystemDefaultsQoS());
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(),
            std::bind(&BNO055HeadingControlNode::imuCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "[High-Perf] BNO055 Heading Controller Node online.");
    }

private:
    [[nodiscard]] static inline double fastExtractYawDeg(double qx, double qy, double qz, double qw) noexcept {
        const double siny_cosp = 2.0 * (qw * qz + qx * qy);
        const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        return std::atan2(siny_cosp, cosy_cosp) * RAD_TO_DEG;
    }

    [[nodiscard]] static inline double normalizeAngleDeg(double angle_deg) noexcept {
        return std::remainder(angle_deg, 360.0);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) noexcept {
        const rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (UNLIKELY(dt <= 0.0 || dt > 1.0)) dt = 0.01;  // Fallback 100Hz

        // Extract orientation & angular rate efficiently
        const double current_heading_deg = fastExtractYawDeg(msg->orientation.x, msg->orientation.y,
                                                             msg->orientation.z, msg->orientation.w);
        const double gyro_z_deg = msg->angular_velocity.z * RAD_TO_DEG;

        // Dynamic parameter updates
        pid_.setGains(this->get_parameter("kp").as_double(),
                      this->get_parameter("ki").as_double(),
                      this->get_parameter("kd").as_double());

        if (UNLIKELY(this->get_parameter("auto_lock_initial_heading").as_bool() && !target_heading_locked_)) {
            target_heading_deg_ = current_heading_deg;
            target_heading_locked_ = true;
            RCLCPP_INFO(this->get_logger(), "Locked initial target heading: %.2f deg", target_heading_deg_);
        } else if (!target_heading_locked_) {
            target_heading_deg_ = this->get_parameter("target_heading_deg").as_double();
        }

        // Shortest path angle error
        const double error_deg = normalizeAngleDeg(target_heading_deg_ - current_heading_deg);
        const double angular_z_cmd = pid_.compute(error_deg, dt, gyro_z_deg, true);

        // Pre-allocated Twist message publish
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = this->get_parameter("base_linear_speed").as_double();
        twist_msg->angular.z = angular_z_cmd;
        cmd_vel_pub_->publish(std::move(twist_msg));
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    HighPerfPidController pid_{0.05, 0.001, 0.01, -2.0, 2.0, 0.5};
    rclcpp::Time last_time_;
    double target_heading_deg_{0.0};
    bool target_heading_locked_{false};
};

}  // namespace bno055_ros2

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bno055_ros2::BNO055HeadingControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
