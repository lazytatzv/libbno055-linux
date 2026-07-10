/**
 * @file ros2_publisher_node.cpp
 * @brief Standard standalone ROS 2 publisher node for the BNO055 IMU sensor.
 * 
 * This node initializes the BNO055 sensor in IMUPlus fusion mode (6-axis),
 * declares standard ROS 2 parameters for runtime configuration, redirects
 * internal library logs to RCLCPP, and publishes IMU telemetry data 
 * (sensor_msgs/msg/Imu) at a configurable frequency.
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "libbno055-linux/bno055.hpp"

using namespace std::chrono_literals;

class BNO055PublisherNode : public rclcpp::Node {
public:
    BNO055PublisherNode() : Node("bno055_publisher_node"), imu_(0x28, "/dev/i2c-1") {
        // Declare parameters for runtime configuration
        this->declare_parameter<std::string>("device", "/dev/i2c-1");
        this->declare_parameter<int>("address", 0x28);
        this->declare_parameter<std::string>("frame_id", "imu_link");
        this->declare_parameter<double>("publish_rate", 50.0);  // 50 Hz

        std::string device = this->get_parameter("device").as_string();
        int address = this->get_parameter("address").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double rate_hz = this->get_parameter("publish_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "Initializing BNO055 on %s (address: 0x%02X)", device.c_str(), address);

        imu_ = bno055lib::BNO055(address, device);

        // Redirect library internal logs into ROS 2 RCLCPP logger
        imu_.setLogger([this](bno055lib::LogLevel level, std::string_view message) {
            switch (level) {
                case bno055lib::LogLevel::Debug:
                    RCLCPP_DEBUG(this->get_logger(), "%s", message.data());
                    break;
                case bno055lib::LogLevel::Info:
                    RCLCPP_INFO(this->get_logger(), "%s", message.data());
                    break;
                case bno055lib::LogLevel::Warning:
                    RCLCPP_WARN(this->get_logger(), "%s", message.data());
                    break;
                case bno055lib::LogLevel::Error:
                    RCLCPP_ERROR(this->get_logger(), "%s", message.data());
                    break;
            }
        });

        // 6-axis Fusion (IMUPlus) is recommended for indoor robotics to avoid magnetic yaw drift.
        // Change to OpMode::NDOF if absolute North orientation is required.
        if (!imu_.begin(bno055lib::OpMode::IMUPlus)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize BNO055 sensor!");
            throw std::runtime_error("Sensor initialization failed");
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        auto interval = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = this->create_wall_timer(interval, std::bind(&BNO055PublisherNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "IMU Node started. Publishing on 'imu/data' at %.1f Hz", rate_hz);
    }

private:
    void timer_callback() {
        // Exception-free (noexcept) read path ensures no CPU overhead on communication drops
        auto quat = imu_.getQuaternionNoexcept();
        auto gyro = imu_.getGyroscopeNoexcept();
        auto accel = imu_.getLinearAccelerationNoexcept();  // Acceleration excluding gravity

        if (!quat || !gyro || !accel) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Communication dropout detected. Diagnostics: RxErr=%u, TxErr=%u, Reconnects=%u",
                                 imu_.getDiagnostics().read_failures, imu_.getDiagnostics().write_failures,
                                 imu_.getDiagnostics().reconnect_attempts);
            return;
        }

        auto message = sensor_msgs::msg::Imu();
        message.header.stamp = this->now();
        message.header.frame_id = frame_id_;

        // Fill Orientation
        message.orientation.w = quat->w;
        message.orientation.x = quat->x;
        message.orientation.y = quat->y;
        message.orientation.z = quat->z;

        // Fill Angular Velocity (rad/s)
        message.angular_velocity.x = gyro->x;
        message.angular_velocity.y = gyro->y;
        message.angular_velocity.z = gyro->z;

        // Fill Linear Acceleration (m/s^2)
        message.linear_acceleration.x = accel->x;
        message.linear_acceleration.y = accel->y;
        message.linear_acceleration.z = accel->z;

        // Set covariance to 0 (unknown)
        std::fill(message.orientation_covariance.begin(), message.orientation_covariance.end(), 0.0);
        std::fill(message.angular_velocity_covariance.begin(), message.angular_velocity_covariance.end(), 0.0);
        std::fill(message.linear_acceleration_covariance.begin(), message.linear_acceleration_covariance.end(), 0.0);

        publisher_->publish(message);
    }

    bno055lib::BNO055 imu_;
    std::string frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#ifndef ROS2_NODE_TESTING
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<BNO055PublisherNode>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Node terminated due to exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
#endif
