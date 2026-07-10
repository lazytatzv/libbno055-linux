/**
 * @file ros2_lifecycle_publisher_node.cpp
 * @brief Managed Lifecycle (State Machine) ROS 2 node for the BNO055 sensor.
 * 
 * This node implements rclcpp_lifecycle::LifecycleNode to fit into managed
 * robot startup and shutdown sequences. It maps the physical states of the 
 * BNO055 hardware to ROS 2 lifecycle states:
 * - on_configure: Boot the sensor and put it into low-power suspend mode.
 * - on_activate: Wake up the sensor to normal mode and start the timer/publisher.
 * - on_deactivate: Stop publishing and put the hardware back to suspend mode.
 * - on_cleanup: Release I2C file descriptors and reset resources.
 */

#include <chrono>
#include <memory>
#include <utility>
#include <algorithm>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "libbno055-linux/bno055.hpp"

namespace bno055_ros2 {

class BNO055LifecyclePublisherNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit BNO055LifecyclePublisherNode(const rclcpp::NodeOptions& options)
        : LifecycleNode("bno055_lifecycle_publisher_node", options), imu_(0x28, "/dev/i2c-1") 
    {
        // Declare parameters (allowed in constructor or on_configure)
        this->declare_parameter<std::string>("device", "/dev/i2c-1");
        this->declare_parameter<int>("address", 0x28);
        this->declare_parameter<std::string>("frame_id", "imu_link");
        this->declare_parameter<double>("publish_rate", 100.0);
    }

    // Configure Transition: Initialize hardware, setup communications
    CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override {
        (void)state;
        std::string device = this->get_parameter("device").as_string();
        int address = this->get_parameter("address").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double rate_hz = this->get_parameter("publish_rate").as_double();

        RCLCPP_INFO(this->get_logger(), "Configuring BNO055: device=%s, address=0x%02X", device.c_str(), address);

        imu_ = bno055lib::BNO055(address, device);

        // Redirect internal logs into ROS 2 RCLCPP
        imu_.setLogger([this](bno055lib::LogLevel level, std::string_view message) {
            switch (level) {
                case bno055lib::LogLevel::Debug: RCLCPP_DEBUG(this->get_logger(), "%s", message.data()); break;
                case bno055lib::LogLevel::Info: RCLCPP_INFO(this->get_logger(), "%s", message.data()); break;
                case bno055lib::LogLevel::Warning: RCLCPP_WARN(this->get_logger(), "%s", message.data()); break;
                case bno055lib::LogLevel::Error: RCLCPP_ERROR(this->get_logger(), "%s", message.data()); break;
            }
        });

        // Initialize BNO055
        if (!imu_.begin(bno055lib::OpMode::NDOF)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize BNO055 during configuration!");
            return CallbackReturn::FAILURE;
        }

        // Put device to low power suspend mode until activated to save energy
        imu_.enterSuspendMode();

        // Create lifecycle publisher
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        // Setup timer
        auto interval = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = this->create_wall_timer(interval, std::bind(&BNO055LifecyclePublisherNode::timer_callback, this));
        
        // Explicitly cancel the timer so it doesn't run until active
        timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "Configuration successful. Device suspended. Ready to activate.");
        return CallbackReturn::SUCCESS;
    }

    // Activate Transition: Transition to active state, resume hardware and resume publishing
    CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "Activating BNO055...");
        
        // Wake up BNO055 sensor
        imu_.enterNormalMode();

        // Activate lifecycle publisher
        publisher_->on_activate();

        // Restart timer
        timer_->reset();

        RCLCPP_INFO(this->get_logger(), "Activation successful. Publishing started.");
        return CallbackReturn::SUCCESS;
    }

    // Deactivate Transition: Transition to inactive state, stop publishing and suspend hardware
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "Deactivating BNO055...");

        // Stop timer
        timer_->cancel();

        // Deactivate publisher
        publisher_->on_deactivate();

        // Suspend sensor to save power
        imu_.enterSuspendMode();

        RCLCPP_INFO(this->get_logger(), "Deactivation successful. Sensor suspended.");
        return CallbackReturn::SUCCESS;
    }

    // Cleanup Transition: Clean up resources
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "Cleaning up BNO055 resources...");

        // Release structures
        timer_.reset();
        publisher_.reset();

        RCLCPP_INFO(this->get_logger(), "Cleanup successful.");
        return CallbackReturn::SUCCESS;
    }

    // Shutdown Transition: Handle final termination
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "Shutting down BNO055 Node...");
        
        // Ensure device is suspended
        imu_.enterSuspendMode();

        timer_.reset();
        publisher_.reset();

        return CallbackReturn::SUCCESS;
    }

private:
    void timer_callback() {
        // High-performance noexcept reads
        auto quat = imu_.getQuaternionNoexcept();
        auto gyro = imu_.getGyroscopeNoexcept();
        auto accel = imu_.getLinearAccelerationNoexcept();

        if (!quat || !gyro || !accel) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Communication dropout (Lifecycle). Diagnostics: RxErr=%u, TxErr=%u, Reconnects=%u",
                                 imu_.getDiagnostics().read_failures, imu_.getDiagnostics().write_failures,
                                 imu_.getDiagnostics().reconnect_attempts);
            return;
        }

        // Allocate std::unique_ptr for zero-copy intra-process support
        auto message = std::make_unique<sensor_msgs::msg::Imu>();
        
        message->header.stamp = this->now();
        message->header.frame_id = frame_id_;

        // Fill dynamic orientation
        message->orientation.w = quat->w;
        message->orientation.x = quat->x;
        message->orientation.y = quat->y;
        message->orientation.z = quat->z;

        // Fill dynamic angular velocity (rad/s)
        message->angular_velocity.x = gyro->x;
        message->angular_velocity.y = gyro->y;
        message->angular_velocity.z = gyro->z;

        // Fill dynamic linear acceleration (m/s^2)
        message->linear_acceleration.x = accel->x;
        message->linear_acceleration.y = accel->y;
        message->linear_acceleration.z = accel->z;

        // Set covariance to 0 (unknown)
        std::fill(message->orientation_covariance.begin(), message->orientation_covariance.end(), 0.0);
        std::fill(message->angular_velocity_covariance.begin(), message->angular_velocity_covariance.end(), 0.0);
        std::fill(message->linear_acceleration_covariance.begin(), message->linear_acceleration_covariance.end(), 0.0);

        // Publish using std::move to enable zero-copy intra-process transport
        publisher_->publish(std::move(message));
    }

    bno055lib::BNO055 imu_;
    std::string frame_id_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace bno055_ros2

#ifndef ROS2_NODE_TESTING
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    // Enable intra-process communication by default for high performance
    options.use_intra_process_comms(true);

    auto node = std::make_shared<bno055_ros2::BNO055LifecyclePublisherNode>(options);

    try {
        rclcpp::spin(node->get_node_base_interface());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Lifecycle Node terminated: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
#endif
