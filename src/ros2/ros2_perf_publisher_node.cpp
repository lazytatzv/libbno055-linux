/**
 * @file ros2_perf_publisher_node.cpp
 * @brief High-performance ROS 2 publisher node optimized for low-latency and zero-copy transport.
 *
 * This node is tailored for resource-constrained embedded Linux systems.
 * It uses ROS 2 intra-process communication features and passes message pointers
 * via std::unique_ptr and std::move. In a single-process composable container,
 * this design completely bypasses message serialization and copy overhead.
 * It also uses exception-free (noexcept) APIs to maintain deterministic cycle times.
 */

#include <algorithm>
#include <chrono>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <utility>

#include "libbno055-linux/bno055.hpp"

using namespace std::chrono_literals;

namespace bno055_ros2 {

class BNO055PerfPublisherNode : public rclcpp::Node {
public:
    explicit BNO055PerfPublisherNode(const rclcpp::NodeOptions& options)
        : Node("bno055_perf_publisher_node", options), imu_(0x28, "/dev/i2c-1") {
        // Declare parameters for runtime configuration
        this->declare_parameter<std::string>("device", "/dev/i2c-1");
        this->declare_parameter<int>("address", 0x28);
        this->declare_parameter<std::string>("frame_id", "imu_link");
        this->declare_parameter<double>("publish_rate", 100.0);  // BNO055 Internal Fusion is 100Hz max

        // Advanced ROS 2 & Sensor configurations
        this->declare_parameter<std::string>("qos_reliability", "best_effort");
        this->declare_parameter<int>("qos_history_depth", 10);
        this->declare_parameter<std::string>("calibration_file", "");
        this->declare_parameter<std::vector<double>>("orientation_covariance", std::vector<double>(9, 0.0));
        this->declare_parameter<std::vector<double>>("angular_velocity_covariance", std::vector<double>(9, 0.0));
        this->declare_parameter<std::vector<double>>("linear_acceleration_covariance", std::vector<double>(9, 0.0));

        std::string device = this->get_parameter("device").as_string();
        int address = this->get_parameter("address").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        double rate_hz = this->get_parameter("publish_rate").as_double();
        std::string calib_file = this->get_parameter("calibration_file").as_string();

        RCLCPP_INFO(this->get_logger(), "Initializing High-Performance BNO055 Node on %s (address: 0x%02X)",
                    device.c_str(), address);

        imu_ = bno055lib::BNO055(address, device);

        // Redirect internal logs into ROS 2 RCLCPP
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

        // Use high-performance NDOF mode (includes sensor fusion)
        if (!imu_.begin(bno055lib::OpMode::NDOF)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to initialize BNO055 sensor!");
            throw std::runtime_error("Sensor initialization failed");
        }

        // Load calibration file if specified
        if (!calib_file.empty()) {
            if (imu_.loadCalibrationFile(calib_file)) {
                RCLCPP_INFO(this->get_logger(), "Loaded calibration offsets from: %s", calib_file.c_str());
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to load calibration file: %s", calib_file.c_str());
            }
        }

        // Setup dynamic QoS
        auto qos = rclcpp::SensorDataQoS();
        std::string reliability = this->get_parameter("qos_reliability").as_string();
        if (reliability == "reliable") {
            qos.reliable();
        } else if (reliability == "best_effort") {
            qos.best_effort();
        }
        qos.keep_last(this->get_parameter("qos_history_depth").as_int());

        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
        diag_publisher_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

        auto interval = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = this->create_wall_timer(interval, std::bind(&BNO055PerfPublisherNode::timer_callback, this));
        diag_timer_ = this->create_wall_timer(1s, std::bind(&BNO055PerfPublisherNode::publish_diagnostics, this));

        RCLCPP_INFO(this->get_logger(), "IMU Perf Node started. Publishing on 'imu/data' at %.1f Hz", rate_hz);
    }

private:
    void timer_callback() {
        // Record timestamp immediately before I2C communication starts to minimize jitter
        auto stamp = this->now();

        // High-performance exception-free (noexcept) reads
        auto quat = imu_.getQuaternionNoexcept();
        auto gyro = imu_.getGyroscopeNoexcept();
        auto accel = imu_.getLinearAccelerationNoexcept();  // Acceleration excluding gravity

        if (!quat || !gyro || !accel) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "Communication dropout. Diagnostics: RxErr=%u, TxErr=%u, Reconnects=%u",
                                 imu_.getDiagnostics().read_failures, imu_.getDiagnostics().write_failures,
                                 imu_.getDiagnostics().reconnect_attempts);
            return;
        }

        // Allocate standard message dynamically as a unique_ptr to enable zero-copy intra-process transport.
        // When running in a single process container, ROS 2 bypasses serialization and passes this pointer directly.
        auto message = std::make_unique<sensor_msgs::msg::Imu>();

        message->header.stamp = stamp;
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

        // Set covariances from parameters
        auto ori_cov = this->get_parameter("orientation_covariance").as_double_array();
        auto gyro_cov = this->get_parameter("angular_velocity_covariance").as_double_array();
        auto accel_cov = this->get_parameter("linear_acceleration_covariance").as_double_array();

        if (ori_cov.size() == 9) std::copy(ori_cov.begin(), ori_cov.end(), message->orientation_covariance.begin());
        if (gyro_cov.size() == 9)
            std::copy(gyro_cov.begin(), gyro_cov.end(), message->angular_velocity_covariance.begin());
        if (accel_cov.size() == 9)
            std::copy(accel_cov.begin(), accel_cov.end(), message->linear_acceleration_covariance.begin());

        // Publish using std::move to enable zero-copy pointer pass
        publisher_->publish(std::move(message));
    }

    void publish_diagnostics() {
        auto diag_arr = diagnostic_msgs::msg::DiagnosticArray();
        diag_arr.header.stamp = this->now();

        auto status = diagnostic_msgs::msg::DiagnosticStatus();
        status.name = "libbno055_linux: IMU Perf Sensor Monitor";
        status.hardware_id =
            this->get_parameter("device").as_string() + ":" + std::to_string(this->get_parameter("address").as_int());

        auto diag = imu_.getDiagnostics();

        auto add_key_value = [](diagnostic_msgs::msg::DiagnosticStatus& stat, const std::string& key,
                                const std::string& val) {
            auto kv = diagnostic_msgs::msg::KeyValue();
            kv.key = key;
            kv.value = val;
            stat.values.push_back(kv);
        };

        add_key_value(status, "I2C Read Failures", std::to_string(diag.read_failures));
        add_key_value(status, "I2C Write Failures", std::to_string(diag.write_failures));
        add_key_value(status, "I2C Reconnect Attempts", std::to_string(diag.reconnect_attempts));

        bno055lib::CalibrationStatus calib;
        bool calib_ok = false;
        try {
            calib = imu_.getCalibrationStatus();
            calib_ok = true;
        } catch (const std::exception& e) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = std::string("Failed to query calibration status: ") + e.what();
        }

        if (calib_ok) {
            add_key_value(status, "Calibration Status: Sys", std::to_string(calib.sys));
            add_key_value(status, "Calibration Status: Gyro", std::to_string(calib.gyro));
            add_key_value(status, "Calibration Status: Accel", std::to_string(calib.accel));
            add_key_value(status, "Calibration Status: Mag", std::to_string(calib.mag));

            if (calib.isFullyCalibrated()) {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                status.message = "Fully Calibrated & Streaming";
            } else {
                status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                status.message = "Calibration incomplete (Sys:" + std::to_string(calib.sys) +
                                 " G:" + std::to_string(calib.gyro) + " A:" + std::to_string(calib.accel) +
                                 " M:" + std::to_string(calib.mag) + ")";
            }
        }

        if (diag.reconnect_attempts > 5) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message =
                "Unstable I2C connection. Reconnected " + std::to_string(diag.reconnect_attempts) + " times.";
        }

        diag_arr.status.push_back(status);
        diag_publisher_->publish(diag_arr);
    }

    bno055lib::BNO055 imu_;
    std::string frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace bno055_ros2

#ifndef ROS2_NODE_TESTING
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    // Enable intra-process communication by default for high performance
    options.use_intra_process_comms(true);

    try {
        rclcpp::spin(std::make_shared<bno055_ros2::BNO055PerfPublisherNode>(options));
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "High-Performance Node terminated: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
#endif
