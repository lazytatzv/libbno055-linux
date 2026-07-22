#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "libbno055-linux/controllers/heading_controller.hpp"

namespace bno055_ros2 {

/**
 * @brief Production-Grade ROS 2 Composable Heading Corrector Node with Watchdog Safety.
 * Features: Zero-Copy Intra-Process transport, Dynamic Parameters, Diagnostics, Reset Service,
 * and a Safety Watchdog Timer to command zero velocity upon command timeout.
 */
class BNO055HeadingControlNode : public rclcpp::Node {
public:
    explicit BNO055HeadingControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("bno055_heading_control_node", options),
          last_time_(this->now()),
          last_cmd_vel_in_time_(this->now()),
          target_heading_locked_(false),
          current_heading_deg_(0.0),
          gyro_z_deg_(0.0),
          has_imu_data_(false),
          has_cmd_vel_in_(false),
          is_watchdog_triggered_(false),
          last_correction_(0.0),
          last_error_deg_(0.0) {

        // 1. Declare Parameters
        this->declare_parameter<double>("kp", 0.05);
        this->declare_parameter<double>("ki", 0.001);
        this->declare_parameter<double>("kd", 0.01);
        this->declare_parameter<double>("max_i_term", 0.2);
        this->declare_parameter<double>("max_output", 1.0);
        this->declare_parameter<double>("angular_deadband", 0.01);
        this->declare_parameter<double>("cmd_vel_timeout", 0.5);  // Safety Watchdog timeout in seconds
        this->declare_parameter<std::string>("imu_topic", "imu/data");
        this->declare_parameter<std::string>("cmd_vel_in_topic", "cmd_vel_in");
        this->declare_parameter<std::string>("cmd_vel_out_topic", "cmd_vel");
        this->declare_parameter<bool>("enable_diagnostics", true);

        updateControllerConfigFromParams();

        // 2. Dynamic Parameters Callback
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&BNO055HeadingControlNode::onParameterChange, this, std::placeholders::_1));

        // 3. Topics & Zero-Copy Transport
        const std::string imu_topic = this->get_parameter("imu_topic").as_string();
        const std::string cmd_vel_in_topic = this->get_parameter("cmd_vel_in_topic").as_string();
        const std::string cmd_vel_out_topic = this->get_parameter("cmd_vel_out_topic").as_string();

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic, rclcpp::SystemDefaultsQoS());
        diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", rclcpp::SystemDefaultsQoS());

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(),
            std::bind(&BNO055HeadingControlNode::imuCallback, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_in_topic, 10,
            std::bind(&BNO055HeadingControlNode::cmdVelInCallback, this, std::placeholders::_1));

        // 4. Trigger Service
        reset_heading_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "~/reset_heading",
            std::bind(&BNO055HeadingControlNode::handleResetHeadingService, this,
                      std::placeholders::_1, std::placeholders::_2));

        // 5. Watchdog Safety Timer (Checking at 20Hz / 50ms)
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&BNO055HeadingControlNode::checkWatchdogTimeout, this));

        // 6. Diagnostics Timer (1Hz)
        if (this->get_parameter("enable_diagnostics").as_bool()) {
            diag_timer_ = this->create_wall_timer(
                std::chrono::seconds(1),
                std::bind(&BNO055HeadingControlNode::publishDiagnostics, this));
        }

        RCLCPP_INFO(this->get_logger(), "[Production Composable Node] BNO055 Heading Corrector Node online with Safety Watchdog.");
    }

private:
    void updateControllerConfigFromParams() noexcept {
        bno055lib::HeadingController::Config cfg;
        cfg.kp = this->get_parameter("kp").as_double();
        cfg.ki = this->get_parameter("ki").as_double();
        cfg.kd = this->get_parameter("kd").as_double();
        cfg.max_i_term = this->get_parameter("max_i_term").as_double();
        cfg.max_output = this->get_parameter("max_output").as_double();
        cfg.min_output = -cfg.max_output;
        controller_.setConfig(cfg);
    }

    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter>& parameters) {

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto& param : parameters) {
            if (param.get_name() == "kp" || param.get_name() == "ki" ||
                param.get_name() == "kd" || param.get_name() == "max_i_term" ||
                param.get_name() == "max_output" || param.get_name() == "cmd_vel_timeout") {
                RCLCPP_INFO(this->get_logger(), "Dynamic parameter updated: %s = %f",
                            param.get_name().c_str(), param.as_double());
            }
        }
        updateControllerConfigFromParams();
        return result;
    }

    void handleResetHeadingService(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

        if (has_imu_data_) {
            target_quat_ = current_quat_;
            target_heading_deg_ = current_heading_deg_;
            target_heading_locked_ = true;
            controller_.reset();
            res->success = true;
            res->message = "Heading target successfully reset to current orientation: " +
                           std::to_string(target_heading_deg_) + " deg";
            RCLCPP_INFO(this->get_logger(), "%s", res->message.c_str());
        } else {
            res->success = false;
            res->message = "Cannot reset heading: No valid IMU data received yet.";
            RCLCPP_WARN(this->get_logger(), "%s", res->message.c_str());
        }
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) noexcept {
        current_quat_ = bno055lib::Quat{msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z};
        current_heading_deg_ = bno055lib::fastExtractYawDeg(current_quat_);
        gyro_z_deg_ = msg->angular_velocity.z * bno055lib::RAD_TO_DEG;
        has_imu_data_ = true;
    }

    void cmdVelInCallback(const geometry_msgs::msg::Twist::SharedPtr msg) noexcept {
        const rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        last_cmd_vel_in_time_ = now;
        has_cmd_vel_in_ = true;

        if (is_watchdog_triggered_) {
            RCLCPP_INFO(this->get_logger(), "Watchdog disengaged: cmd_vel_in command resumed.");
            is_watchdog_triggered_ = false;
        }

        if (BNO055_UNLIKELY(dt <= 0.0 || dt > 1.0)) dt = 0.02;

        auto out_twist = std::make_unique<geometry_msgs::msg::Twist>();
        out_twist->linear = msg->linear;

        const double deadband = this->get_parameter("angular_deadband").as_double();
        const bool is_commanded_to_turn = std::abs(msg->angular.z) > deadband;

        if (is_commanded_to_turn || !has_imu_data_) {
            // Passthrough during active turns
            target_heading_locked_ = false;
            controller_.reset();
            out_twist->angular = msg->angular;
            last_correction_ = 0.0;
            last_error_deg_ = 0.0;
        } else {
            // Straight driving -> Lock target heading & apply IMU PID correction
            if (!target_heading_locked_) {
                target_quat_ = current_quat_;
                target_heading_deg_ = current_heading_deg_;
                target_heading_locked_ = true;
            }

            auto out = controller_.update(target_quat_, current_quat_, dt, gyro_z_deg_, msg->linear.x);
            out_twist->angular.z = out.correction;
            last_correction_ = out.correction;
            last_error_deg_ = out.error_deg;
        }

        cmd_vel_pub_->publish(std::move(out_twist));
    }

    /**
     * @brief Safety Watchdog Timer: Safely publishes zero velocity if cmd_vel_in is lost/timed out.
     */
    void checkWatchdogTimeout() {
        if (!has_cmd_vel_in_) return;

        const double timeout = this->get_parameter("cmd_vel_timeout").as_double();
        const double elapsed = (this->now() - last_cmd_vel_in_time_).seconds();

        if (elapsed > timeout) {
            if (!is_watchdog_triggered_) {
                RCLCPP_WARN(this->get_logger(),
                            "Watchdog Timeout Triggered! cmd_vel_in lost for %.2f s. Publishing ZERO VELOCITY.",
                            elapsed);
                is_watchdog_triggered_ = true;
                target_heading_locked_ = false;
                controller_.reset();
            }

            // Continuously publish zero velocity to stop motors safely
            auto stop_twist = std::make_unique<geometry_msgs::msg::Twist>();
            cmd_vel_pub_->publish(std::move(stop_twist));
        }
    }

    void publishDiagnostics() {
        auto diag_arr = std::make_unique<diagnostic_msgs::msg::DiagnosticArray>();
        diag_arr->header.stamp = this->now();

        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = "libbno055_linux: Heading Controller";
        status.hardware_id = "BNO055_PID_Controller";

        if (is_watchdog_triggered_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            status.message = "SAFETY WATCHDOG TRIGGERED: Input cmd_vel_in Timed Out!";
        } else if (!has_imu_data_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            status.message = "Waiting for IMU data...";
        } else if (target_heading_locked_) {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Active Straight Heading Correction";
        } else {
            status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            status.message = "Passthrough (Active Turning Command)";
        }

        auto add_kv = [&status](const std::string& k, const std::string& v) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = k;
            kv.value = v;
            status.values.push_back(kv);
        };

        add_kv("Target Heading (deg)", std::to_string(target_heading_deg_));
        add_kv("Current Heading (deg)", std::to_string(current_heading_deg_));
        add_kv("Heading Error (deg)", std::to_string(last_error_deg_));
        add_kv("PID Correction (rad/s)", std::to_string(last_correction_));
        add_kv("Target Locked", target_heading_locked_ ? "True" : "False");
        add_kv("Watchdog Triggered", is_watchdog_triggered_ ? "True" : "False");

        diag_arr->status.push_back(status);
        diag_pub_->publish(std::move(diag_arr));
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_heading_srv_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    bno055lib::HeadingController controller_;

    rclcpp::Time last_time_;
    rclcpp::Time last_cmd_vel_in_time_;
    bno055lib::Quat current_quat_;
    bno055lib::Quat target_quat_;
    double current_heading_deg_;
    double gyro_z_deg_;
    double target_heading_deg_;
    bool target_heading_locked_;
    bool has_imu_data_;
    bool has_cmd_vel_in_;
    bool is_watchdog_triggered_;
    double last_correction_;
    double last_error_deg_;
};

}  // namespace bno055_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(bno055_ros2::BNO055HeadingControlNode)

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bno055_ros2::BNO055HeadingControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
