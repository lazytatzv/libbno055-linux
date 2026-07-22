#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "libbno055-linux/controllers/heading_controller.hpp"

namespace bno055_ros2 {

class BNO055HeadingControlNode : public rclcpp::Node {
public:
    explicit BNO055HeadingControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("bno055_heading_control_node", options),
          last_time_(this->now()),
          target_heading_locked_(false) {

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

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, rclcpp::SystemDefaultsQoS());
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(),
            std::bind(&BNO055HeadingControlNode::imuCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "[Production] BNO055 Heading PID Controller Node online.");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) noexcept {
        const rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (BNO055_UNLIKELY(dt <= 0.0 || dt > 1.0)) dt = 0.01;  // Fallback 100Hz

        const double current_heading_deg = bno055lib::fastExtractYawDeg(
            msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        const double gyro_z_deg = msg->angular_velocity.z * bno055lib::RAD_TO_DEG;

        controller_.setGains(this->get_parameter("kp").as_double(),
                             this->get_parameter("ki").as_double(),
                             this->get_parameter("kd").as_double());

        if (BNO055_UNLIKELY(this->get_parameter("auto_lock_initial_heading").as_bool() && !target_heading_locked_)) {
            target_heading_deg_ = current_heading_deg;
            target_heading_locked_ = true;
            RCLCPP_INFO(this->get_logger(), "Locked initial target heading: %.2f deg", target_heading_deg_);
        } else if (!target_heading_locked_) {
            target_heading_deg_ = this->get_parameter("target_heading_deg").as_double();
        }

        const double base_speed = this->get_parameter("base_linear_speed").as_double();
        auto out = controller_.update(target_heading_deg_, current_heading_deg, dt, gyro_z_deg, base_speed);

        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        twist_msg->linear.x = base_speed;
        twist_msg->angular.z = out.correction;
        cmd_vel_pub_->publish(std::move(twist_msg));
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    bno055lib::HeadingController controller_;
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
