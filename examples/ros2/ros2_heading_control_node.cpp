#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "libbno055-linux/controllers/heading_controller.hpp"

namespace bno055_ros2 {

/**
 * @brief Production Twist-In / Twist-Out Heading Corrector Node.
 * Subscribes to raw input velocity (cmd_vel_in) and IMU data (/imu/data).
 * When driving straight (cmd_vel_in.angular.z == 0), locks heading and applies PID correction to output cmd_vel.
 */
class BNO055HeadingControlNode : public rclcpp::Node {
public:
    explicit BNO055HeadingControlNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("bno055_heading_control_node", options),
          last_time_(this->now()),
          target_heading_locked_(false),
          current_heading_deg_(0.0),
          gyro_z_deg_(0.0),
          has_imu_data_(false) {

        // Declare ROS 2 parameters
        this->declare_parameter<double>("kp", 0.05);
        this->declare_parameter<double>("ki", 0.001);
        this->declare_parameter<double>("kd", 0.01);
        this->declare_parameter<std::string>("imu_topic", "imu/data");
        this->declare_parameter<std::string>("cmd_vel_in_topic", "cmd_vel_in");
        this->declare_parameter<std::string>("cmd_vel_out_topic", "cmd_vel");
        this->declare_parameter<double>("angular_deadband", 0.01);  // Threshold for straight driving

        const std::string imu_topic = this->get_parameter("imu_topic").as_string();
        const std::string cmd_vel_in_topic = this->get_parameter("cmd_vel_in_topic").as_string();
        const std::string cmd_vel_out_topic = this->get_parameter("cmd_vel_out_topic").as_string();

        // Subscriptions & Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_out_topic, rclcpp::SystemDefaultsQoS());

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic, rclcpp::SensorDataQoS(),
            std::bind(&BNO055HeadingControlNode::imuCallback, this, std::placeholders::_1));

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_in_topic, 10,
            std::bind(&BNO055HeadingControlNode::cmdVelInCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "[Production] BNO055 Twist-In/Twist-Out Heading Corrector Node online.");
        RCLCPP_INFO(this->get_logger(), "Listening: %s -> Outputting: %s (IMU: %s)",
                    cmd_vel_in_topic.c_str(), cmd_vel_out_topic.c_str(), imu_topic.c_str());
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) noexcept {
        current_heading_deg_ = bno055lib::fastExtractYawDeg(
            msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
        gyro_z_deg_ = msg->angular_velocity.z * bno055lib::RAD_TO_DEG;
        has_imu_data_ = true;
    }

    void cmdVelInCallback(const geometry_msgs::msg::Twist::SharedPtr msg) noexcept {
        const rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (BNO055_UNLIKELY(dt <= 0.0 || dt > 1.0)) dt = 0.02;

        // Update PID parameters
        controller_.setGains(this->get_parameter("kp").as_double(),
                             this->get_parameter("ki").as_double(),
                             this->get_parameter("kd").as_double());

        auto out_twist = std::make_unique<geometry_msgs::msg::Twist>();
        out_twist->linear = msg->linear;

        const double deadband = this->get_parameter("angular_deadband").as_double();
        const bool is_commanded_to_turn = std::abs(msg->angular.z) > deadband;

        if (is_commanded_to_turn || !has_imu_data_) {
            // User/Nav2 is commanding an active turn -> unlock target heading & passthrough
            target_heading_locked_ = false;
            controller_.reset();
            out_twist->angular = msg->angular;
        } else {
            // Straight driving or stationary -> Lock target heading and apply IMU PID correction
            if (!target_heading_locked_) {
                target_heading_deg_ = current_heading_deg_;
                target_heading_locked_ = true;
            }

            auto out = controller_.update(target_heading_deg_, current_heading_deg_, dt, gyro_z_deg_, msg->linear.x);

            // Add PID angular velocity correction to output Twist
            out_twist->angular.z = out.correction;
        }

        cmd_vel_pub_->publish(std::move(out_twist));
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    bno055lib::HeadingController controller_;
    rclcpp::Time last_time_;
    double current_heading_deg_;
    double gyro_z_deg_;
    double target_heading_deg_{0.0};
    bool target_heading_locked_;
    bool has_imu_data_;
};

}  // namespace bno055_ros2

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bno055_ros2::BNO055HeadingControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
