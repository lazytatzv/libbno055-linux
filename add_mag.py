import os

# 1. Update bno055_ros2_common.hpp
with open('src/ros2/bno055_ros2_common.hpp', 'r') as f:
    hpp = f.read()

hpp = hpp.replace('#include <sensor_msgs/msg/imu.hpp>', '#include <sensor_msgs/msg/imu.hpp>\n#include <sensor_msgs/msg/magnetic_field.hpp>')

declare_old = '    node->template declare_parameter<bool>("use_external_crystal", true);\n}'
declare_new = '    node->template declare_parameter<bool>("use_external_crystal", true);\n    node->template declare_parameter<std::vector<double>>("magnetic_field_covariance", std::vector<double>(9, 0.0));\n}'
hpp = hpp.replace(declare_old, declare_new)

cov_old = '}  // namespace bno055_ros2'
cov_new = """
template <typename T>
inline void fill_mag_covariance(T* node, sensor_msgs::msg::MagneticField& message) {
    auto mag_cov = node->template get_parameter("magnetic_field_covariance").as_double_array();
    if (mag_cov.size() == 9) std::copy(mag_cov.begin(), mag_cov.end(), message.magnetic_field_covariance.begin());
}

}  // namespace bno055_ros2"""
hpp = hpp.replace(cov_old, cov_new)

with open('src/ros2/bno055_ros2_common.hpp', 'w') as f:
    f.write(hpp)

# 2. Update Publishers
def update_node(file_path, is_perf=False):
    with open(file_path, 'r') as f:
        cpp = f.read()
    
    # Add header
    cpp = cpp.replace('#include <sensor_msgs/msg/imu.hpp>', '#include <sensor_msgs/msg/imu.hpp>\n#include <sensor_msgs/msg/magnetic_field.hpp>')

    # Add publisher definition
    cpp = cpp.replace('rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;', 'rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;\n    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_publisher_;')
    
    # Add publisher instantiation
    pub_init_old = 'publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);'
    pub_init_new = pub_init_old + '\n        mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", qos);'
    cpp = cpp.replace(pub_init_old, pub_init_new)

    # Note for lifecycle node:
    pub_init_old_lc = 'publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);'
    if pub_init_old_lc in cpp:
        pub_init_new_lc = pub_init_old_lc + '\n        mag_publisher_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);'
        cpp = cpp.replace(pub_init_old_lc, pub_init_new_lc)

    # In timer_callback
    get_accel_old = 'auto accel = imu_->getLinearAccelerationNoexcept();'
    get_accel_new = get_accel_old + '\n        auto mag = imu_->getMagnetometerNoexcept();'
    cpp = cpp.replace(get_accel_old, get_accel_new)

    if_old = 'if (!quat || !gyro || !accel) {'
    if_new = 'if (!quat || !gyro || !accel || !mag) {'
    cpp = cpp.replace(if_old, if_new)

    # Publish msg
    if is_perf:
        pub_old = 'publisher_->publish(std::move(message));\n    }'
        pub_new = """publisher_->publish(std::move(message));

        // Magnetic Field Zero-Copy
        auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();
        mag_msg->header.stamp = stamp;
        mag_msg->header.frame_id = frame_id_;
        mag_msg->magnetic_field.x = mag->x * 1e-6; // Convert uT to Tesla
        mag_msg->magnetic_field.y = mag->y * 1e-6;
        mag_msg->magnetic_field.z = mag->z * 1e-6;
        bno055_ros2::fill_mag_covariance(this, *mag_msg);
        mag_publisher_->publish(std::move(mag_msg));
    }"""
    else:
        pub_old = 'publisher_->publish(std::move(message));\n    }'
        pub_new = """publisher_->publish(std::move(message));

        // Magnetic Field
        auto mag_msg = std::make_unique<sensor_msgs::msg::MagneticField>();
        mag_msg->header.stamp = stamp;
        mag_msg->header.frame_id = frame_id_;
        mag_msg->magnetic_field.x = mag->x * 1e-6; // Convert uT to Tesla
        mag_msg->magnetic_field.y = mag->y * 1e-6;
        mag_msg->magnetic_field.z = mag->z * 1e-6;
        bno055_ros2::fill_mag_covariance(this, *mag_msg);
        mag_publisher_->publish(std::move(mag_msg));
    }"""
    
    cpp = cpp.replace(pub_old, pub_new)
    
    with open(file_path, 'w') as f:
        f.write(cpp)

update_node('src/ros2/ros2_publisher_node.cpp', False)
update_node('src/ros2/ros2_lifecycle_publisher_node.cpp', False)
update_node('src/ros2/ros2_perf_publisher_node.cpp', True)

# 3. Update yaml
with open('config/bno055_params.yaml', 'r') as f:
    yaml = f.read()

yaml = yaml.replace('linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]', 'linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n    magnetic_field_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]')

with open('config/bno055_params.yaml', 'w') as f:
    f.write(yaml)

