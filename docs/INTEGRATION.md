# Advanced ROS 2 Integration Guide

For enterprise-grade robotics, simply dropping a sensor read into a `rclcpp::TimerBase` is insufficient. Hardware sensors can disconnect, drivers can crash, and I2C buses can lock up.

This guide demonstrates how to integrate `libbno055-linux` using **ROS 2 Lifecycle Nodes (Managed Nodes)** and strict **Quality of Service (QoS)** profiles to build a truly fault-tolerant IMU driver.

---

## 1. The Lifecycle Node (State Machine) Approach

ROS 2 Lifecycle Nodes enforce a strict state machine (`Unconfigured` ➔ `Inactive` ➔ `Active`). Because the BNO055 requires an explicit initialization sequence over I2C, mapping its hardware states directly to the ROS 2 lifecycle states ensures predictable system startup.

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <libbno055-linux/bno055.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BNO055DriverNode : public rclcpp_lifecycle::LifecycleNode {
public:
    BNO055DriverNode() : LifecycleNode("bno055_driver") {}

    // ---------------------------------------------------------
    // 1. CONFIGURE: Allocate resources and test I2C connection
    // ---------------------------------------------------------
    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        RCLCPP_INFO(get_logger(), "Configuring BNO055 Hardware...");
        
        imu_ = std::make_unique<bno055lib::BNO055>(0x28, "/dev/i2c-1");
        
        // Attempt hardware boot into NDOF fusion mode
        if (!imu_->begin(bno055lib::OpMode::NDOF)) {
            RCLCPP_ERROR(get_logger(), "CRITICAL: I2C connection failed. Sensor missing?");
            return CallbackReturn::FAILURE;
        }

        // Load deterministic calibration
        imu_->loadCalibrationFile("/etc/robot_config/bno055_calib.bin");

        // Setup publisher with SENSOR_DATA QoS (Best Effort)
        auto qos = rclcpp::QoS(rclcpp::SensorDataQoS());
        pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", qos);
        
        return CallbackReturn::SUCCESS;
    }

    // ---------------------------------------------------------
    // 2. ACTIVATE: Start real-time data streaming
    // ---------------------------------------------------------
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        pub_->on_activate();
        
        // Start 100Hz high-priority polling timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&BNO055DriverNode::publish_imu_data, this)
        );
        
        RCLCPP_INFO(get_logger(), "IMU Stream Activated.");
        return CallbackReturn::SUCCESS;
    }

    // ---------------------------------------------------------
    // 3. DEACTIVATE: Pause streaming (hardware remains configured)
    // ---------------------------------------------------------
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        pub_->on_deactivate();
        timer_->cancel();
        RCLCPP_INFO(get_logger(), "IMU Stream Paused.");
        return CallbackReturn::SUCCESS;
    }

    // ---------------------------------------------------------
    // 4. CLEANUP: Fully release I2C file descriptors
    // ---------------------------------------------------------
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        timer_.reset();
        pub_.reset();
        imu_.reset(); // RAII safely closes /dev/i2c-*
        RCLCPP_INFO(get_logger(), "Hardware released.");
        return CallbackReturn::SUCCESS;
    }

private:
    void publish_imu_data() {
        // Strict exception-free path for deterministic timing
        auto quat = imu_->getQuaternionNoexcept();
        auto gyro = imu_->getGyroscopeNoexcept();
        auto accel = imu_->getLinearAccelerationNoexcept();

        if (quat && gyro && accel) {
            auto msg = sensor_msgs::msg::Imu();
            msg.header.stamp = this->now();
            msg.header.frame_id = "imu_link";
            
            msg.orientation.w = quat->w;
            msg.orientation.x = quat->x;
            msg.orientation.y = quat->y;
            msg.orientation.z = quat->z;
            
            msg.angular_velocity.x = gyro->x;
            msg.angular_velocity.y = gyro->y;
            msg.angular_velocity.z = gyro->z;
            
            msg.linear_acceleration.x = accel->x;
            msg.linear_acceleration.y = accel->y;
            msg.linear_acceleration.z = accel->z;
            
            pub_->publish(msg);
        } else {
            // Log telemtry to diagnostics system
            auto diag = imu_->getDiagnostics();
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, 
                "I2C Dropout Detected. Reconnect attempts: %d", diag.reconnect_attempts);
        }
    }

    std::unique_ptr<bno055lib::BNO055> imu_;
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

---

## 2. SensorData QoS (Quality of Service)

Notice the use of `rclcpp::SensorDataQoS()` in the example above. 
For high-frequency sensor streams (like IMUs running at 100Hz), using the default ROS 2 QoS (`RELIABLE`) is an anti-pattern. If the network experiences temporary packet loss (e.g., over Wi-Fi to a ground station), `RELIABLE` will buffer and attempt to retransmit old IMU data, leading to massive latency spikes in your Kalman Filters.

`SensorDataQoS` defaults to `BEST_EFFORT`. It guarantees that your robot's state estimator will only ever process the *newest, most up-to-date* orientation, instantly dropping stale packets.

---

## 3. Registering with rosdep

To make your package easily buildable by others, declare this library as a system dependency in your `package.xml`:
```xml
<depend>libbno055-linux</depend>
```

When users run `rosdep install --from-paths src -y`, `rosdep` will automatically fetch and install this library via `vcpkg`, `apt`, or source, depending on how you distribute your final software stack.
