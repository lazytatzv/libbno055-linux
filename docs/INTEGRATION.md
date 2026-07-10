# ROS 2 Integration Best Practices

While you can use `libbno055-linux` in standard ROS 2 `rclcpp::Node` instances, for production robotics, we highly recommend utilizing **ROS 2 Lifecycle Nodes (Managed Nodes)**.

## Lifecycle Node Integration Example

Because the BNO055 requires initialization and calibration over I2C (which can fail if the sensor is disconnected or if the I2C bus is not ready), wrapping it in a Lifecycle Node ensures your robot's state machine handles sensor failures gracefully and predictably.

```cpp
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "libbno055-linux/bno055.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class BNO055Node : public rclcpp_lifecycle::LifecycleNode {
public:
    BNO055Node() : LifecycleNode("bno055_node") {}

    CallbackReturn on_configure(const rclcpp_lifecycle::State &) override {
        imu_ = std::make_unique<bno055lib::BNO055>(0x28, "/dev/i2c-1");
        
        // Attempt to initialize the hardware
        if (!imu_->begin(bno055lib::OpMode::IMUPlus)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize BNO055 over I2C");
            return CallbackReturn::FAILURE;
        }

        // Load pre-calibrated offsets for deterministic startup
        imu_->loadCalibrationFile("/etc/robot/bno055_calib.bin");
        RCLCPP_INFO(this->get_logger(), "BNO055 Configured Successfully.");
        
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override {
        // Start publishing timers and data streams
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override {
        // Stop publishing timers
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override {
        // Safely destroy the instance and strictly release I2C file descriptors
        imu_.reset();
        return CallbackReturn::SUCCESS;
    }

private:
    std::unique_ptr<bno055lib::BNO055> imu_;
};
```

## Integrating with rosdep (System Dependencies)
When distributing your ROS 2 package, you can declare `libbno055-linux` as a system dependency in your `package.xml`:
```xml
<depend>libbno055-linux</depend>
```
If you are distributing via source, users can use `vcpkg` or `FetchContent` to satisfy this dependency automatically until the library is fully pushed to the ROS Index.
