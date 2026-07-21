use libbno055::{BNO055, OpMode, Quaternion};

fn main() -> Result<(), &'static str> {
    println!("Initializing BNO055 via Rust crate...");

    let mut imu = BNO055::new_i2c(0x28, "/dev/i2c-1")?;
    if imu.begin(OpMode::NDOF) {
        println!("BNO055 Initialized Successfully!");

        let q = Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };
        let euler = BNO055::to_euler_degrees(&q);
        println!("Roll: {}, Pitch: {}, Yaw: {}", euler.x, euler.y, euler.z);
    } else {
        println!("Initialization failed (Device missing or unreadable).");
    }

    Ok(())
}
