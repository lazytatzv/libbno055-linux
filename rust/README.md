# libbno055

[![crates.io](https://img.shields.io/crates/v/libbno055.svg)](https://crates.io/crates/libbno055)
[![Docs](https://img.shields.io/badge/docs-GitHub%20Pages-blue.svg)](https://lazytatzv.github.io/libbno055-linux/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/lazytatzv/libbno055-linux/blob/main/LICENSE)

Safe, idiomatic Rust bindings for the [`libbno055-linux`](https://github.com/lazytatzv/libbno055-linux) C++17 Bosch BNO055 9-DOF IMU driver on Linux systems.

---

## ⚡ Highlights

- **Safe Public API**: Zero `unsafe` required by application code. Memory and file descriptors are cleaned up automatically via `Drop` RAII.
- **`Send + Sync` Thread Safety**: Implements `Send` and `Sync` for multi-threaded robotics control loops and async runtimes.
- **Idiomatic Rust Types**: Uses `Option<Vector3>`, `Option<Quaternion>`, and `Result<T, E>` for zero-cost error handling.
- **Automated Build Integration**: Built-in `build.rs` compiles the underlying C++ engine seamlessly during `cargo build`.

---

## 🚀 Quick Start

### 1. Add Dependency

```bash
cargo add libbno055
```

Or add to your `Cargo.toml`:
```toml
[dependencies]
libbno055 = "1.5.4"
```

### 2. Basic Example

```rust
use libbno055::{BNO055, OpMode};

fn main() -> Result<(), &'static str> {
    // Initialize IMU via I2C (Address 0x28, /dev/i2c-1)
    let mut imu = BNO055::new_i2c(0x28, "/dev/i2c-1")?;

    // Boot sensor in NDOF 9-DOF Fusion mode
    if imu.begin(OpMode::NDOF) {
        if let Some(q) = imu.get_quaternion() {
            let euler = BNO055::to_euler_degrees(&q);
            println!("Roll: {:.2}, Pitch: {:.2}, Yaw: {:.2}", euler.x, euler.y, euler.z);
        }
    }

    Ok(())
}
```

---

## 📚 Documentation

* 📖 **[API Reference](https://github.com/lazytatzv/libbno055-linux/blob/main/docs/API_REFERENCE.md)**
* 🛠️ **[Integration & Tuning Guide](https://github.com/lazytatzv/libbno055-linux/blob/main/docs/INTEGRATION.md#10-rust-integration-guide-use-libbno055)**
* 🏗️ **[Architecture & Design](https://github.com/lazytatzv/libbno055-linux/blob/main/docs/ARCHITECTURE.md#9-multi-language-binding-architecture)**

---

## 📄 License

This project is licensed under the [MIT License](https://github.com/lazytatzv/libbno055-linux/blob/main/LICENSE).
