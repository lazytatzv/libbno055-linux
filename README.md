# bno055lib

A robust, thread-safe, and dependency-free C++17 library for the Bosch BNO055 orientation sensor over I2C on Linux systems.

BNO055 orientation sensorをLinux上のI2C経由で制御するための、堅牢でスレッドセーフかつ外部依存性のないC++17ライブラリです。

---

## Features (特徴)

- **Pure C++17 & CMake**: No ROS or external dependencies. Highly portable for any C++ project on Linux.
  - ROSなどの外部依存関係が一切ありません。Linux上のあらゆるC++プロジェクトで動作します。
- **Pimpl Idiom (API Cleanliness)**: System headers like `<linux/i2c-dev.h>` or internal register mappings are hidden from the public header, resulting in fast compilation and absolute API cleanliness.
  - 公開ヘッダーからLinux依存のヘッダーやレジスタ構造を完全に排除しているため、ビルドがクリーンかつ高速です。
- **Thread-Safety (スレッドセーフ)**: Fully thread-safe operations protected by `std::mutex`. Safe to query sensor data from multiple threads.
  - 全てのI2Cアクセスは `std::mutex` で保護されており、複数スレッドからの同時呼び出しも安全です。
- **Robust Communication & Auto-Reconnection (自動再接続)**: Automatically handles temporary I2C glitches by retrying operations. In case of major bus failures or disconnections, it automatically tries to close/re-open and reinitialize the sensor seamlessly.
  - 一時的なI2Cノイズに対するリトライ処理に加え、バスの切断が検知された場合は自動的に再オープンと再初期化を試みます。
- **Calibration Save/Restore (キャリブレーション保存と復元)**: Easily save the 22-byte calibration profiles into a binary file and restore it instantly on startup.
  - キャリブレーションデータを22バイトのバイナリファイルとして保存・復元する機能を提供します。
- **SI Unit Standardization (SI単位系でのデータ提供)**: Raw sensor values are automatically converted into standard physical SI units (`m/s^2` for accel, `rad/s` for gyro, `rad` for euler, `uT` for mag).
  - 取得した値は自動的に標準物理単位（加速度: $m/s^2$, 角速度: $rad/s$, オイラー角: $rad$, 磁場: $\mu T$）に変換されて返されます。
- **Custom Logger Hook (カスタムロガーの登録)**: Easily pipe log outputs into your custom logger (e.g., `ROS_INFO`, syslog, or Google Log).
  - ログ出力をROS 2のロガーや独自のロギングシステムにコールバック経由で転送できます。

---

## Directory Structure (ディレクトリ構造)

```
bno055lib/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── cmake/
│   └── bno055libConfig.cmake.in
├── include/
│   └── bno055lib/
│       └── bno055.hpp (Clean Public Header)
├── src/
│   └── bno055.cpp (Implementation containing Pimpl)
└── examples/
    └── calibrate.cpp (Calibration Utility Tool)
```

---

## Build & Install (ビルドとインストール)

### Build Library (ライブラリのビルド)
```bash
git clone git@github.com:lazytatzv/bno055lib.git
cd bno055lib
mkdir build && cd build
cmake ..
make
```

### Install Library (インストール)
```bash
sudo make install
```
This installs the library targets and config files to standard locations, allowing other CMake projects to find it easily via `find_package(bno055lib REQUIRED)`.

---

## Usage Example (使用例)

```cpp
#include <bno055lib/bno055.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    // Initialize sensor on address 0x28 (default) using I2C device "/dev/i2c-1"
    bno055lib::BNO055 imu(0x28, "/dev/i2c-1");

    // Optional: Setup custom logger callback (e.g., forwarding to your custom logger)
    imu.setLogger([](bno055lib::LogLevel level, std::string_view message) {
        std::cout << "[IMU LOG] " << message << std::endl;
    });

    // Begin in NDOF mode (sensor fusion)
    if (!imu.begin(bno055lib::OpMode::NDOF)) {
        std::cerr << "Failed to initialize BNO055!" << std::endl;
        return 1;
    }

    // Load calibration file if it exists
    imu.loadCalibrationFile("bno055_calib.bin");

    while (true) {
        try {
            // Get standard SI values
            auto accel = imu.getLinearAcceleration(); // m/s^2
            auto gyro = imu.getGyroscope();           // rad/s
            auto quat = imu.getQuaternion();          // w, x, y, z

            std::cout << "Accel: [" << accel.x << ", " << accel.y << ", " << accel.z << "] m/s^2" << std::endl;
            std::cout << "Gyro:  [" << gyro.x << ", " << gyro.y << ", " << gyro.z << "] rad/s" << std::endl;
            std::cout << "Quat:  [" << quat.w << ", " << quat.x << ", " << quat.y << ", " << quat.z << "]" << std::endl;
        } catch (const bno055lib::IMUError& e) {
            std::cerr << "Sensor error: " << e.what() << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return 0;
}
```

---

## Calibration Utility (キャリブレーションユーティリティ)

A utility tool to calibrate BNO055 and save its profile is compiled in `build/calibrate_imu`.
`build/calibrate_imu` にキャリブレーション及びプロファイル保存用のツールがビルドされます。

Run it by specifying the I2C device and output filename:
```bash
./calibrate_imu /dev/i2c-1 bno055_calib.bin
```

---

## License (ライセンス)

MIT License. Feel free to use in commercial and non-commercial projects.
