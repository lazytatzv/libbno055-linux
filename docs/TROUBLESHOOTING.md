# Troubleshooting & FAQ

## I2C Permission Denied
**Error**: `Failed to open I2C device: /dev/i2c-1 (Permission denied)`
**Cause**: The user running the binary does not have access to the hardware I2C bus.
**Solution**:
Add your user to the `i2c` group:
```bash
sudo usermod -aG i2c $USER
```
Then log out and log back in.

## BNO055 Clock Stretching (I2C Lockups on Raspberry Pi)
**Error**: `[Warning] Temporary communication dropout` appears frequently.
**Cause**: The Broadcom I2C hardware on Raspberry Pi has a known bug handling the BNO055's clock stretching, causing the kernel driver to timeout.
**Solution**:
1. Lower the I2C baudrate. Edit `/boot/firmware/config.txt` (or `/boot/config.txt`) and set:
   ```text
   dtparam=i2c_arm=on,i2c_arm_baudrate=10000
   ```
   (The BNO055 supports up to 400kHz, but 10kHz to 50kHz is significantly more stable on the Raspberry Pi).
2. Reboot the Pi.

## Sensor Calibration is Lost on Reboot
**Error**: The heading (Yaw) drifts immediately after power-on.
**Cause**: The BNO055 does not have internal non-volatile memory for calibration profiles. It loses its calibration every time it loses power.
**Solution**:
Use the provided `calibrate_imu` example to generate a `bno055_calib.bin` file, then load it in your code immediately after `begin()`:
```cpp
imu.loadCalibrationFile("bno055_calib.bin");
```

## Why isn't the Magnetometer working indoors?
**Error**: Calibration status for `Mag` is always 0 or 1, and heading is highly erratic.
**Cause**: Indoor environments with heavy metal structures, motors, and power lines cause severe magnetic distortion.
**Solution**:
Do not use `NDOF` mode indoors. Switch to `IMUPlus` mode:
```cpp
imu.begin(bno055lib::OpMode::IMUPlus);
```
`IMUPlus` mode ignores the magnetometer and provides a relative heading using only the high-precision gyroscope and accelerometer.
