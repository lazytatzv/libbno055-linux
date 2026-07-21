use std::ffi::CString;

pub mod sys {
    use std::os::raw::{c_char, c_void};

    #[repr(C)]
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct bno055_vector3_t {
        pub x: f32,
        pub y: f32,
        pub z: f32,
    }

    #[repr(C)]
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct bno055_quaternion_t {
        pub w: f32,
        pub x: f32,
        pub y: f32,
        pub z: f32,
    }

    #[repr(C)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct bno055_calibration_status_t {
        pub sys: u8,
        pub gyro: u8,
        pub accel: u8,
        pub mag: u8,
    }

    #[repr(C)]
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub struct bno055_diagnostics_t {
        pub write_failures: u32,
        pub read_failures: u32,
        pub reconnect_attempts: u32,
    }

    #[repr(C)]
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub struct bno055_raw_sensor_data_t {
        pub accel: bno055_vector3_t,
        pub mag: bno055_vector3_t,
        pub gyro: bno055_vector3_t,
    }

    #[allow(non_camel_case_types)]
    pub type bno055_handle_t = *mut c_void;

    extern "C" {
        pub fn bno055_create_i2c(address: u8, device: *const c_char) -> bno055_handle_t;
        pub fn bno055_create_uart(port: *const c_char, baudrate: u32) -> bno055_handle_t;
        pub fn bno055_destroy(handle: bno055_handle_t);

        pub fn bno055_begin(handle: bno055_handle_t, mode: u8) -> bool;
        pub fn bno055_reset(handle: bno055_handle_t) -> bool;

        pub fn bno055_set_mode(handle: bno055_handle_t, mode: u8);
        pub fn bno055_get_mode(handle: bno055_handle_t) -> u8;
        pub fn bno055_set_ext_crystal_use(handle: bno055_handle_t, use_xtal: bool);

        pub fn bno055_get_accelerometer(handle: bno055_handle_t, out_accel: *mut bno055_vector3_t) -> bool;
        pub fn bno055_get_magnetometer(handle: bno055_handle_t, out_mag: *mut bno055_vector3_t) -> bool;
        pub fn bno055_get_gyroscope(handle: bno055_handle_t, out_gyro: *mut bno055_vector3_t) -> bool;
        pub fn bno055_get_euler_angles(handle: bno055_handle_t, out_euler: *mut bno055_vector3_t) -> bool;
        pub fn bno055_get_linear_acceleration(handle: bno055_handle_t, out_accel: *mut bno055_vector3_t) -> bool;
        pub fn bno055_get_gravity(handle: bno055_handle_t, out_gravity: *mut bno055_vector3_t) -> bool;
        pub fn bno055_get_quaternion(handle: bno055_handle_t, out_quat: *mut bno055_quaternion_t) -> bool;
        pub fn bno055_get_temperature(handle: bno055_handle_t, out_temp: *mut i8) -> bool;
        pub fn bno055_get_raw_sensor_data(handle: bno055_handle_t, out_raw: *mut bno055_raw_sensor_data_t) -> bool;

        pub fn bno055_get_calibration_status(handle: bno055_handle_t, out_status: *mut bno055_calibration_status_t) -> bool;
        pub fn bno055_is_fully_calibrated(handle: bno055_handle_t) -> bool;
        pub fn bno055_get_diagnostics(handle: bno055_handle_t, out_diag: *mut bno055_diagnostics_t) -> bool;

        pub fn bno055_save_calibration_file(handle: bno055_handle_t, filepath: *const c_char) -> bool;
        pub fn bno055_load_calibration_file(handle: bno055_handle_t, filepath: *const c_char) -> bool;
        pub fn bno055_enable_auto_calibration(handle: bno055_handle_t, filepath: *const c_char);
        pub fn bno055_disable_auto_calibration(handle: bno055_handle_t);

        pub fn bno055_enter_suspend_mode(handle: bno055_handle_t);
        pub fn bno055_enter_normal_mode(handle: bno055_handle_t);

        pub fn bno055_to_euler_degrees(q: *const bno055_quaternion_t) -> bno055_vector3_t;
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vector3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CalibrationStatus {
    pub sys: u8,
    pub gyro: u8,
    pub accel: u8,
    pub mag: u8,
}

impl CalibrationStatus {
    pub fn is_fully_calibrated(&self) -> bool {
        self.sys == 3 && self.gyro == 3 && self.accel == 3 && self.mag == 3
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Diagnostics {
    pub write_failures: u32,
    pub read_failures: u32,
    pub reconnect_attempts: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OpMode {
    Config = 0x00,
    AccOnly = 0x01,
    MagOnly = 0x02,
    GyroOnly = 0x03,
    AccMag = 0x04,
    AccGyro = 0x05,
    MagGyro = 0x06,
    AMG = 0x07,
    IMUPlus = 0x08,
    Compass = 0x09,
    M4G = 0x0A,
    NDOFFmcOff = 0x0B,
    NDOF = 0x0C,
}

pub struct BNO055 {
    handle: sys::bno055_handle_t,
}

unsafe impl Send for BNO055 {}
unsafe impl Sync for BNO055 {}

impl BNO055 {
    pub fn new_i2c(address: u8, device: &str) -> Result<Self, &'static str> {
        let dev_c = CString::new(device).map_err(|_| "Invalid device string")?;
        let handle = unsafe { sys::bno055_create_i2c(address, dev_c.as_ptr()) };
        if handle.is_null() {
            Err("Failed to create BNO055 handle")
        } else {
            Ok(Self { handle })
        }
    }

    pub fn new_uart(port: &str, baudrate: u32) -> Result<Self, &'static str> {
        let port_c = CString::new(port).map_err(|_| "Invalid port string")?;
        let handle = unsafe { sys::bno055_create_uart(port_c.as_ptr(), baudrate) };
        if handle.is_null() {
            Err("Failed to create BNO055 handle")
        } else {
            Ok(Self { handle })
        }
    }

    pub fn begin(&mut self, mode: OpMode) -> bool {
        unsafe { sys::bno055_begin(self.handle, mode as u8) }
    }

    pub fn reset(&mut self) -> bool {
        unsafe { sys::bno055_reset(self.handle) }
    }

    pub fn set_mode(&mut self, mode: OpMode) {
        unsafe { sys::bno055_set_mode(self.handle, mode as u8) }
    }

    pub fn get_mode(&self) -> OpMode {
        let raw = unsafe { sys::bno055_get_mode(self.handle) };
        match raw {
            0x01 => OpMode::AccOnly,
            0x02 => OpMode::MagOnly,
            0x03 => OpMode::GyroOnly,
            0x04 => OpMode::AccMag,
            0x05 => OpMode::AccGyro,
            0x06 => OpMode::MagGyro,
            0x07 => OpMode::AMG,
            0x08 => OpMode::IMUPlus,
            0x09 => OpMode::Compass,
            0x0A => OpMode::M4G,
            0x0B => OpMode::NDOFFmcOff,
            0x0C => OpMode::NDOF,
            _ => OpMode::Config,
        }
    }

    pub fn get_quaternion(&self) -> Option<Quaternion> {
        let mut raw = sys::bno055_quaternion_t { w: 0.0, x: 0.0, y: 0.0, z: 0.0 };
        if unsafe { sys::bno055_get_quaternion(self.handle, &mut raw) } {
            Some(Quaternion { w: raw.w, x: raw.x, y: raw.y, z: raw.z })
        } else {
            None
        }
    }

    pub fn get_accelerometer(&self) -> Option<Vector3> {
        let mut raw = sys::bno055_vector3_t { x: 0.0, y: 0.0, z: 0.0 };
        if unsafe { sys::bno055_get_accelerometer(self.handle, &mut raw) } {
            Some(Vector3 { x: raw.x, y: raw.y, z: raw.z })
        } else {
            None
        }
    }

    pub fn get_gyroscope(&self) -> Option<Vector3> {
        let mut raw = sys::bno055_vector3_t { x: 0.0, y: 0.0, z: 0.0 };
        if unsafe { sys::bno055_get_gyroscope(self.handle, &mut raw) } {
            Some(Vector3 { x: raw.x, y: raw.y, z: raw.z })
        } else {
            None
        }
    }

    pub fn get_magnetometer(&self) -> Option<Vector3> {
        let mut raw = sys::bno055_vector3_t { x: 0.0, y: 0.0, z: 0.0 };
        if unsafe { sys::bno055_get_magnetometer(self.handle, &mut raw) } {
            Some(Vector3 { x: raw.x, y: raw.y, z: raw.z })
        } else {
            None
        }
    }

    pub fn get_euler_angles(&self) -> Option<Vector3> {
        let mut raw = sys::bno055_vector3_t { x: 0.0, y: 0.0, z: 0.0 };
        if unsafe { sys::bno055_get_euler_angles(self.handle, &mut raw) } {
            Some(Vector3 { x: raw.x, y: raw.y, z: raw.z })
        } else {
            None
        }
    }

    pub fn get_calibration_status(&self) -> Option<CalibrationStatus> {
        let mut raw = sys::bno055_calibration_status_t { sys: 0, gyro: 0, accel: 0, mag: 0 };
        if unsafe { sys::bno055_get_calibration_status(self.handle, &mut raw) } {
            Some(CalibrationStatus {
                sys: raw.sys,
                gyro: raw.gyro,
                accel: raw.accel,
                mag: raw.mag,
            })
        } else {
            None
        }
    }

    pub fn get_diagnostics(&self) -> Diagnostics {
        let mut raw = sys::bno055_diagnostics_t { write_failures: 0, read_failures: 0, reconnect_attempts: 0 };
        unsafe { sys::bno055_get_diagnostics(self.handle, &mut raw) };
        Diagnostics {
            write_failures: raw.write_failures,
            read_failures: raw.read_failures,
            reconnect_attempts: raw.reconnect_attempts,
        }
    }

    pub fn to_euler_degrees(q: &Quaternion) -> Vector3 {
        let sys_q = sys::bno055_quaternion_t { w: q.w, x: q.x, y: q.y, z: q.z };
        let res = unsafe { sys::bno055_to_euler_degrees(&sys_q) };
        Vector3 { x: res.x, y: res.y, z: res.z }
    }
}

impl Drop for BNO055 {
    fn drop(&mut self) {
        if !self.handle.is_null() {
            unsafe { sys::bno055_destroy(self.handle) };
        }
    }
}



#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rust_bindings_creation_and_math() {
        let _imu = BNO055::new_i2c(0x28, "mock_device").expect("Creation failed");
        let q = Quaternion { w: 1.0, x: 0.0, y: 0.0, z: 0.0 };
        let euler = BNO055::to_euler_degrees(&q);
        assert_eq!(euler.x, 0.0);
        assert_eq!(euler.y, 0.0);
        assert_eq!(euler.z, 0.0);
    }
}
