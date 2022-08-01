//! # Adafruit NXP sensor driver
//! 
//! [![made-with-rust](https://img.shields.io/badge/Made%20with-Rust-1f425f?style=plastic)](https://www.rust-lang.org/)
//! [![powered-by-sii](https://img.shields.io/badge/Powered%20By-SII-blue?style=plastic)](https://sii-group.com/fr-FR/sii-sud-ouest)
//! 
//! This is a driver for the [Adafruit Precision NXP 9-DOF Breakout Board - FXOS8700 + FXAS21002C](https://www.adafruit.com/product/3463).
//! It is based on the libraries provided by Adafruit in C++ for the [FXAS21002C](https://github.com/adafruit/Adafruit_FXAS21002C)
//! and the [FXOS8700](https://github.com/adafruit/Adafruit_FXOS8700).
//! 
//! `embedded_hal` based driver with i2c access to the Adafruit chip.
//! It provides a basic interface to get raw and scaled data from the sensors.
//! All 3 sensors are separated in their own structure, respectively `Accelerometer`, `Magnetometer` and `Gyroscope`. 
//! 
//! ### Calibration
//! You might need to calibrate and provide offset for the accelerometer and the gyroscope 
//! -> check `accel_gyro_calibration.rs` example code or the function `calibration()`.
//! For the magnetometer, you might need to compute hard and soft iron correction. 
//! If you want to learn more about those correction you can go [here](https://www.fierceelectronics.com/components/compensating-for-tilt-hard-iron-and-soft-iron-effects).
//! We won't provide any tool for those corrections.
//! However, you can follow this [tutorial](https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration) provided by Adafruit.
//! 
//! ### More information
//! This crate allows for a full customization except for Fifo and hardware interruptions implementation.
//! This is not plan in the near future.
//! 
//! To use this driver you must provide a concrete `embedded_hal` implementation both for Delay and I2C.
//! 
//! You must be able to compute a delta time. e.g. the execution time of each loop. 
//! Most examples uses a Raspberry Pi for practicals reasons.
//! NB: The more accurate the delta time, the more accurate the measurement.
//! 
//! A better ROS2 example might be provided in the future.
//! 
//! ### Usage
//! The example below uses [rppal](https://crates.io/crates/rppal) crates and runs on a Raspberry Pi.
//! ```no_run
//! use rppal::hal::Delay;
//! use rppal::i2c::I2c;
//! 
//! use embedded_hal::blocking::delay::*;
//! use nalgebra::{Vector3, Matrix3};
//! 
//! use adafruit::*;
//! 
//! fn main() -> -> Result<(), SensorError<rppal::i2c::Error>> {
//! 
//!     // Init a delay used in certain functions and between each loop.
//!     let mut delay = Delay::new();
//! 
//!     // Setup the raspberry's I2C interface to create the sensor.
//!     let i2c = I2c::new().unwrap();
//! 
//!     // Create an Adafruit object.
//!     let mut sensor = AdafruitNXP::new(0x8700A, 0x8700B, 0x0021002C, i2c);
//! 
//!     // Check if the sensor is ready to go.
//!     let ready = sensor.begin()?;
//!     if !ready {
//!         std::eprintln!("Sensor not detected, check your wiring!");
//!         std::process::exit(1);
//!     }
//!     
//!     // If you don't want to use the default configuration.
//!     sensor.set_accel_range(config::AccelMagRange::Range4g)?;
//!     sensor.set_gyro_range(config::GyroRange::Range500dps)?;
//!     sensor.set_accelmag_output_data_rate(config::AccelMagODR::ODR200HZ)?;
//!     sensor.set_gyro_output_data_rate(config::GyroODR::ODR200HZ)?;
//! 
//!     // If you need more precise results you might consider adding offsets. (Values are examples)
//!     sensor.accel_sensor.set_offset(-0.0526, -0.5145, 0.1439);
//!     sensor.gyro_sensor.set_offset(0.0046, 0.0014, 0.0003);
//!     let hard_iron = Vector3::new(-3.87, -35.45, -50.32);
//!     let soft_iron = Matrix3::new(1.002, -0.012, 0.002,
//!                                  -0.012, 0.974, -0.010,
//!                                  0.002, -0.010, 1.025);
//!     sensor.mag_sensor.set_hard_soft_iron(hard_iron, soft_iron);
//! 
//!     loop {
//! 
//!         sensor.read_data();
//! 
//!         let acc_x = sensor.accel_sensor.get_scaled_x();
//!         let acc_y = sensor.accel_sensor.get_scaled_y();
//!         let acc_z = sensor.accel_sensor.get_scaled_z();
//! 
//!         let gyro_x = sensor.gyro_sensor.get_scaled_x();
//!         let gyro_y = sensor.gyro_sensor.get_scaled_y();
//!         let gyro_z = sensor.gyro_sensor.get_scaled_z();
//! 
//!         let mag_x = sensor.mag_sensor.get_scaled_x();
//!         let mag_y = sensor.mag_sensor.get_scaled_y();
//!         let mag_z = sensor.mag_sensor.get_scaled_z();
//! 
//!         // Print data
//!         std::println!("###############################################");
//!         std::println!("Accel X: {}", acc_x);
//!         std::print!(" Accel Y: {}", acc_y);
//!         std::print!(" Accel Z: {}", acc_z);
//!         std::println!("-----------------------------------------------");
//!         std::println!("Gyro X: {}", gyro_x);
//!         std::print!(" Gyro Y: {}", gyro_y);
//!         std::print!(" Gyro Z: {}", gyro_z);
//!         std::println!("-----------------------------------------------");
//!         std::print!("Mag X: {}", mag_x);
//!         std::print!(" Mag Y: {}", mag_y);
//!         std::print!(" Mag Z: {}", mag_z);
//! 
//!         delay.delay_ms(5_u8);
//!     }
//! }
//! ```

#![no_std]
#![deny(missing_docs)]

pub mod config;
pub mod sensor;

use crate::config::*;
use crate::sensor::{Sensor, SensorType};
use crate::config::{ACCEL_MAG_ADDR, GYRO_ADDR};
use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Write, WriteRead},
};
use nalgebra::{Vector3, Matrix3};

/// Gravity constant.
pub const G: f32 = 9.80665;
/// PI. 3.14159274_f32
pub const PI: f32 = core::f32::consts::PI;
/// Constant to convert radians to degrees.
pub const RAD2DEG: f32 = 180.0 / PI;
/// Constant to convert degrees to radians.
pub const DEG2RAD: f32 = PI / 180.0;

/// ### Accelerometer sensor
/// This sensor is used to measure the acceleration of the device.
/// The sensor is connected to the I2C bus and is configured with defaults values.
/// Within this structure you have access to raw and converted data.
pub struct Accelerometer {
    sensor_id: i32,
    /// Raw data from the accelerometer.
    pub raw_data: Data,
    /// Scaled data from the accelerometer in ***m/s²***.
    pub scaled_data: ScaledData,
    offset: ScaledData,
}

impl Accelerometer {
    /// #### Creates a new accelerometer sensor.
    /// ### Arguments
    /// * `sensor_id` - The sensor id to name the device (usually `0x8700A` as reference to the sensor's model).
    pub fn new(sensor_id: i32) -> Self {
        Accelerometer {
            sensor_id,
            raw_data: Data{x:0, y:0, z:0},
            scaled_data: ScaledData{x:0.0, y:0.0, z:0.0},
            offset: ScaledData{x:0.0, y:0.0, z:0.0},
        }
    }

    /// #### Returns basics information about the sensor.
    /// This field is not yet updated through the initialization.
    pub fn get_sensor(&self) -> Sensor {
        Sensor { 
            name: "FXOS8700_A", 
            version: 1, 
            sensor_id: self.sensor_id, 
            sensor_type: SensorType::Accelerometer, 
            max_value: 78.4532, // 8g = 78.4532 m/s^2
            min_value: -78.4532, // -8g = - 78.4532 m/s^2
            resolution: 0.061, // 0.061 mg/LSB at +-2g
            min_delay: 0 
        }
    }

    /// #### Set raw data to the accelerometer sensor on the all axis.
    /// Raw data must be signed 16-bit integers.
    fn set_raw_data(&mut self, x: i16, y: i16, z: i16) {
        self.raw_data.x = x;
        self.raw_data.y = y;
        self.raw_data.z = z;
    }

    /// #### Get Raw data from the accelerometer sensor on the X-axis.
    /// Returns a signed 16-bit integer.
    pub fn get_raw_x(&self) -> i16 {
        self.raw_data.x
    }

    /// #### Get Raw data from the accelerometer sensor on the Y-axis.
    /// Returns a signed 16-bit integer.
    pub fn get_raw_y(&self) -> i16 {
        self.raw_data.y
    }

    /// #### Get Raw data from the accelerometer sensor on the Z-axis.
    /// Returns a signed 16-bit integer.
    pub fn get_raw_z(&self) -> i16 {
        self.raw_data.z
    }

    /// #### Get Raw data from the accelerometer sensor on the all axis.
    /// Returns a `Data` structure with signed 16-bit integers.
    pub fn get_raw_data(&self) -> Data {
        self.raw_data
    }

    /// #### Set scaled data to the accelerometer sensor on the all axis.
    /// Scaled data must be in m/s².
    fn set_scaled_data(&mut self, x: f32, y: f32, z: f32) {
        self.scaled_data.x = x - self.offset.x;
        self.scaled_data.y = y - self.offset.y;
        self.scaled_data.z = z - self.offset.z;
    }

    /// #### Get Scaled data from the accelerometer sensor on the X-axis.
    /// Returns the acceleration in m/s² as a f32. 
    pub fn get_scaled_x(&self) -> f32 {
        self.scaled_data.x
    }

    /// #### Get Scaled data from the accelerometer sensor on the Y-axis.
    /// Returns the acceleration in m/s² as a f32.
    pub fn get_scaled_y(&self) -> f32 {
        self.scaled_data.y
    }

    /// #### Get Scaled data from the accelerometer sensor on the Z-axis.
    /// Returns the acceleration in m/s² as a f32.
    pub fn get_scaled_z(&self) -> f32 {
        self.scaled_data.z
    }

    /// #### Get Scaled data from the accelerometer sensor on the all axis.
    /// Returns a `ScaledData` structure with the acceleration in m/s² as a f32.
    pub fn get_scaled_data(&self) -> ScaledData {
        self.scaled_data
    }

    /// #### Set offset data to the accelerometer sensor on all axis.
    /// This function is used for calibration purposes.
    /// Data must be in m/s².
    /// Offset only affects the scaled data.
    /// 
    /// To find the offset values, refer to the file `accel_gyro_calibration.rs` in the examples DIR.
    /// Or you use the calibration function in the main lib to auto set the offsets. 
    /// By using this function you won't need to manually use `set_offset()`.
    pub fn set_offset(&mut self, x: f32, y: f32, z: f32) {
        self.offset = ScaledData{x, y, z};
    }
    
}

/// ### Magnetometer sensor
/// This sensor is used to measure the magnetic field around the device.
/// The sensor is connected to the I2C bus and is configured with defaults values.
/// Within this structure you have access to raw data and converted data.
pub struct Magnetometer {
    sensor_id: i32,
    /// Raw data from the magnetometer sensor.
    pub raw_data: Data,
    /// Data from the magnetometer sensor converted to ***uT*** but without corrections.
    pub data: ScaledData,
    /// Scaled data from the magnetometer sensor in ***µT*** with corrections.
    pub scaled_data: ScaledData,
    hard_iron: Vector3<f32>,
    soft_iron: Matrix3<f32>,
    calibrated: bool,
}

impl Magnetometer {
    /// #### Creates a new magnetometer sensor.
    /// ### Arguments
    /// * `sensor_id` - The sensor id to name the device (usually `0x8700B` as reference to the sensor's model).
    pub fn new(sensor_id: i32) -> Self {
        Magnetometer {
            sensor_id,
            raw_data: Data{x:0, y:0, z:0},
            data: ScaledData{x:0.0, y:0.0, z:0.0},
            scaled_data: ScaledData{x:0.0, y:0.0, z:0.0},
            hard_iron: Vector3::zeros(),
            soft_iron: Matrix3::identity(),
            calibrated: false,
        }
    }

    /// #### Returns basics information about the sensor.
    /// This field is not yet updated through the initialization.
    pub fn get_sensor(&self) -> Sensor {
        Sensor { 
            name: "FXOS8700_M", 
            version: 1, 
            sensor_id: self.sensor_id, 
            sensor_type: SensorType::MagneticField,
            max_value: 1200.0,
            min_value: -1200.0,
            resolution: 0.0, 
            min_delay: 0
        }
    }

    /// #### Set hard iron offset to the magnetometer sensor on all axis.
    /// This function is used for calibration purposes.
    /// Data must be in ***µT***.
    /// Offset only affects the scaled data.
    /// If you don't want to set the soft iron, put an identity matrix.
    pub fn set_hard_soft_iron(&mut self, hard_iron: Vector3<f32>, soft_iron: Matrix3<f32>) {
        self.hard_iron = hard_iron;
        self.soft_iron = soft_iron;
        self.calibrated = true;
    }
    
    /// #### Set raw data to the magnetometer sensor on the all axis.
    /// Raw data must be signed 16-bit integers.
    fn set_raw_data(&mut self, x: i16, y: i16, z: i16) {
        self.raw_data.x = x;
        self.raw_data.y = y;
        self.raw_data.z = z;
    }

    /// #### Get Raw data from the magnetometer sensor on the X-axis.
    /// Returns a signed 16-bit integer.
    pub fn get_raw_x(&self) -> i16 {
        self.raw_data.x
    }

    /// #### Get Raw data from the magnetometer sensor on the Y-axis.
    /// Returns a signed 16-bit integer.
    pub fn get_raw_y(&self) -> i16 {
        self.raw_data.y
    }

    /// #### Get Raw data from the magnetometer sensor on the Z-axis.
    /// Returns a signed 16-bit integer.
    pub fn get_raw_z(&self) -> i16 {
        self.raw_data.z
    }

    /// #### Get Raw data from the magnetometer sensor on the all axis.
    /// Returns a `Data` structure with signed 16-bit integers.
    pub fn get_raw_data(&self) -> Data {
        self.raw_data
    }

    /// #### Set data to the magnetometer sensor on the all axis.
    /// Data must be in ***µT***.
    fn set_data(&mut self, x: f32, y: f32, z: f32) {
        self.data.x = x;
        self.data.y = y;
        self.data.z = z;
    }

    /// #### Get data from the magnetometer sensor on the X-axis without correction.
    /// Returns the magnetic field in ***µT***.
    pub fn get_x(&self) -> f32 {
        self.data.x
    }

    /// #### Get data from the magnetometer sensor on the Y-axis without correction.
    /// Returns the magnetic field in ***µT***.
    pub fn get_y(&self) -> f32 {
        self.data.y
    }

    /// #### Get data from the magnetometer sensor on the Z-axis without correction.
    /// Returns the magnetic field in ***µT***.
    pub fn get_z(&self) -> f32 {
        self.data.z
    }

    /// #### Set scaled data to the magnetometer sensor on the all axis.
    /// Scaled data must be in µT.
    fn set_scaled_data(&mut self, x: f32, y: f32, z: f32) {
        self.set_data(x, y, z);
        if self.calibrated {
            let data = Vector3::new(x, y, z);
            let data_scaled = self.soft_iron * (data - self.hard_iron);
            self.scaled_data.x = data_scaled[0];
            self.scaled_data.y = data_scaled[1];
            self.scaled_data.z = data_scaled[2];
        } else {
            self.scaled_data.x = x;
            self.scaled_data.y = y;
            self.scaled_data.z = z;
        }
    }

    /// #### Get Scaled data from the magnetometer sensor on the X-axis with correction.
    /// Returns the magnetic field in µT as a f32.
    pub fn get_scaled_x(&self) -> f32 {
        self.scaled_data.x
    }

    /// #### Get Scaled data from the magnetometer sensor on the Y-axis with correction.
    /// Returns the magnetic field in µT as a f32.
    pub fn get_scaled_y(&self) -> f32 {
        self.scaled_data.y
    }

    /// #### Get Scaled data from the magnetometer sensor on the Z-axis with correction.
    /// Returns the magnetic field in µT as a f32.
    pub fn get_scaled_z(&self) -> f32 {
        self.scaled_data.z
    }

    /// #### Get Scaled data from the magnetometer sensor on the all axis with correction.
    /// Returns a `ScaledData` structure with the magnetic field in µT as a f32.
    pub fn get_scaled_data(&self) -> ScaledData {
        self.scaled_data
    }

}

/// ### Gyroscope sensor
/// This sensor is used to measure the angular velocity of the device.
/// The sensor is connected to the I2C bus and is configured with defaults values.
/// Within this structure you have access to raw data and converted data.
pub struct Gyroscope {
    sensor_id: i32,
    /// Raw data from the gyroscope sensor.
    pub raw_data: Data,
    /// Scaled data from the gyroscope sensor in ***rad/s***.
    pub scaled_data: ScaledData,
    offset: ScaledData,
}

impl Gyroscope {
    /// #### Creates a new gyroscope sensor.
    /// ### Arguments
    /// * `sensor_id` - The sensor id to name the device (usually `0x0021002C` as reference to the sensor's model).
    pub fn new(sensor_id: i32) -> Self {
        Gyroscope {
            sensor_id,
            raw_data: Data{x:0, y:0, z:0},
            scaled_data: ScaledData{x:0.0, y:0.0, z:0.0},
            offset: ScaledData{x:0.0, y:0.0, z:0.0},
        }
    }

    /// #### Returns basics information about the sensor. TODO: modify information
    /// This field is not yet updated through the initialization.
    pub fn get_sensor(&self) -> Sensor {
        Sensor { 
            name: "FXAS21002C", 
            version: 1, 
            sensor_id: self.sensor_id, 
            sensor_type: SensorType::Gyroscope,
            max_value: 2000.0 * DEG2RAD,
            min_value: -1. * DEG2RAD,
            resolution: 0.0, 
            min_delay: 0
        }
    }

    /// #### Set raw data to the gyroscope sensor on the all axis.
    /// Raw data must be 16-bit integers
    fn set_raw_data(&mut self, x: i16, y: i16, z: i16) {
        self.raw_data.x = x;
        self.raw_data.y = y;
        self.raw_data.z = z;
    }

    /// #### Get Raw data from the gyroscope sensor on the X-axis.
    /// Returns a signed 16-bit integer.
    pub fn get_raw_x(&self) -> i16 {
        self.raw_data.x
    }

    /// #### Get Raw data from the gyroscope sensor on the Y-axis.
    /// Returns a signed 16-bit integer.
    pub fn get_raw_y(&self) -> i16 {
        self.raw_data.y
    }

    /// #### Get Raw data from the gyroscope sensor on the Z-axis.
    /// Returns a signed 16-bit integer.
    pub fn get_raw_z(&self) -> i16 {
        self.raw_data.z
    }

    /// #### Get Raw data from the gyroscope sensor on the all axis.
    /// Returns a `Data` structure with signed 16-bit integers.
    pub fn get_raw_data(&self) -> Data {
        self.raw_data
    }

    /// #### Set scaled data to the gyroscope sensor on the all axis.
    /// Scaled data must be in rad/s.
    fn set_scaled_data(&mut self, x: f32, y: f32, z: f32) {
        self.scaled_data.x = x - self.offset.x;
        self.scaled_data.y = y - self.offset.y;
        self.scaled_data.z = z - self.offset.z;
    }

    /// #### Get Scaled data from the gyroscope sensor on the X-axis.
    /// Returns the angular velocity in rad/s as a f32.
    pub fn get_scaled_x(&self) -> f32 {
        self.scaled_data.x
    }

    /// #### Get Scaled data from the gyroscope sensor on the Y-axis.
    /// Returns the angular velocity in rad/s as a f32.
    pub fn get_scaled_y(&self) -> f32 {
        self.scaled_data.y
    }

    /// #### Get Scaled data from the gyroscope sensor on the Z-axis.
    /// Returns the angular velocity in rad/s as a f32.
    pub fn get_scaled_z(&self) -> f32 {
        self.scaled_data.z
    }

    /// #### Get Scaled data from the gyroscope sensor on the all axis.
    /// Returns a `ScaledData` structure with the angular velocity in rad/s as a f32.
    pub fn get_scaled_data(&self) -> ScaledData {
        self.scaled_data
    }

    /// #### Set offset data to the accelerometer sensor on all axis.
    /// This function is used for calibration purposes.
    /// Data must be in m/s².
    /// Offset only affects the scaled data.
    /// 
    /// To find the offset values, refer to the file `accel_gyro_calibration.rs` in the examples DIR.
    /// By using this function you won't need to manually use `set_offset()`.
    pub fn set_offset(&mut self, x: f32, y: f32, z: f32) {
        self.offset = ScaledData{x, y, z};
    }

}


#[derive(Debug)]
/// Enumeration to handle main errors we may encounter. 
/// We only consider I2C errors while reading or writing to the sensor
/// and InvalidChipId if the chip ID is not the expected one.
pub enum SensorError<E> {
    /// I2C bus error
    I2c(E),
    /// Invalid chip ID was read
    InvalidChipId(u8),
}

/// ### Put in the same place all sensors, 
/// ie, the accelerometer, the gyroscope and the magnetometer.
///
/// It uses the `I2cBus` to communicate with the sensor.
/// Within this structure, you have access to the accelerometer, the gyroscope and the magnetometer sensors individually.
pub struct AdafruitNXP<I2C> {
    accel_id: i32,
    /// Accelerometer sensor.
    pub accel_sensor: Accelerometer,
    mag_id: i32,
    /// Magnetometer sensor.
    pub mag_sensor: Magnetometer,
    gyro_id: i32,
    /// Gyroscope sensor.
    pub gyro_sensor: Gyroscope,
    i2c: I2C,
    range_accel: AccelMagRange,
    range_gyro: GyroRange,
    mode: AccelMagSensorMode,
    accelmag_rate: AccelMagODR,
    ratio: AccelMagMagOSR,
    gyro_rate: GyroODR,
    enable_calibration: bool,
}

impl<I2C, E> AdafruitNXP<I2C> 
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    /// #### Create a new `AdafruitNXP` structure.
    /// You must provide an `I2C` bus to communicate with the sensor.
    /// Each sensor needs its own ID. 
    /// Respectively, `0x8700A`, `0x8700B` and `0x0021002C` (usually used values).
    /// The sensor will be configured with the default values.
    /// The default values are:
    /// - Accelerometer: `Range 2G`
    /// - Gyroscope: `Range 250°/s`
    /// - Magnetometer: `Over Sampling Ratio 7`
    /// - Over Sampling rate: `100Hz`
    /// - Sensor mode: `Hybrid`
    /// 
    /// ***Usage:***
    /// ```no_run
    /// use adafruit::*;
    /// use rppal::i2c::I2c; // On Raspberry Pi
    /// // Setup the raspberry's I2C interface to create the sensor.
    /// let i2c = I2c::new().unwrap();
    /// // Create an Adafruit object
    /// let mut sensor = AdafruitNXP::new(0x8700A, 0x8700B, 0x0021002C, i2c);
    /// ```
    pub fn new(accel_id: i32, mag_id: i32, gyro_id: i32, i2c: I2C) -> Self {
        AdafruitNXP {
            accel_id,
            accel_sensor: Accelerometer::new(accel_id),
            mag_id,
            mag_sensor: Magnetometer::new(mag_id),
            gyro_id,
            gyro_sensor: Gyroscope::new(gyro_id),
            i2c,
            range_accel: AccelMagRange::Range2g,
            range_gyro: GyroRange::Range250dps,
            mode: AccelMagSensorMode::HybridMode,
            accelmag_rate: AccelMagODR::ODR100HZ,
            ratio: AccelMagMagOSR::MagOSR7,
            gyro_rate: GyroODR::ODR100HZ,
            enable_calibration: false,
        }
    }

    /// #### Write a byte at the specified address.
    /// Returns the error if any or true if no error.
    fn write(&mut self, sensor_addr: u8, addr: u8, data: u8) -> Result<(), SensorError<E>> {
        self.i2c.write(sensor_addr, &[addr, data]).map_err(SensorError::I2c)
    }
    
    /// #### Read a byte at the specified address.
    /// Returns the read byte as a u8.
    fn read(&mut self, sensor_addr: u8, addr:u8) -> Result<u8, SensorError<E>> {
        let mut buffer: [u8; 1] = [0; 1];
        let res = self.i2c.write_read(sensor_addr, &[addr], &mut buffer).map_err(SensorError::I2c);
        match res {
            Ok(_) => Ok(buffer[0]),
            Err(e) => Err(e),
        }
    }

    /// #### Read a bit at the specified address.
    /// Returns the read bit as a u32.
    fn read_bits(&mut self, sensor_addr: u8, addr: u8, bits: u8, shift: u8) -> Result<u32, SensorError<E>> {
        let buffer = self.read(sensor_addr, addr)?;
        let res = (buffer >> shift) as u32;
        Ok(res & ((1 << bits) - 1))
    }

    /// #### Write a bit at the specified address.
    /// Returns the error if any or true if no error.
    fn write_bits(&mut self, sensor_addr: u8, addr: u8, bits: u8, shift: u8, value: u32) -> Result<(), SensorError<E>> {
        let buffer = self.read(sensor_addr, addr)?;
        let mut mask = (1 << bits) - 1;
        let mut data = value;
        data &= mask;
        mask <<= shift;
        let mut val = buffer as u32;
        val &= !mask;
        val |= data << shift; 
        self.write(sensor_addr, addr, val as u8)
    }

    /// ### Initialize the sensor.
    /// Returns the error if any or true if no error.
    fn initialize(&mut self) -> Result<bool, SensorError<E>> {

        /* Configure the accelerometer and magnetometer */

        // Set the full scale range of the accelerometer
        self.set_accel_range(AccelMagRange::Range2g)?;

        // Tell the sensor to enable reduced noise mode
        self.set_accelmag_low_noise_mode(true)?;

        // Tell the sensor to enable high resolution mode.
        self.set_accelmag_osr_mode(config::AccelMagOSRMode::HighResolution)?;

        // Set in hybrid mode, jumps to reg 0x33 after reading 0x06
        self.set_sensor_mode(AccelMagSensorMode::HybridMode)?;

        // Set the output data rate to 100Hz, default 
        self.set_accelmag_output_data_rate(AccelMagODR::ODR100HZ)?;

        // Configure the magnetometer
        // Highest Over Sampling Ratio = 7 > (Over Sampling Rate = 16 @ 100Hz ODR)
        self.set_mag_oversampling_ratio(AccelMagMagOSR::MagOSR7)?;


        /* Configure the gyroscope */

        // Set the full scale range of the gyroscope
        self.set_gyro_range(GyroRange::Range250dps)?;
        self.set_gyro_output_data_rate(GyroODR::ODR100HZ)?;

        // Clear the raw sensor data
        self.accel_sensor.set_raw_data(0, 0, 0);
        self.mag_sensor.set_raw_data(0, 0, 0);
        self.gyro_sensor.set_raw_data(0, 0, 0);

        // If we get here, we're all good
        Ok(true)
    }

    /// ### Begin the configuration of the sensor.
    /// 
    /// The sensor will be configured with the default values by sending I2C messages.
    /// The default values are:
    /// - Accelerometer: `Range 2G`
    /// - Gyroscope: `Range 250°/s`
    /// - Magnetometer: `Over Sampling Ratio 7`
    /// - Over Sampling rate: `100Hz`
    /// - Sensor mode: `Hybrid`
    /// - Calibration: `Disabled`
    /// - Accelerometer: `ODR 100Hz`
    /// - Gyroscope: `ODR 100Hz`
    /// - OSR mode: `High Resolution`
    /// - Low noise mode: `Enabled`
    /// 
    /// ***Usage:***
    /// ```no_run
    /// // Check if the sensor is ready to go
    /// let ready = sensor.begin();
    /// if !ready {
    ///     std::eprintln!("Sensor not detected, check your wiring!");
    ///     std::process::exit(1);
    /// }
    /// ```
    pub fn begin(&mut self) -> Result<bool, SensorError<E>> {
        let who_am_i = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::WhoAmI as u8)?;
        if who_am_i != config::ACCEL_MAG_ID {
            Err(SensorError::InvalidChipId(who_am_i))
        } else {
            let who_am_i = self.read(config::GYRO_ADDR, GyroRegisters::WhoAmI as u8)?;
            if who_am_i != config::GYRO_ID {
                Err(SensorError::InvalidChipId(who_am_i))
            } else {
                self.initialize()
            }
        }
    }

    /// Puts accelerometer/magnetometer into/out of standby mode.
    fn standby_accel(&mut self, standby: bool) -> Result<(), SensorError<E>> {
        let ctrl_reg1 = AccelMagRegisters::CtrlReg1 as u8;
        let sys_mod = AccelMagRegisters::Sysmod as u8;

        let value = match standby {
            true => 0x00,
            false => 0x01,
        };

        self.write_bits(ACCEL_MAG_ADDR, ctrl_reg1, 1, 0, value)?;
        let mut cond = self.read_bits(ACCEL_MAG_ADDR, sys_mod, 2, 0)?;

        if standby {
            while cond != config::AccelMagSystemStatus::Standby as u32 {
                cond = self.read_bits(ACCEL_MAG_ADDR, sys_mod, 2, 0)?;
                for _ in 0..1000000 {
                    // Wait for the sensor to be ready
                }
            }
        } else {
            while cond == config::AccelMagSystemStatus::Standby as u32 {
                cond = self.read_bits(ACCEL_MAG_ADDR, sys_mod, 2, 0)?;
                for _ in 0..1000000 {
                    // Wait for the sensor to be ready
                }
            }
        }

        Ok(())
    }

    /// Puts gyroscope into/out of standby mode.
    fn standby_gyro(&mut self, standby: bool) -> Result<(), SensorError<E>> {
        let ctrl_reg1 = GyroRegisters::CtrlReg1 as u8;

        if standby {
            self.write_bits(GYRO_ADDR, ctrl_reg1, 2, 0, 0x00)?;
            for _ in 0..1000000 {
                // Wait for the sensor to be ready
            }
        } else {
            self.write_bits(GYRO_ADDR, ctrl_reg1, 2, 0, 0x03)?;
            for _ in 0..1000000 {
                // Wait for the sensor to be ready
            }
        }

        Ok(())
    }

    /// #### Set the accelerometer full scale range.
    /// Defaults to 2g.
    /// #### Arguments:
    /// * `range` - The full scale range to set the accelerometer to
    /// 
    /// ***Usage:***
    /// ```rust
    /// use adafruit::*;
    /// sensor.set_accel_range(config::AccelMagRange::Range4g)?;
    /// ```
    pub fn set_accel_range(&mut self, range: AccelMagRange) -> Result<(), SensorError<E>> {
        let ctrl_reg1 = AccelMagRegisters::CtrlReg1 as u8;
        let xyz_data_cfg = AccelMagRegisters::XyzDataCFG as u8;

        self.standby_accel(true)?;
        self.write_bits(ACCEL_MAG_ADDR, xyz_data_cfg, 2, 0, range as u32)?;
        if range == AccelMagRange::Range8g {
            self.write_bits(ACCEL_MAG_ADDR, ctrl_reg1, 1, 2, 0x00)?;
        }
        self.standby_accel(false)?;
        self.range_accel = range;

        Ok(())
    }

    /// #### Set the gyroscope full scale range.
    /// Defaults to 250dps.
    /// #### Arguments:
    /// * `range` - The full scale range to set the gyroscope to
    /// 
    /// ***Usage:***
    /// ```rust
    /// use adafruit::*;
    /// sensor.set_gyro_range(config::GyroRange::Range500dps)?;
    /// ```
    pub fn set_gyro_range(&mut self, range: GyroRange) -> Result<(), SensorError<E>>{
        let ctrl_reg0 = GyroRegisters::CtrlReg0 as u8;
        self.standby_gyro(true)?;
        self.write_bits(GYRO_ADDR, ctrl_reg0, 2, 0, range as u32)?;
        self.standby_gyro(false)?;

        self.range_gyro = range;

        Ok(())
    }

    /// Get the accelerometer full scale range.
    /// #### Returns:
    /// * `AccelMagRange` - The full scale range of the accelerometer
    pub fn get_accel_range(&self) -> AccelMagRange {
        self.range_accel
    }

    /// Set the accelerometer/magnetometer mode.
    /// Defaults to Hybrid mode.
    /// #### Arguments:
    /// * `mode` - The mode to set the accelerometer/magnetometer to
    /// 
    /// ***Usage:***
    /// ```rust
    /// use adafruit::*;
    /// sensor.set_sensor_mode(config::AccelMagSensorMode::HybridMode)?;
    /// ```
    pub fn set_sensor_mode(&mut self, mode: AccelMagSensorMode) -> Result<(), SensorError<E>>{
        let mctrl_reg1 = AccelMagRegisters::MctrlReg1 as u8;
        let mctrl_reg2 = AccelMagRegisters::MctrlReg2 as u8;

        self.standby_accel(true)?;
        self.write_bits(ACCEL_MAG_ADDR, mctrl_reg1, 2, 0, mode as u32)?;
        if mode == AccelMagSensorMode::HybridMode {
            self.write_bits(ACCEL_MAG_ADDR, mctrl_reg2, 1, 5, 1)?;
        } else {
            self.write_bits(ACCEL_MAG_ADDR, mctrl_reg2, 1, 5, 0)?;
        }
        self.standby_accel(false)?;

        self.mode = mode;

        Ok(())
    }

    /// Get the accelerometer/magnetometer mode.
    /// #### Returns:
    /// * `AccelMagSensorMode` - The mode of the accelerometer/magnetometer
    pub fn get_sensor_mode(&self) -> AccelMagSensorMode {
        self.mode
    }

    /// #### Set the Accelerometer/Magnetometer output data rate.
    /// Defaults to 100Hz.
    /// #### Arguments:
    /// * `rate` - The output data rate to set the accelerometer/magnetometer to
    /// 
    /// ***Usage:***
    /// ```rust
    /// use adafruit::*;
    /// sensor.set_accelmag_output_data_rate(config::AccelMagODR::ODR200HZ)?;
    /// ```
    pub fn set_accelmag_output_data_rate(&mut self, rate: AccelMagODR) -> Result<(), SensorError<E>>{
        let mut is_rate_in_mode = false;
        let mut odr: u8 = 0x00;

        if self.mode == AccelMagSensorMode::HybridMode {
            for i in 0..9 {
                if rate == HYBRID_AVAILABLE_ODR[i] {
                    odr = ACCEL_MAG_ODR_DR_BITS[i];
                    is_rate_in_mode = true;
                    break;
                }
            }
        } else {
            for i in 0..9 {
                if rate == ACCEL_MAG_ONLY_AVAILABLE_ODR[i] {
                    odr = ACCEL_MAG_ODR_DR_BITS[i];
                    is_rate_in_mode = true;
                    break;
                }
            }
        }

        if is_rate_in_mode {
            self.standby_accel(true)?;
            self.write(ACCEL_MAG_ADDR, AccelMagRegisters::CtrlReg1 as u8, odr)?;
            self.standby_accel(false)?;
        }


        self.accelmag_rate = rate;

        Ok(())
    }

    /// Set the Gyroscope output data rate.
    /// Defaults to 100Hz.
    /// #### Arguments:
    /// * `rate` - The output data rate to set the gyroscope to
    /// ***Usage:***
    /// ```rust
    /// use adafruit::*;
    /// sensor.set_gyro_output_data_rate(config::GyroODR::ODR200HZ)?;
    /// ```
    pub fn set_gyro_output_data_rate(&mut self, rate: GyroODR) -> Result<(), SensorError<E>>{

        let odr = GYRO_ODR_DR_BITS[rate as usize];

        self.standby_gyro(true)?;
        self.write(GYRO_ADDR, AccelMagRegisters::CtrlReg1 as u8, odr)?;
        self.standby_gyro(false)?;

        self.gyro_rate = rate;

        Ok(())
    }
    
    /// Get the Accelerometer/Magnetometer output data rate.
    /// #### Returns:
    /// * `AccelMagODR` - The output data rate of the accelerometer/magnetometer
    pub fn get_output_data_rate(&self) -> AccelMagODR {
        self.accelmag_rate
    }

    /// Get the Gyroscope output data rate.
    /// #### Returns:
    /// * `GyroODR` - The output data rate of the gyroscope
    pub fn get_gyro_output_data_rate(&self) -> GyroODR {
        self.gyro_rate
    }

    /// Set the Magnetometer oversampling ratio.
    /// Defaults to OSR7.
    /// #### Arguments:
    /// * `ratio` - The oversampling ratio to set the magnetometer to
    /// 
    /// ***Usage:***
    /// ```rust
    /// use adafruit::*;
    /// sensor.set_accelmag_output_data_rate(config::AccelMagODR::ODR200HZ)?;
    /// ```
    pub fn set_mag_oversampling_ratio(&mut self, ratio: AccelMagMagOSR) -> Result<(), SensorError<E>> {
        let mctrl_reg1 = AccelMagRegisters::MctrlReg1 as u8;
        self.standby_accel(true)?;
        self.write_bits(ACCEL_MAG_ADDR, mctrl_reg1, 3, 2, ratio as u32)?;
        self.standby_accel(false)?;

        self.ratio = ratio;

        Ok(())
    }

    /// Get the Magnetometer oversampling ratio.
    /// #### Returns:
    /// * `AccelMagMagOSR` - The oversampling ratio of the magnetometer
    pub fn get_mag_oversampling_ratio(&self) -> AccelMagMagOSR {
        self.ratio
    }

    /// #### Enable low noise mode for the accelerometer and magnetometer.
    /// Note that the FSR setting is restricted to ±2 g or ±4 g mode. 
    /// ***This feature cannot be used in ±8 g mode.***
    pub fn set_accelmag_low_noise_mode(&mut self, enable: bool) -> Result<(), SensorError<E>> {
        let ctrl_reg1 = AccelMagRegisters::CtrlReg1 as u8;
        let val = match enable {
            true => 0x01,
            false => 0x00,
        };
        if self.range_accel != AccelMagRange::Range8g {
            self.standby_accel(true)?;
            self.write_bits(ACCEL_MAG_ADDR, ctrl_reg1, 1, 2, val)?;
            self.standby_accel(false)?;
        }

        Ok(())
    }

    /// #### Set the oversampling modes for the accelerometer and magnetometer.
    /// This setting, along with the ODR selection determines the wake mode power and noise for acceleration measurements.
    /// Defaults to High resolution.
    /// 
    /// ***Usage:***
    /// ```rust
    /// use adafruit::*;
    /// sensor.set_accelmag_osr_mode(config::AccelMagOSRMode::HighResolution)?;
    /// ```
    pub fn set_accelmag_osr_mode(&mut self, mode: AccelMagOSRMode) -> Result<(), SensorError<E>> {
        let ctrl_reg2 = AccelMagRegisters::CtrlReg2 as u8;
        self.standby_accel(true)?;
        self.write_bits(ACCEL_MAG_ADDR, ctrl_reg2, 2, 0, mode as u32)?;
        self.standby_accel(false)?;
        Ok(())
    }

    /// Get the accel mag's accel id.
    pub fn get_accel_id(&self) -> i32 {
        self.accel_id
    }

    /// Get the accel mag's mag id.
    pub fn get_mag_id(&self) -> i32 {
        self.mag_id
    }

    /// Get the accel mag's gyro id.
    pub fn get_gyro_id(&self) -> i32 {
        self.gyro_id
    }

    /// ### Read data from the sensor.
    /// * Sets raw data to the accelerometer, magnetometer, and gyroscope.
    /// * Sets scaled data to the accelerometer, magnetometer, and gyroscope.
    /// 
    /// Returns true if the data is valid and all the reads were successful.
    /// 
    /// ***Usage:***
    /// ```rust
    /// sensor.read_data();
    /// let acc = sensor.accel_sensor.get_scaled_data();
    /// let gyro = sensor.gyro_sensor.get_scaled_data();
    /// let mag = sensor.mag_sensor.get_scaled_data();
    /// ```
    pub fn read_data(&mut self) -> Result<(), SensorError<E>> {

        let buf_xlsb = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::OutXLSB as u8)? as u16;
        let buf_ylsb = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::OutYLSB as u8)? as u16;
        let buf_zlsb = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::OutZLSB as u8)? as u16;
        let buf_xmsb = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::OutXMSB as u8)? as u16;
        let buf_ymsb = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::OutYMSB as u8)? as u16;
        let buf_zmsb = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::OutZMSB as u8)? as u16;
        

        let mode = match self.mode {
            AccelMagSensorMode::AccelOnlyMode => 0b00,
            AccelMagSensorMode::MagOnlyMode => 0b01,
            AccelMagSensorMode::HybridMode => 0b11
        };

        if mode == 0b00 || mode == 0b11 {
            // Clear the raw data placeholder
            self.accel_sensor.set_raw_data(0, 0, 0);

            // We shift shift values to create properly formed integers
            // Note, accel data is 14-bit and left-aligned, so we shift two bit right
            let acc_x = (((buf_xmsb << 8) | buf_xlsb) as i16) >> 2;
            let acc_y = (((buf_ymsb << 8) | buf_ylsb) as i16) >> 2;
            let acc_z = (((buf_zmsb << 8) | buf_zlsb) as i16) >> 2;
            
            // Assign raw values in case someone needs them
            self.accel_sensor.set_raw_data(acc_x, acc_y, acc_z);

            // Convert accel values to m/s²
            let factor = match self.range_accel {
                AccelMagRange::Range2g => config::ACCEL_MG_LSB_2G * G,
                AccelMagRange::Range4g => config::ACCEL_MG_LSB_4G * G,
                AccelMagRange::Range8g => config::ACCEL_MG_LSB_8G * G,
            };

            let scale_acc_x = acc_x as f32 * factor;
            let scale_acc_y = acc_y as f32 * factor;
            let scale_acc_z = acc_z as f32 * factor;

            // Assign scaled values
            self.accel_sensor.set_scaled_data(scale_acc_x, scale_acc_y, scale_acc_z);

        } 
        if mode == 0b01 || mode == 0b11 {
            // Clear the raw data placeholder
            self.mag_sensor.set_raw_data(0, 0, 0);
            
            let buf_xm_lsb = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::MoutXLSB as u8)? as u16;
            let buf_ym_lsb = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::MoutYLSB as u8)? as u16;
            let buf_zm_lsb = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::MoutZLSB as u8)? as u16;
            let buf_xm_msb = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::MoutXMSB as u8)? as u16;
            let buf_ym_msb = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::MoutYMSB as u8)? as u16;
            let buf_zm_msb = self.read(ACCEL_MAG_ADDR, AccelMagRegisters::MoutZMSB as u8)? as u16;
            
            // We shift shift values to create properly formed integers
            let mag_x = ((buf_xm_msb << 8) | buf_xm_lsb) as i16;
            let mag_y = ((buf_ym_msb << 8) | buf_ym_lsb) as i16;
            let mag_z = ((buf_zm_msb << 8) | buf_zm_lsb) as i16;

            // Assign raw values in case someone needs them
            self.mag_sensor.set_raw_data(mag_x, mag_y, mag_z);

            // Convert mag values to uTesla
            let scale_mag_x = mag_x as f32 * MAG_UT_LSB;
            let scale_mag_y = mag_y as f32 * MAG_UT_LSB;
            let scale_mag_z = mag_z as f32 * MAG_UT_LSB;

            // Assign scaled values
            self.mag_sensor.set_scaled_data(scale_mag_x, scale_mag_y, scale_mag_z);
        }

        /* Gyroscope */

        // Clear the raw data placeholder
        self.gyro_sensor.set_raw_data(0, 0, 0);

        let buf_xlsb = self.read(GYRO_ADDR, GyroRegisters::OutXLSB as u8)? as u16;
        let buf_ylsb = self.read(GYRO_ADDR, GyroRegisters::OutYLSB as u8)? as u16;
        let buf_zlsb = self.read(GYRO_ADDR, GyroRegisters::OutZLSB as u8)? as u16;
        let buf_xmsb = self.read(GYRO_ADDR, GyroRegisters::OutXMSB as u8)? as u16;
        let buf_ymsb = self.read(GYRO_ADDR, GyroRegisters::OutYMSB as u8)? as u16;
        let buf_zmsb = self.read(GYRO_ADDR, GyroRegisters::OutZMSB as u8)? as u16;

        // We shift shift values to create properly formed integers
        let gyro_x = ((buf_xmsb << 8) | buf_xlsb) as i16;
        let gyro_y = ((buf_ymsb << 8) | buf_ylsb) as i16;
        let gyro_z = ((buf_zmsb << 8) | buf_zlsb) as i16;

        // Assign raw values in case someone needs them
        self.gyro_sensor.set_raw_data(gyro_x, gyro_y, gyro_z);

        // Convert gyro values to rad/s
        let factor = match self.range_gyro {
            GyroRange::Range250dps => config::GYRO_SENSITIVITY_250DPS * DEG2RAD,
            GyroRange::Range500dps => config::GYRO_SENSITIVITY_500DPS * DEG2RAD,
            GyroRange::Range1000dps => config::GYRO_SENSITIVITY_1000DPS * DEG2RAD,
            GyroRange::Range2000dps => config::GYRO_SENSITIVITY_2000DPS * DEG2RAD,
        };

        let scale_gyro_x = gyro_x as f32 * factor;
        let scale_gyro_y = gyro_y as f32 * factor;
        let scale_gyro_z = gyro_z as f32 * factor;

        // Assign scaled values
        self.gyro_sensor.set_scaled_data(scale_gyro_x, scale_gyro_y, scale_gyro_z);

        Ok(())
    }

    /// #### Enable the calibration for the accelerometer and gyroscope. 
    /// You need to call the calibration function afterwards and only once.
    pub fn enable_calibration(&mut self, enable: bool) {
        self.enable_calibration = enable;
    }

    /// #### Calibrate the accelerometer and gyroscope.
    /// This function will calibrate the accelerometer and gyroscope.
    /// You need to call this function only once before any loop.
    /// You must provide `num_samples` usually 5000 to get accurate results and a `Delay` interface.
    /// 
    /// ***Please note that this function will take some time to execute.***
    /// During this time, you should NOT touch the sensor. 
    /// Moreover, the sensor must be on a FLAT surface.
    /// 
    /// The use of this function is optional. 
    /// However, in order to get accurate results, you should call it or at least run the calibration code example once.
    /// If you use the example code, you will need to hard code the values using the `set_offset` function linked to the sensors.
    /// e.g 
    /// ```
    /// sensor.accel_sensor.set_offset(-0.1234, 0.1234, 0.4321);
    /// sensor.gyro_sensor.set_offset(-0.1234, 0.1234, 0.4321);
    /// ```
    /// 
    /// ***Usage:***
    /// ```rust
    /// let mut delay = Delay::new();
    /// // After the initialization of the sensor
    /// sensor.enable_calibration(true);
    /// sensor.calibrate(5000, delay);
    /// ```
    pub fn calibration<D: DelayMs<u8>>(&mut self, num_samples: u16, delay: &mut D) -> Result<(), SensorError<E>> {
        if self.enable_calibration {
            // Calibration variables
            let (mut acc_min_x, mut acc_min_y, mut acc_min_z) =(0.0_f32, 0.0_f32, 0.0_f32);
            let (mut acc_max_x, mut acc_max_y, mut acc_max_z) = (0.0_f32, 0.0_f32, 0.0_f32);
            let (mut acc_mid_x, mut acc_mid_y, mut acc_mid_z) = (0.0_f32, 0.0_f32, 0.0_f32);

            let (mut gyro_min_x, mut gyro_min_y, mut gyro_min_z) = (0.0_f32, 0.0_f32, 0.0_f32);
            let (mut gyro_max_x, mut gyro_max_y, mut gyro_max_z) = (0.0_f32, 0.0_f32, 0.0_f32);
            let (mut gyro_mid_x, mut gyro_mid_y, mut gyro_mid_z) = (0.0_f32, 0.0_f32, 0.0_f32);

            for _sample in 0..num_samples {
                self.read_data()?;
               
                let ax = self.accel_sensor.get_scaled_x();
                let ay = self.accel_sensor.get_scaled_y();
                let az = self.accel_sensor.get_scaled_z();
                let gx = self.gyro_sensor.get_scaled_x();
                let gy = self.gyro_sensor.get_scaled_y();
                let gz = self.gyro_sensor.get_scaled_z();
        
                acc_min_x = acc_min_x.min(ax);
                acc_min_y = acc_min_y.min(ay);
                acc_min_z = acc_min_z.min(az);
        
                acc_max_x = acc_max_x.max(ax);
                acc_max_y = acc_max_y.max(ay);
                acc_max_z = acc_max_z.max(az);
        
                acc_mid_x = (acc_min_x + acc_max_x) / 2.0;
                acc_mid_y = (acc_min_y + acc_max_y) / 2.0;
                acc_mid_z = (acc_min_z + acc_max_z) / 2.0;
        
                gyro_min_x = gyro_min_x.min(gx);
                gyro_min_y = gyro_min_y.min(gy);
                gyro_min_z = gyro_min_z.min(gz);
        
                gyro_max_x = gyro_max_x.max(gx);
                gyro_max_y = gyro_max_y.max(gy);
                gyro_max_z = gyro_max_z.max(gz);
        
                gyro_mid_x = (gyro_min_x + gyro_max_x) / 2.0;
                gyro_mid_y = (gyro_min_y + gyro_max_y) / 2.0;
                gyro_mid_z = (gyro_min_z + gyro_max_z) / 2.0;
                delay.delay_ms(10_u8);
            }
            // Assign calibration values
            self.accel_sensor.set_offset(acc_mid_x, acc_mid_y, acc_mid_z);
            self.gyro_sensor.set_offset(gyro_mid_x, gyro_mid_y, gyro_mid_z - G);
        }

        Ok(())
    }

}