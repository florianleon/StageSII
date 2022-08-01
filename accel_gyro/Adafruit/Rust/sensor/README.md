# Adafruit NXP sensor driver

[![made-with-rust](https://img.shields.io/badge/Made%20with-Rust-1f425f?style=plastic)](https://www.rust-lang.org/)
[![powered-by-sii](https://img.shields.io/badge/Powered%20By-SII-blue?style=plastic)](https://sii-group.com/fr-FR/sii-sud-ouest)

This is a driver for the [Adafruit Precision NXP 9-DOF Breakout Board - FXOS8700 + FXAS21002C](https://www.adafruit.com/product/3463).
It is based on the libraries provided by Adafruit in C++ for the [FXAS21002C](https://github.com/adafruit/Adafruit_FXAS21002C)
and the [FXOS8700](https://github.com/adafruit/Adafruit_FXOS8700).

`embedded_hal` based driver with i2c access to the Adafruit chip.
It provides a basic interface to get raw and scaled data from the sensors.
All 3 sensors are separated in their own structure, respectively `Accelerometer`, `Magnetometer` and `Gyroscope`. 

### Calibration
You might need to calibrate and provide offset for the accelerometer and the gyroscope 
-> check `accel_gyro_calibration.rs` example code or the function `calibration()`.
For the magnetometer, you might need to compute hard and soft iron correction. 
If you want to learn more about those correction you can go [here](https://www.fierceelectronics.com/components/compensating-for-tilt-hard-iron-and-soft-iron-effects).
We won't provide any tool for those corrections.
However, you can follow this [tutorial](https://learn.adafruit.com/adafruit-sensorlab-magnetometer-calibration) provided by Adafruit.

### More information
This crate allows for a full customization except for Fifo and hardware interruptions implementation.
This is not plan in the near future.

To use this driver you must provide a concrete `embedded_hal` implementation both for Delay and I2C.

You must be able to compute a delta time. e.g. the execution time of each loop. 
Most examples uses a Raspberry Pi for practicals reasons.
NB: The more accurate the delta time, the more accurate the measurement.

A better ROS2 example might be provided in the future.

### Usage
The example below uses [rppal](https://crates.io/crates/rppal) crates and runs on a Raspberry Pi.
```rust
use rppal::hal::Delay;
use rppal::i2c::I2c;

use embedded_hal::blocking::delay::*;
use nalgebra::{Vector3, Matrix3};

use adafruit::*;

fn main() -> -> Result<(), SensorError<rppal::i2c::Error>> {

    // Init a delay used in certain functions and between each loop.
    let mut delay = Delay::new();

    // Setup the raspberry's I2C interface to create the sensor.
    let i2c = I2c::new().unwrap();

    // Create an Adafruit object.
    let mut sensor = AdafruitNXP::new(0x8700A, 0x8700B, 0x0021002C, i2c);

    // Check if the sensor is ready to go.
    let ready = sensor.begin()?;
    if !ready {
        std::eprintln!("Sensor not detected, check your wiring!");
        std::process::exit(1);
    }
    
    // If you don't want to use the default configuration.
    sensor.set_accel_range(config::AccelMagRange::Range2g)?;
    sensor.set_gyro_range(config::GyroRange::Range500dps)?;
    sensor.set_accelmag_output_data_rate(config::AccelMagODR::ODR200HZ)?;
    sensor.set_gyro_output_data_rate(config::GyroODR::ODR200HZ)?;

    // If you need more precise results you might consider adding offsets. (Values are exemples)
    sensor.accel_sensor.set_offset(-0.0526, -0.5145, 0.1439);
    sensor.gyro_sensor.set_offset(0.0046, 0.0014, 0.0003);
    let hard_iron = Vector3::new(-3.87, -35.45, -50.32);
    let soft_iron = Matrix3::new(1.002, -0.012, 0.002,
                                 -0.012, 0.974, -0.010,
                                 0.002, -0.010, 1.025);
    sensor.mag_sensor.set_hard_soft_iron(hard_iron, soft_iron);

    loop {

        sensor.read_data();

        let acc_x = sensor.accel_sensor.get_scaled_x();
        let acc_y = sensor.accel_sensor.get_scaled_y();
        let acc_z = sensor.accel_sensor.get_scaled_z();

        let gyro_x = sensor.gyro_sensor.get_scaled_x();
        let gyro_y = sensor.gyro_sensor.get_scaled_y();
        let gyro_z = sensor.gyro_sensor.get_scaled_z();

        let mag_x = sensor.mag_sensor.get_scaled_x();
        let mag_y = sensor.mag_sensor.get_scaled_y();
        let mag_z = sensor.mag_sensor.get_scaled_z();

        // Print data
        std::println!("###############################################");
        std::println!("Accel X: {}", acc_x);
        std::print!(" Accel Y: {}", acc_y);
        std::print!(" Accel Z: {}", acc_z);
        std::println!("-----------------------------------------------");
        std::println!("Gyro X: {}", gyro_x);
        std::print!(" Gyro Y: {}", gyro_y);
        std::print!(" Gyro Z: {}", gyro_z);
        std::println!("-----------------------------------------------");
        std::print!("Mag X: {}", mag_x);
        std::print!(" Mag Y: {}", mag_y);
        std::print!(" Mag Z: {}", mag_z);

        delay.delay_ms(5_u8);
    }
}
```