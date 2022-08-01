
# GY521 sensor driver

## Introduction

This is `embedded_hal` based driver with i2c access to the MPU6050 chip.
This driver is based on the [Mpu6050](https://github.com/juliangaal/mpu6050).
To use this driver you must provide a concrete `embedded_hal` implementation both for Delay and I2C.

***You must be able to compute a delta time. e.g. the execution time of each loop.*** This is why most examples uses a Raspberry Pi.  
The more accurate the delta time, the more accurate the measurement.

A ROS2 example will be provided but is currently on hold because of a dependency error with the r2r crate.

> 🔴 ***For now, the sensor needs to be perfectly flat to work. Also, using angle and distance measurement at the same time is not recommended.***  
> More specifically, for the distance it doesn't really matter as the high pass filter will automatically delete the angle offset after a few second. But, it needs to stay in the same inclination during the measurement.  
> However, for the angle (and hopefully this will change in the future) the sensor has to be flat or at least stay within the same inclination on the X and Y-axis after initialization.  

---
The example below uses [rppal](https://crates.io/crates/rppal) crates and runs on a Raspberry Pi.

```rust
use std::time::Instant;
use rppal::hal::Delay;
use rppal::i2c::I2c;
use embedded_hal::blocking::delay::*;
use gy521::{self as _, Gy521};

fn main() -> ! {

    // Init a delay used in certain functions and between each loop.
    let mut delay = Delay::new();

    // Setup the raspberry's I2C interface to create the sensor.
    let i2c = I2c::new().unwrap();
    
    // Create an Gy521 object
    let mut gy = Gy521::new(i2c, 0.05, 20., 50);
 
    // Initialize the sensor
    gy.init(&mut delay).unwrap();

   // Setting up the delta time
    let mut time = Instant::now();
    
    loop {
        // Calculate delta time in seconds
        let dt = time.elapsed().as_micros() as f32 / 1_000_000.;
        time = Instant::now();

        // Perform a step of the algorithm
        gy.step(dt);

        // Collect outputs
        let angle_x = gy.get_x_angle();
        let angle_y = gy.get_y_angle();
        let angle_z = gy.get_z_angle();
        let distance = gy.get_final_distance();

        // Print data
        std::println!("Angle X: {} °", angle_x);
        std::println!("Angle Y: {} °", angle_y);
        std::println!("Total Z angle traveled: {} °", angle_z);
        std::println!("Total distance traveled: {} cm", distance);

        delay.delay_us(10_u32);
    }
}
```

---

## Usage

Include this crate in your Cargo project by adding the following to `Cargo.toml`:

```toml
[dependencies]
gy521 = "0.1.0"
embedded-hal = "0.2.7"
nalgebra = "0.30.1" # Optional -> needed to collect full acceleration/gyroscope vector
```

We provide the following examples:

* `rasp` -> To run our code using a Raspberry Pi. This works the same for other platforms.
* `auto_reset` -> Based on the `rasp` code and shows how to auto reset the values.
* `simple_ros` -> Based on the `auto_reset` code and shows how to publish data to ros topics (using ros2)

---

## Documentation

As this package is not published to *`crates.io`* yet, you need to generate the documentation using cargo:

```bash
cargo doc --open
```

---

## ROS

In order to use ROS2 with this package you must follow this [tutorial](https://docs.ros.org/en/galactic/Installation/Ubuntu-Development-Setup.html).  
> PS: Don't forget to source ROS before building your code.  

If you are working with a Raspberry Pi you might need to run the following command if you encountered missing *.h files.  
```bash
sudo apt install libclang-dev # Only if libclang is missing
sudo apt install gcc-multilib