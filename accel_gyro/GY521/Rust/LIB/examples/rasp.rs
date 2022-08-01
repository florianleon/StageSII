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