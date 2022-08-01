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

    // Distance auto reset variables
    let mut distance= 0.0;
    let mut old_distance:f32;
    let mut counter_reset_distance = 0;

    // Angle auto reset variables
    let mut angle_z = 0.0;
    let mut old_angle_z: f32;
    let mut counter_reset_angle_z = 0;

    // Setting up the delta time
    let mut time = Instant::now();

    loop {
        // Calculate delta time in seconds
        let dt = time.elapsed().as_micros() as f32 / 1_000_000.;
        time = Instant::now();

        // Update old variables
        old_distance = distance;
        old_angle_z = angle_z;

        // Perform a step of the algorithm
        gy.step(dt);

        // Collect outputs
        angle_z = gy.get_z_angle();
        distance = gy.get_final_distance();

        // Compare data for distance
        if (old_distance - distance).abs() == 0.0 {
            if counter_reset_distance == 500 {
                counter_reset_distance = 0;
                gy.reset_distance();
            } else {
                counter_reset_distance += 1;
            }
        } else {
            counter_reset_distance = 0;
        }

        // Compare data for angle_z
        if (old_angle_z - angle_z).abs() == 0.0 {
            if counter_reset_angle_z == 500 {
                counter_reset_angle_z = 0;
                gy.reset_angle_z();
            } else {
                counter_reset_angle_z += 1;
            }
        } else {
            counter_reset_angle_z = 0;
        }

        // Print data
        std::println!("Total Z angle traveled: {} °", angle_z);
        std::println!("Total distance traveled: {} cm", distance);

        delay.delay_us(10_u32);
    }
}