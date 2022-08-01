use std::{time::Instant, println};

use rppal::hal::Delay;
use rppal::i2c::I2c;

use embedded_hal::blocking::delay::*;

use mpu6050::*;

use gy521::{self as _, filters};
use filters::{Smooth, HPFilter, LPFilter};

/// Computes the integral of the angular velocity.
/// Returns the integral in degrees.
/// Angle should be in degrees/second.
/// dt is the time in seconds since the last call to this function.
fn compute_integral(value: f32, dt: f32) -> f32 {
    value * dt
}

/// Computes the distance
fn distance(x: f32, y: f32) -> f32 {
    (x * x + y * y).sqrt()
}


fn main() -> ! {

    //Const variables
    //const RAW2G: f32 = 1.0/16384.0; //peut etre pas utile
    const BUFFER_ZONE_X: f32 = 7.;
    const BUFFER_ZONE_Y: f32 = 4.;
    const SCALE_FACTOR: f32 = 100.;
    const G_TO_CM2: f32 = 980.665;
    const G: f32 = 9.80665;
    const G2: f32 = G * G;
    let sqrt_g = G.sqrt();
    let correction_factor = G2/sqrt_g * 10.;
    const OFFSET_COUNTER: u16 = 250;
    const RESET_COUNTER: u16 = 500;
    const PRINT_ACC: bool = false;

    //Creating I2C object
    let i2c = I2c::new().unwrap();

    //Creating delay object
    let mut delay = Delay::new();

    // Create an MPU6050 object.
    println!("MPU Configuration...");
    let mut mpu = Mpu6050::new(i2c);
    mpu.init(&mut delay).unwrap();
    println!("\t MPU6050 created.");
    mpu.set_accel_range(mpu6050::device::AccelRange::G8).unwrap();
    println!("\t MPU6050 Accelerometer Range set.");
    mpu.set_gyro_range(mpu6050::device::GyroRange::D500).unwrap();
    println!("\t MPU6050 Gyroscope Range set.");
    println!("MPU6050 configured.");

    // Read data from the MPU6050.
    let acc = mpu.get_acc().unwrap();
    let acc_x = acc.x;
    let acc_y = acc.y;

    
    //X-AXIS
    //SPEED
    //High pass filter for the speed of the X-axis
    let mut hp_vx = HPFilter::new(0.05);
    let mut hp_e_vx = acc_x;
    let mut old_hp_e_vx: f32;
    let mut hp_s_vx = hp_e_vx;

    //We define speed and distance variables
    let mut speed_x: f32;
    let mut distance_x = 0.;

    //Low pass filter for the speed of the X-axis
    let mut lp_vx = LPFilter::new(20.);
    let mut lp_e_vx: f32;
    let mut lp_s_vx = hp_s_vx;

    //DISTANCE
    //High pass filter for the distance of the X-axis
    let mut hp_dx = HPFilter::new(0.05);
    let mut hp_e_dx = acc_x;
    let mut old_hp_e_dx: f32;
    let mut hp_s_dx = acc_x;

    //Low pass filter for the distance of the X-axis
    let mut lp_dx = LPFilter::new(20.);
    let mut lp_e_dx: f32;
    let mut lp_s_dx = hp_s_dx;

    //MOVING AVERAGE
    //Moving average filter of the X-axis
    let mut ma_x = Smooth::new(50);
    let mut average_x = 0.0;


    //Y-AXIS
    //SPEED
    //High pass filter for the speed of the Y-axis
    let mut hp_vy = HPFilter::new(0.05);
    let mut hp_e_vy = acc_y;
    let mut old_hp_e_vy: f32;
    let mut hp_s_vy = acc_y;

    //We define speed and distance variables
    let mut speed_y: f32;
    let mut distance_y = 0.;
    let mut offset_x = 0.;

    //Low pass filter for the speed of the Y-axis
    let mut lp_vy = LPFilter::new(20.);
    let mut lp_e_vy: f32;
    let mut lp_s_vy = hp_s_vy;

    //DISTANCE
    //High pass filter for the distance of the Y-axis
    let mut hp_dy = HPFilter::new(0.05);
    let mut hp_e_dy = acc_y;
    let mut old_hp_e_dy: f32;
    let mut hp_s_dy = acc_y;

    //Low pass filter for the distance of the Y-axis
    let mut lp_dy = LPFilter::new(20.);
    let mut lp_e_dy: f32;
    let mut lp_s_dy = hp_s_dy;

    //MOVING AVERAGE
    //Moving average filter of the Y-axis
    let mut ma_y = Smooth::new(50);
    let mut average_y = 0.0;
    let mut offset_y = 0.;

    println!("Filters created and configured.");

    // Reset the count
    let mut old_dx = 0.;
    let mut old_dy = 0.;
    let mut counter_reset = 0;

    //Other variables
    let mut counter = 0;
    let mut ready_2_go = false;
    let mut final_distance: f32;
    
    println!("Correction factor is: {}", correction_factor);
    println!("Here we go!");
    delay.delay_ms(2_500_u32);

    let mut time = Instant::now();

    loop {
        // Offset management
        if counter == OFFSET_COUNTER {
            offset_x = average_x;
            offset_y = average_y;
            ready_2_go = true;
            distance_x = 0.;
            distance_y = 0.;
            println!("Offset set.");
        }
        if counter < OFFSET_COUNTER + 1 {
            counter += 1;
        }

        // Read data from sensor
        let acc = mpu.get_acc().unwrap();
        let acc_x = acc.x * G_TO_CM2;
        let acc_y = acc.y * G_TO_CM2;

        // Calculate delta time in seconds
        let dt = time.elapsed().as_micros() as f32 / 1_000_000.;
        time = Instant::now();


        // X-AXIS
        // We update the old value of the speed HP filter
        old_hp_e_vx = hp_e_vx;

        // HP filter for the speed of the X-axis
        hp_e_vx = acc_x;
        hp_s_vx = hp_vx.compute(hp_e_vx, old_hp_e_vx, hp_s_vx, dt);

        // LP filter for the speed of the X-axis
        lp_e_vx = hp_s_vx;
        lp_s_vx = lp_vx.compute(lp_e_vx, lp_s_vx, dt);

        // We compute VX
        speed_x = compute_integral(lp_s_vx, dt);

        // we update the old value of the distance HP filter
        old_hp_e_dx = hp_e_dx;

        // HP filter for the distance of the X-axis
        hp_e_dx = speed_x;
        hp_s_dx = hp_dx.compute(hp_e_dx, old_hp_e_dx, hp_s_dx, dt);

        // LP filter for the distance of the X-axis
        lp_e_dx = hp_s_dx;
        lp_s_dx = lp_dx.compute(lp_e_dx, lp_s_dx, dt);


        // Y-AXIS
        // We update the old value of the speed HP filter
        old_hp_e_vy = hp_e_vy;

        // HP filter for the speed of the Y-axis
        hp_e_vy = acc_y;
        hp_s_vy = hp_vy.compute(hp_e_vy, old_hp_e_vy, hp_s_vy, dt);

        // LP filter for the speed of the Y-axis
        lp_e_vy = hp_s_vy;
        lp_s_vy = lp_vy.compute(lp_e_vy, lp_s_vy, dt);

        // We compute VY
        speed_y = compute_integral(lp_s_vy, dt);

        // we update the old value of the distance HP filter
        old_hp_e_dy = hp_e_dy;

        // HP filter for the distance of the Y-axis
        hp_e_dy = speed_y;
        hp_s_dy = hp_dy.compute(hp_e_dy, old_hp_e_dy, hp_s_dy, dt);

        // LP filter for the distance of the Y-axis
        lp_e_dy = hp_s_dy;
        lp_s_dy = lp_dy.compute(lp_e_dy, lp_s_dy, dt);


        // Smoothing the curves
        ma_x.add_reading(lp_s_dx);
        average_x = ma_x.get_average() * SCALE_FACTOR - offset_x;
        ma_y.add_reading(lp_s_dy);
        average_y = ma_y.get_average() * SCALE_FACTOR - offset_y;


        // We compute the distance on the X-axis
        if (average_x.abs() > BUFFER_ZONE_X) && ready_2_go {
            old_dx = distance_x;
            distance_x += compute_integral(average_x, dt) * correction_factor/SCALE_FACTOR;
        }

        // We compute the distance on the Y-axis
        if (average_y.abs() > BUFFER_ZONE_Y) && ready_2_go {
            old_dy = distance_y;
            distance_y += compute_integral(average_y, dt) * correction_factor/SCALE_FACTOR;
        }

        // Reset the distance measured if too much time has ellapsed withour change
        if ((old_dx - distance_x).abs() == 0. || (old_dy - distance_y).abs() == 0.) && ready_2_go {
            if counter_reset == RESET_COUNTER {
                counter_reset = 0;
                distance_x = 0.;
                distance_y = 0.;
                println!("Reset.");
            } else {
                counter_reset += 1;
            }
        } else {
            counter_reset = 0;
        }

        // We compute the final distance
        final_distance = distance(distance_x, distance_y);

        // We print the data
        if PRINT_ACC {
            println!("{:?}, {:?}", average_x.abs(), average_y.abs());
        } else {
            println!("{:?}, {:?}, {:?}", distance_x, distance_y, final_distance);
        }
        
        delay.delay_us(10_u32);

    }


}