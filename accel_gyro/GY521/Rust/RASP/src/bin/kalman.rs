use std::{time::Instant, println};

use rppal::hal::Delay;
use rppal::i2c::I2c;

use embedded_hal::blocking::delay::*;

use mpu6050::*;

use gy521::{self as _, kalman::Kalman};

/// Computes roll from accelerometer data.
/// Returns the roll in degrees;
fn compute_roll(accel_y: f32, accel_z: f32) -> f32 {
    accel_y.atan2(accel_z) * 180.0/mpu6050::PI
}

/// Computes pitch from accelerometer data.
/// Returns the pitch in degrees;
fn compute_pitch(accel_x: f32, accel_y: f32, accel_z: f32) -> f32 {
    (-accel_x / (accel_y * accel_y + accel_z * accel_z).sqrt()).atan() * 180.0/mpu6050::PI
}

/// Computes yaw from accelerometer data.
/// Returns the yaw in degrees;
fn compute_yaw(accel_x: f32, accel_z: f32) -> f32 {
    (accel_z / (accel_x * accel_x + accel_z * accel_z).sqrt()).atan() * 180.0/mpu6050::PI
}

/// Computes the integral of the angular velocity.
/// Returns the integral in degrees.
/// Angle should be in degrees/second.
/// dt is the time in seconds since the last call to this function.
fn compute_integral(angle: f32, dt: f32) -> f32 {
    angle.abs() * dt
}

fn main() -> ! {
    println!("Program started.");

    //Const variables
    const BUFFER_ZONE: f32 = 0.05;
    const RAD2DEG: f32 = 180.0/mpu6050::PI;
    let sqrt_g:f32 = 9.80665_f32.sqrt()*3.0/4.0;

    //Kalman Variables
    let mut gyro_x_angle: f32;
    let mut gyro_y_angle: f32;
    let mut gyro_z_angle: f32;
    let mut kalman_x_angle: f32;
    let mut kalman_y_angle: f32;
    let mut kalman_z_angle: f32;

    let mut kalman_x = Kalman::new();
    let mut kalman_y = Kalman::new();
    let mut kalman_z = Kalman::new();

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

    // Seting up yaw offset
    let acc = mpu.get_acc().unwrap();
    let acc_x = acc.x;
    let acc_y = acc.y;
    let acc_z = acc.z;
    
    let roll = compute_roll(acc_y, acc_z);
    let pitch = compute_pitch(acc_x, acc_y, acc_z);
    let yaw = compute_yaw(acc_x, acc_z);

    kalman_x.set_angle(roll);
    kalman_y.set_angle(pitch);
    kalman_z.set_angle(yaw);

    gyro_x_angle = roll;
    gyro_y_angle = pitch;
    gyro_z_angle = yaw;
    // Initialized here because we restrict the pitch later
    kalman_y_angle = pitch;

    // Other variables
    let offset = yaw;
    let mut sens: i8; // 0 = neutre, 1 = droite, -1 = gauche
    let mut sens_prec = 0;
    let mut sens_count = 0;
    #[allow(unused_variables)]
    let mut total_z_angle = 0.0;

    println!("Offset set and init done.");

    delay.delay_ms(2_500_u32);

    let mut time = Instant::now();

    loop {
        // Read data from sensor
        let acc = mpu.get_acc().unwrap();
        let acc_x = acc.x;
        let acc_y = acc.y;
        let acc_z = acc.z;
        let gyro = mpu.get_gyro().unwrap();
        let gyro_x = gyro.x * RAD2DEG;
        let gyro_y = gyro.y * RAD2DEG;
        let gyro_z = gyro.z * RAD2DEG;

        // Calculate delta time in seconds
        let dt = time.elapsed().as_micros() as f32 / 1_000_000.;
        time = Instant::now();

        // Calculate roll, pitch and yaw
        let roll = compute_roll(acc_y, acc_z);
        let pitch = compute_pitch(acc_x, acc_y, acc_z);
        let yaw = compute_yaw(acc_x, acc_z);

        // Calculate the angular speed of the x, y, z axes
        let mut gyro_x_rate = gyro_x;
        let gyro_y_rate = gyro_y;
        let gyro_z_rate = gyro_z;

        // Restrict Pitch
        // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
        if ((pitch < -90.0) && (kalman_y_angle > 90.0)) || ((pitch > 90.0) && (kalman_y_angle < -90.0)) {
            kalman_y.set_angle(pitch);
            kalman_y_angle = pitch;
            gyro_y_angle = pitch;
        } else {
            kalman_y.compute_angle(pitch, gyro_y_rate, dt);
            kalman_y_angle = kalman_y.get_angle(); 
        }

        // Invert rate, so it fits the restriced accelerometer reading
        if kalman_y_angle.abs() > 90.0 {
            gyro_x_rate = -gyro_x_rate;
        }


        kalman_x.compute_angle(roll, gyro_x_rate, dt);
        kalman_x_angle = kalman_x.get_angle();
        kalman_z.compute_angle(yaw, gyro_z_rate, dt);
        kalman_z_angle = kalman_z.get_angle();


        // Calculate gyro angle without any filter
        gyro_x_angle += gyro_x_rate * dt;
        gyro_y_angle += gyro_y_rate * dt;
        gyro_z_angle += gyro_z_rate * dt;

        // Reset the gyro angle when it has drifted too much
        if (gyro_x_angle < -180.0) || (gyro_x_angle > 180.0) {
            gyro_x_angle = kalman_x_angle;
        }
        if (gyro_y_angle < -180.0) || (gyro_y_angle > 180.0) {
            gyro_y_angle = kalman_y_angle;
        }
        if (gyro_z_angle < -180.0) || (gyro_z_angle > 180.0) {
            gyro_z_angle = kalman_z_angle;
        }

        let angle_z = kalman_z_angle - offset;

        // Tries to correct the drift of the angle
        // Acts like a buffer zone
        if angle_z < -BUFFER_ZONE {
            sens = -1;
        } else if angle_z > BUFFER_ZONE {
            sens = 1;
        } else {
            sens = 0;
        }

        // Counts the number of times the angle has changed
        if (sens != sens_prec) && (sens != 0) {
            sens_count += 1;
            sens_prec = sens;
        }

        // Movement is done when the angle has changed more than 3 times
        if sens_count > 3 {
            sens_count = 0;
            sens_prec = 0;
        } else {
            total_z_angle += compute_integral(angle_z, dt) * sqrt_g;
        }

        // Prints the data
        //println!("X: {:?}, Y: {:?}, Z: {:?}", kalman_x_angle, kalman_y_angle, angle_z);
        //println!("dt: {}, angle Y: {}", dt, kalman_y_angle);
        println!("dt: {}, angle: {}", dt, total_z_angle);
    }
}