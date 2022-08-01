#![no_std]
#![no_main]

use gy521::{self as _, kalman, systick};
use cortex_m::{self, delay::Delay};
use mpu6050::*;

use stm32_hal2::{
    self,
    clocks::{Clocks, Pllp},
    gpio::{Pin, PinMode, Port, OutputType},
    pac, 
    i2c::I2c, i2c_f4::I2cDevice,
};

use defmt_rtt as _;
use micromath::F32Ext;

/// Sleep for a number of microseconds.
fn delay_us(value: u32) {
    let start = systick::micros();
    while systick::micros() < start + value as u64 {}
}

fn delay_ms(value: u32) {
    // Avoid the counter overflow
    systick::reset();
    let steps = value * 2;
    for _ in 0..steps {
        delay_us(500);
        systick::reset();
    }
    defmt::println!("\t Delay_ms done.")
}

/// Corrects the millisecond bug
fn update_delay() -> u64 {
    let btw = systick::micros();
    systick::reset();
    btw
}

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


#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Program started.");

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    
    let clock = Clocks {
        pllp: Pllp::Div4,
        plln: 84,
        ..Default::default()
    };

    // Write the clock configuration to the MCU. If you wish, you can modify `clocks` above
    // in accordance with [its docs](https://docs.rs/stm32-hal2/0.2.0/stm32_hal2/clocks/index.html),
    // and the `clock_cfg` example.
    clock.setup().unwrap();
    defmt::println!("Clock Configured.");

    //Const variables
    //const RAW2G: f32 = 1.0/16384.0; //peut etre pas utile
    const BUFFER_ZONE: f32 = 0.05;
    const RAD2DEG: f32 = 180.0/mpu6050::PI;
    let sqrt_g = 9.80665.sqrt()*3.0/4.0;


    //Kalman Variables
    let mut gyro_x_angle;
    let mut gyro_y_angle;
    let mut gyro_z_angle;
    let mut kalman_x_angle;
    let mut kalman_y_angle;
    let mut kalman_z_angle;

    let mut kalman_x = kalman::Kalman::new();
    let mut kalman_y = kalman::Kalman::new();
    let mut kalman_z = kalman::Kalman::new();

    let mut syst = cp.SYST;
    let mut delay = Delay::new(syst, clock.systick());


    defmt::println!("I2C Configuration...");
    // Setup the I2C peripheral.
    let mut scl = Pin::new(Port::B, 8, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);
    let mut sda = Pin::new(Port::B, 9, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);

    defmt::println!("\t I2C creation...");
    let i2c = I2c::new(dp.I2C1, I2cDevice::One, 100_000, &clock);
    defmt::println!("I2C created.");

    // Create an MPU6050 object.
    defmt::println!("MPU Configuration...");
    let mut mpu = Mpu6050::new(i2c);
    mpu.init(&mut delay).unwrap();
    mpu.set_accel_range(mpu6050::device::AccelRange::G8).unwrap();
    defmt::println!("\t MPU6050 Accelerometer Range set.");
    mpu.set_gyro_range(mpu6050::device::GyroRange::D500).unwrap();
    defmt::println!("\t MPU6050 Gyroscope Range set.");
    defmt::println!("MPU6050 configured.");

    syst = delay.free();

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
    let mut sens; // 0 = neutre, 1 = droite, 2 = gauche
    let mut sens_prec = 0;
    let mut sens_count = 0;
    #[allow(unused_variables)]
    let mut total_z_angle = 0.0;

    defmt::println!("Offset set and init done.");

    // Systick initialisation
    systick::init_with_frequency(syst, 84_000_000, 1_000);
    defmt::println!("Systick setup done.");
    systick::reset();
    defmt::println!("Syst count started.");
    
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
        let dt = systick::micros() as f32 / 1_000_000.0;

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
        defmt::println!("X: {}, Y: {}, Z: {}, dt:{}", kalman_x_angle, kalman_y_angle, angle_z, dt);
        //defmt::println!("dt: {}, angle Y: {}", dt, kalman_y_angle);
        //defmt::println!("dt: {}, angle: {}", dt, total_z_angle);

        //delay_us(10);
        defmt::println!("Millis: {}", systick::millis());
        defmt::println!("Micros before: {}", systick::micros());
        systick::stop();
        systick::reset();
        systick::start();
        defmt::println!("Micros after: {}", systick::micros());
    }
}
