use std::time::Instant;

use rppal::hal::Delay;
use rppal::i2c::I2c;

use embedded_hal::blocking::delay::*;

use mpu6050::*;

fn main() -> ! {

    let mut delay = Delay::new();

    let i2c = I2c::new().unwrap();
    
    // Create an MPU6050 object.
    std::println!("MPU Configuration...");
    let mut mpu = Mpu6050::new(i2c);
    mpu.init(&mut delay).unwrap();
    std::println!("\t MPU6050 created.");
    mpu.set_accel_range(mpu6050::device::AccelRange::G8).unwrap();
    std::println!("\t MPU6050 Accelerometer Range set.");
    mpu.set_gyro_range(mpu6050::device::GyroRange::D500).unwrap();
    std::println!("\t MPU6050 Gyroscope Range set.");
    std::println!("MPU6050 configured.");

    std::println!("Here we go!");
    delay.delay_ms(1000_u32);

    loop {
        let start = Instant::now();
        let acc = mpu.get_acc().unwrap();
        let acc_x = acc.x;
        let acc_y = acc.y;
        let acc_z = acc.z;
        std::println!("acc: {}, {}, {}", acc_x, acc_y, acc_z);
        let gyro = mpu.get_gyro().unwrap();
        let gyro_x = gyro.x;
        let gyro_y = gyro.y;
        let gyro_z = gyro.z;
        std::println!("gyro: {}, {}, {}", gyro_x, gyro_y, gyro_z);
        delay.delay_ms(100_u32);
        let diff = (Instant::now() - start).as_millis();
        std::println!("Total time (ms): {}", diff);
        std::println!("-------------------------------------");
    }
}