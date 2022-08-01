#![no_std]
#![no_main]

use gy521 as _;
use cortex_m::delay::Delay;
use mpu6050::*;

use stm32_hal2::{
    self,
    clocks::{Clocks, Pllp},
    gpio::{Pin, PinMode, Port, OutputType},
    pac,
    i2c::I2c, i2c_f4::I2cDevice,
};

use defmt_rtt as _;

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Program started.");
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut _dp = pac::Peripherals::take().unwrap();
    
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
    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, clock.systick());
    defmt::println!("I2C Configuration...");
    // Setup the I2C peripheral.
    let mut scl = Pin::new(Port::B, 8, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);
    let mut sda = Pin::new(Port::B, 9, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);

    defmt::println!("\t I2C creation...");
    let i2c = I2c::new(_dp.I2C1, I2cDevice::One, 100_000, &clock);
    defmt::println!("I2C created.");

    // Create an MPU6050 object.
    defmt::println!("MPU Configuration...");
    let mut mpu = Mpu6050::new(i2c);
    
    mpu.init(&mut delay).unwrap();
    defmt::println!("\t MPU6050 created.");
    mpu.set_accel_range(mpu6050::device::AccelRange::G8).unwrap();
    defmt::println!("\t MPU6050 Accelerometer Range set.");
    mpu.set_gyro_range(mpu6050::device::GyroRange::D500).unwrap();
    defmt::println!("\t MPU6050 Gyroscope Range set.");
    defmt::println!("MPU6050 configured.");

    defmt::println!("Here we go!");
    delay.delay_ms(1000);
    
    loop {
        let acc = mpu.get_acc().unwrap();
        let acc_x = acc.x;
        let acc_y = acc.y;
        let acc_z = acc.z;
        defmt::println!("acc: {}, {}, {}", acc_x, acc_y, acc_z);
        let gyro = mpu.get_gyro().unwrap();
        let gyro_x = gyro.x;
        let gyro_y = gyro.y;
        let gyro_z = gyro.z;
        defmt::println!("gyro: {}, {}, {}", gyro_x, gyro_y, gyro_z);
        delay.delay_ms(10);
    }
}

