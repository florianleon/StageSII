use rppal::hal::Delay;
use rppal::i2c::I2c;

use embedded_hal::blocking::delay::*;
use nalgebra::{Vector3, Matrix3};

use adafruit_nxp::*;

fn main() -> Result<(), SensorError<rppal::i2c::Error>> {

    // Init a delay used in certain functions and between each loop.
    let mut delay = Delay::new();

    // Setup the raspberry's I2C interface to create the sensor.
    let i2c = I2c::new().unwrap();

    // Create an Adafruit object
    let mut sensor = AdafruitNXP::new(0x8700A, 0x8700B, 0x0021002C, i2c);

    // Check if the sensor is ready to go
    let ready = sensor.begin()?;
    if !ready {
        std::eprintln!("Sensor not detected, check your wiring!");
        std::process::exit(1);
    }

    sensor.set_accel_range(config::AccelMagRange::Range2g)?;
    sensor.set_gyro_range(config::GyroRange::Range500dps)?;
    sensor.set_accelmag_output_data_rate(config::AccelMagODR::ODR200HZ)?;
    sensor.set_gyro_output_data_rate(config::GyroODR::ODR200HZ)?;

    sensor.accel_sensor.set_offset(-0.0526, -0.5145, 0.1439);
    sensor.gyro_sensor.set_offset(0.0046, 0.0014, 0.0003);
    let hard_iron = Vector3::new(-3.87, -35.45, -50.32);
    let soft_iron = Matrix3::new(1.002, -0.012, 0.002,
                                                                    -0.012, 0.974, -0.010,
                                                                    0.002, -0.010, 1.025);
    sensor.mag_sensor.set_hard_soft_iron(hard_iron, soft_iron);
    std::println!("Config done!");

    delay.delay_ms(2500_u16);

    loop {

        sensor.read_data()?;

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
        std::println!("Accel Y: {}", acc_y);
        std::println!("Accel Z: {}", acc_z);
        std::println!("-----------------------------------------------");
        std::println!("Gyro X: {}", gyro_x);
        std::println!("Gyro Y: {}", gyro_y);
        std::println!("Gyro Z: {}", gyro_z);
        std::println!("-----------------------------------------------");
        std::println!("Mag X: {}", mag_x);
        std::println!("Mag Y: {}", mag_y);
        std::println!("Mag Z: {}", mag_z);

        delay.delay_ms(100_u8);
    }
}