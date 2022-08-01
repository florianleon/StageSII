use rppal::hal::Delay;
use rppal::i2c::I2c;

use embedded_hal::blocking::delay::*;

use adafruit_nxp::*;

const NUMBER_SAMPLES: u16 = 10_000;

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
    std::println!("Config done!");

    // Calibration variables
    let (mut acc_min_x, mut acc_min_y, mut acc_min_z) =(0.0_f32, 0.0_f32, 0.0_f32);
    let (mut acc_max_x, mut acc_max_y, mut acc_max_z) = (0.0_f32, 0.0_f32, 0.0_f32);
    let (mut acc_mid_x, mut acc_mid_y, mut acc_mid_z) = (0.0_f32, 0.0_f32, 0.0_f32);

    let (mut gyro_min_x, mut gyro_min_y, mut gyro_min_z) = (0.0_f32, 0.0_f32, 0.0_f32);
    let (mut gyro_max_x, mut gyro_max_y, mut gyro_max_z) = (0.0_f32, 0.0_f32, 0.0_f32);
    let (mut gyro_mid_x, mut gyro_mid_y, mut gyro_mid_z) = (0.0_f32, 0.0_f32, 0.0_f32);

    delay.delay_ms(1000_u16);
    std::println!("Please note that depending on the NUMBER_SAMPLES, this might take a while...");
    delay.delay_ms(500_u16);
    std::println!("DO NOT touch or move the sensor until the calibration is done otherwise it will be inaccurate!");
    delay.delay_ms(500_u16);
    std::println!("Place the sensor on a FLAT surface.");
    delay.delay_ms(2500_u16);
    std::println!("Calibration starting in 3...");
    delay.delay_ms(1000_u16);
    std::println!("Calibration starting in 2...");
    delay.delay_ms(1000_u16);
    std::println!("Calibration starting in 1...");
    delay.delay_ms(1000_u16);
    std::println!("Calibration started!");
    
   for _sample in 0..NUMBER_SAMPLES {
        sensor.read_data()?;
       
        let ax = sensor.accel_sensor.get_scaled_x();
        let ay = sensor.accel_sensor.get_scaled_y();
        let az = sensor.accel_sensor.get_scaled_z();
        let gx = sensor.gyro_sensor.get_scaled_x();
        let gy = sensor.gyro_sensor.get_scaled_y();
        let gz = sensor.gyro_sensor.get_scaled_z();

        acc_min_x = acc_min_x.min(ax);
        acc_min_y = acc_min_y.min(ay);
        acc_min_z = acc_min_z.min(az);

        acc_max_x = acc_max_x.max(ax);
        acc_max_y = acc_max_y.max(ay);
        acc_max_z = acc_max_z.max(az);

        acc_mid_x = (acc_min_x + acc_max_x) / 2.0;
        acc_mid_y = (acc_min_y + acc_max_y) / 2.0;
        acc_mid_z = (acc_min_z + acc_max_z) / 2.0;

        gyro_min_x = gyro_min_x.min(gx);
        gyro_min_y = gyro_min_y.min(gy);
        gyro_min_z = gyro_min_z.min(gz);

        gyro_max_x = gyro_max_x.max(gx);
        gyro_max_y = gyro_max_y.max(gy);
        gyro_max_z = gyro_max_z.max(gz);

        gyro_mid_x = (gyro_min_x + gyro_max_x) / 2.0;
        gyro_mid_y = (gyro_min_y + gyro_max_y) / 2.0;
        gyro_mid_z = (gyro_min_z + gyro_max_z) / 2.0;

        std::print!(" Zero acc offset: (");
        std::print!("{:.4}, ", acc_mid_x);
        std::print!("{:.4}, ", acc_mid_y);
        std::print!("{:.4})", acc_mid_z);

        std::print!(" m/s² noise: (");
        std::print!("{:.3}, ", acc_max_x - acc_min_x);
        std::print!("{:.3}, ", acc_max_y - acc_min_y);
        std::println!("{:.3})", acc_max_z - acc_min_z);

        std::print!(" Zero gyro offset: (");
        std::print!("{:.4}, ", gyro_mid_x);
        std::print!("{:.4}, ", gyro_mid_y);
        std::print!("{:.4})", gyro_mid_z);

        std::print!(" rad/s noise: (");
        std::print!("{:.3}, ", gyro_max_x - gyro_min_x);
        std::print!("{:.3}, ", gyro_max_y - gyro_min_y);
        std::println!("{:.3})", gyro_max_z - gyro_min_z);

        delay.delay_ms(10_u8);
   }

    std::print!("\n\n");
    std::println!("Calibration done!");
    std::println!("Final Acceleration offset:");
    std::print!("\t{:.4}, ", acc_mid_x);
    std::print!("\t{:.4}, ", acc_mid_y);
    std::print!("\t{:.4})", acc_mid_z);
    std::println!("Final Gyroscope offset:");
    std::print!("\t{:.4}, ", gyro_mid_x);
    std::print!("\t{:.4}, ", gyro_mid_y);
    std::print!("\t{:.4})", gyro_mid_z - 9.80665);
    Ok(())
}