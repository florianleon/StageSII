use std::time::Instant;

use rppal::hal::Delay;
use rppal::i2c::I2c;

use embedded_hal::blocking::delay::*;

use adafruit_nxp::*;
use datafusion_imu::{self as _, Fusion};

use r2r::{
    self,
    std_msgs::msg::Float32,
    QosProfile,
};

#[tokio::main]
async fn main() -> Result<(), SensorError<rppal::i2c::Error>> {

    // Create node
    let ctx = r2r::Context::create().unwrap();
    let mut node = r2r::Node::create(ctx, "imu", "").unwrap();
    let qos_mag_x_corrected = QosProfile::default();
    let qos_mag_y_corrected = QosProfile::default();
    let qos_mag_z_corrected = QosProfile::default();
    let qos_mag_x = QosProfile::default(); // Without correction
    let qos_mag_y = QosProfile::default(); // Without correction
    let qos_mag_z = QosProfile::default(); // Without correction
    let qos_heading = QosProfile::default();
    let pub_mag_x_corrected = node.create_publisher::<Float32>("/mag_x_corrected", qos_mag_x_corrected).unwrap();
    let pub_mag_y_corrected = node.create_publisher::<Float32>("/mag_y_corrected", qos_mag_y_corrected).unwrap();
    let pub_mag_z_corrected = node.create_publisher::<Float32>("/mag_z_corrected", qos_mag_z_corrected).unwrap();
    let pub_mag_x = node.create_publisher::<Float32>("/mag_x", qos_mag_x).unwrap();
    let pub_mag_y = node.create_publisher::<Float32>("/mag_y", qos_mag_y).unwrap();
    let pub_mag_z = node.create_publisher::<Float32>("/mag_z", qos_mag_z).unwrap();
    let pub_heading = node.create_publisher::<Float32>("/heading", qos_heading).unwrap();

    // Print node data
    println!("node name: {}", node.name().unwrap());
    println!(
        "node fully qualified name: {}",
        node.fully_qualified_name().unwrap()
    );

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

    // Create a datafusion object
    let mut fusion = Fusion::new(0.05, 20., 50);
    fusion.set_mode(datafusion_imu::Mode::Dof9);

    // Initialize the datafusion object
    fusion.init();

    // Set magnetic declination --> 1.39951° in Toulouse, France
    fusion.set_declination(1.39951);

    fusion.disable_distance(true);

    // Setting up the delta time
    let mut time = Instant::now();

    // Avoid spamming the ROS service
    let mut time_ros = Instant::now();
    let mut time_elapsed_ros: u32;

    loop {

        // Calculate delta time in seconds
        let dt = time.elapsed().as_micros() as f32 / 1_000_000.;
        time = Instant::now();

        sensor.read_data()?;

        let acc_x = sensor.accel_sensor.get_scaled_x();
        let acc_y = sensor.accel_sensor.get_scaled_y();
        let acc_z = sensor.accel_sensor.get_scaled_z();

        let gyro_x = sensor.gyro_sensor.get_scaled_x();
        let gyro_y = sensor.gyro_sensor.get_scaled_y();
        let gyro_z = sensor.gyro_sensor.get_scaled_z();

        let mag_x_corrected = sensor.mag_sensor.get_scaled_x();
        let mag_y_corrected = sensor.mag_sensor.get_scaled_y();
        let mag_z_corrected = sensor.mag_sensor.get_scaled_z();

        let mag_x = sensor.mag_sensor.get_x();
        let mag_y = sensor.mag_sensor.get_y();
        let mag_z = sensor.mag_sensor.get_z();


        // Set data to the fusion object
        fusion.set_data_dof9(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x_corrected, mag_y_corrected, mag_z_corrected);

        
        // Perform a step of the algorithm
        fusion.step(dt);

        // Collect outputs
        let angle_z = fusion.get_heading();

        // Messages configuration

        let msg_mag_x_corrected = Float32 {
            data: mag_x_corrected,
        };

        let msg_mag_y_corrected = Float32 {
            data: mag_y_corrected,
        };

        let msg_mag_z_corrected = Float32 {
            data: mag_z_corrected,
        };

        let msg_mag_x = Float32 {
            data: mag_x,
        };

        let msg_mag_y = Float32 {
            data: mag_y,
        };

        let msg_mag_z = Float32 {
            data: mag_z,
        };

        let msg_angle = Float32 {
            data: angle_z,
        };

        time_elapsed_ros = time_ros.elapsed().as_millis() as u32;

        if time_elapsed_ros > 25 {
            time_ros = Instant::now();
            pub_heading.publish(&msg_angle).unwrap();
            pub_mag_x_corrected.publish(&msg_mag_x_corrected).unwrap();
            pub_mag_y_corrected.publish(&msg_mag_y_corrected).unwrap();
            pub_mag_z_corrected.publish(&msg_mag_z_corrected).unwrap();
            pub_mag_x.publish(&msg_mag_x).unwrap();
            pub_mag_y.publish(&msg_mag_y).unwrap();
            pub_mag_z.publish(&msg_mag_z).unwrap();
        }

        delay.delay_ms(5_u8);
    }
}
