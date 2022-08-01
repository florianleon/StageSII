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
    let qos_dist = QosProfile::default();
    let qos_agl = QosProfile::default();
    let dist = node.create_publisher::<Float32>("/skippy_distance_data", qos_dist).unwrap();
    let agl = node.create_publisher::<Float32>("/skippy_heading_data", qos_agl).unwrap();

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

    // Initialize the sensor
    sensor.read_data()?;

    let acc_x = sensor.accel_sensor.get_scaled_x();
    let acc_y = sensor.accel_sensor.get_scaled_y();
    let acc_z = sensor.accel_sensor.get_scaled_z();

    let gyro_x = sensor.gyro_sensor.get_scaled_x();
    let gyro_y = sensor.gyro_sensor.get_scaled_y();
    let gyro_z = sensor.gyro_sensor.get_scaled_z();

    let mag_rx = sensor.mag_sensor.get_scaled_x();
    let mag_ry = sensor.mag_sensor.get_scaled_y();
    let mag_rz = sensor.mag_sensor.get_scaled_z();

    // Create a datafusion object
    let mut fusion = Fusion::new(0.05, 20., 50);
    fusion.set_mode(datafusion_imu::Mode::Dof9);

    // Set data to the fusion object
    fusion.set_data_dof9(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_rx, mag_ry, mag_rz);

    // Initialize the datafusion object
    fusion.init();

    // Set magnetic declination --> 1.39951° in Toulouse, France
    fusion.set_declination(1.39951);

    // Distance auto reset variables
    let mut distance= 0.0;
    let mut old_distance:f32;
    let mut counter_reset_distance = 0;

    // Setting up the delta time
    let mut time = Instant::now();

    //Avoid spamming the ROS service
    let mut time_ros = Instant::now();
    let mut time_elapsed_ros: u32;

    loop {

        // Calculate delta time in seconds
        let dt = time.elapsed().as_micros() as f32 / 1_000_000.;
        time = Instant::now();

        // Update old values for the next loop
        fusion.set_old_values(acc_x, acc_y);

        sensor.read_data()?;

        let acc_x = sensor.accel_sensor.get_scaled_x();
        let acc_y = sensor.accel_sensor.get_scaled_y();
        let acc_z = sensor.accel_sensor.get_scaled_z();

        let gyro_x = sensor.gyro_sensor.get_scaled_x();
        let gyro_y = sensor.gyro_sensor.get_scaled_y();
        let gyro_z = sensor.gyro_sensor.get_scaled_z();

        let mag_rx = sensor.mag_sensor.get_scaled_x();
        let mag_ry = sensor.mag_sensor.get_scaled_y();
        let mag_rz = sensor.mag_sensor.get_scaled_z();

        // Set data to the fusion object
        fusion.set_data_dof9(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_rx, mag_ry, mag_rz);

        // Update old variables
        old_distance = distance;

        // Perform a step of the algorithm
        fusion.step(dt);

        // Collect outputs
        let angle_z = fusion.get_heading();
        distance = fusion.get_final_distance();

        // Compare data for distance
        if (old_distance - distance).abs() == 0.0 {
            if counter_reset_distance == 500 {
                counter_reset_distance = 0;
                fusion.reset_distance();
            } else {
                counter_reset_distance += 1;
            }
        } else {
            counter_reset_distance = 0;
        }

        // Print data
        std::println!("Angle Z: {} °", angle_z);
        std::println!("Total distance traveled: {} cm", distance);

        let msg_dist = Float32 {
            data: distance,
        };

        let msg_angle = Float32 {
            data: angle_z,
        };

        time_elapsed_ros = time_ros.elapsed().as_millis() as u32;

        if time_elapsed_ros > 250 {
            time_ros = Instant::now();
            dist.publish(&msg_dist).unwrap();
            agl.publish(&msg_angle).unwrap();
        }

        delay.delay_ms(5_u8);
    }
}
