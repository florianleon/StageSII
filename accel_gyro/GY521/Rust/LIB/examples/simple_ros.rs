use std::time::Instant;

use rppal::hal::Delay;
use rppal::i2c::I2c;

use embedded_hal::blocking::delay::*;

use gy521::{self as _, Gy521};

use r2r::{
    self,
    geometry_msgs::msg::{Accel, Vector3},
    std_msgs::msg::Float32,
    QosProfile,
};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {

    // Create node
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "accel_gyro", "")?;
    let qos_accel = QosProfile::default();
    let qos_dist = QosProfile::default();
    let qos_agl = QosProfile::default();
    let accel = node.create_publisher::<Accel>("/skippy_accel_data", qos_accel)?;
    let dist = node.create_publisher::<Float32>("skippy_distance_data", qos_dist)?;
    let agl = node.create_publisher::<Float32>("skippy_yaw_data", qos_agl)?;

    // Print node data
    println!("node name: {}", node.name()?);
    println!(
        "node fully qualified name: {}",
        node.fully_qualified_name()?
    );

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

    //Avoid spamming the ROS service
    let mut time_ros = Instant::now();
    let mut time_elapsed_ros: u32;

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

        let acc = gy.get_accel_f64();
        let gyro = gy.get_gyro_f64();

        // Print data
        std::println!("Total Z angle traveled: {} °", angle_z);
        std::println!("Total distance traveled: {} cm", distance);

        let msg_accel = Accel {
            linear: Vector3 {
                x: acc.x,
                y: acc.y,
                z: acc.z,
            },
            angular: Vector3 {
                x: gyro.x,
                y: gyro.y,
                z: gyro.z,
            },

        };

        let msg_dist = Float32 {
            data: distance,
        };

        let msg_angle = Float32 {
            data: angle_z,
        };

        time_elapsed_ros = time_ros.elapsed().as_millis() as u32;

        if time_elapsed_ros > 250 {
            time_ros = Instant::now();
            accel.publish(&msg_accel)?;
            dist.publish(&msg_dist)?;
            agl.publish(&msg_angle)?;
        }

        delay.delay_us(10_u32);
    }
}