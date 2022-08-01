//! # GY521 sensor driver
//! 
//! `embedded_hal` based driver with i2c access to the MPU6050 chip.
//! This driver is based on the [Mpu6050](https://github.com/juliangaal/mpu6050).
//! 
//! To use this driver you must provide a concrete `embedded_hal` implementation both for Delay and I2C.
//! 
//! You must be able to compute a delta time. e.g. the execution time of each loop. This is why most examples uses a Raspberry Pi.
//! The more accurate the delta time, the more accurate the measurement.
//! 
//! A ROS2 example will be provided but is currently on hold because of a dependency error with the r2r crate. 
//! 
//! > 🔴 ***For now, the sensor needs to be perfectly flat to work. Also, using angle and distance measurement at the same time is not recommended.***  
//! > More specifically, for the distance it doesn't really matter as the high pass filter will automatically delete the angle offset after a few second.  
//! > However, for the angle (and hopefully this will change in the future) the sensor has to be flat or at least stay within the same inclination on the X and Y-axis after initialization.  
//! 
//! The example below uses [rppal](https://crates.io/crates/rppal) crates and runs on a Raspberry Pi.
//! 
//! ```no_run
//! use std::time::Instant;
//! use rppal::hal::Delay;
//! use rppal::i2c::I2c;
//! use embedded_hal::blocking::delay::*;
//! use gy521::{self as _, Gy521};
//!
//! fn main() -> ! {
//!
//!     // Init a delay used in certain functions and between each loop.
//!     let mut delay = Delay::new();
//!
//!     // Setup the raspberry's I2C interface to create the sensor.
//!     let i2c = I2c::new().unwrap();
//! 
//!     // Create an Gy521 object
//!     let mut gy = Gy521::new(i2c, 0.05, 20., 50);
//! 
//!     // Initialize the sensor
//!     gy.init(&mut delay).unwrap();
//! 
//!     // Setting up the delta time
//!     let mut time = Instant::now();
//! 
//!     loop {
//!         // Calculate delta time in seconds
//!         let dt = time.elapsed().as_micros() as f32 / 1_000_000.;
//!         time = Instant::now();
//! 
//!         // Perform a step of the algorithm
//!         gy.step(dt);
//! 
//!         // Collect outputs
//!         let angle_x = gy.get_x_angle();
//!         let angle_y = gy.get_y_angle();
//!         let angle_z = gy.get_z_angle();
//!         let distance = gy.get_final_distance();
//! 
//!         // Print data
//!         std::println!("Angle X: {} °", angle_x);
//!         std::println!("Angle Y: {} °", angle_y);
//!         std::println!("Total Z angle traveled: {} °", angle_z);
//!         std::println!("Total distance traveled: {} cm", distance);
//! 
//!         delay.delay_us(10_u32);
//!     }
//! }
//! ```


mod kalman;
mod filters;
use mpu6050::*;
use nalgebra::Vector3;
use crate::kalman::Kalman;
use crate::filters::{Smooth, HPFilter, LPFilter};
use embedded_hal::{
    blocking::delay::DelayMs,
    blocking::i2c::{Write, WriteRead},
};

/// Gravity constant.
pub const G: f32 = 9.80665;
/// Squared G.
pub const G2: f32 = G * G;
/// PI. 3.14159274_f32
pub const PI: f32 = core::f32::consts::PI;
/// Constant to convert radians to degrees.
pub const RAD2DEG: f32 = 180.0 / PI;
/// Constant to convert degrees to radians.
pub const DEG2RAD: f32 = PI / 180.0;
// Square root of g.
pub const SQRT_G: f32 = 3.1315571207;
/// Tweaked square root of g. Used to correct the output value.
pub const SQRT_G_KALMAN: f32 = SQRT_G * 3.0 / 4.0;
/// Converts g to a scale value, here in squared centimeters.
pub const G_TO_SCALE: f32 = 980.665;
/// Correction factor to tweak the output value of our distance.
pub const CORRECTION_FACTOR: f32 = G2 / SQRT_G * 10.;
/// Buffer zone used with the Kalman filter.
pub const BUFFER_ZONE_KALMAN: f32 = 0.05;
/// Factor used to scale up or down the tolerance. 1 is normal. 100 is 100 times the tolerance.
/// Defaults to 100 for better results. Tweaks BUFFER_ZONE_X and BUFFER_ZONE_Y.
pub const SCALE_FACTOR: f32 = 100.;
/// Buffer zone used on the X-axis to compute the distance.
pub const BUFFER_ZONE_X: f32 = 0.07 * SCALE_FACTOR; 
/// Buffer zone used on the Y-axis to compute the distance.
pub const BUFFER_ZONE_Y: f32 = 0.04 * SCALE_FACTOR;
/// Counter used to set the offset. Currently 250 iterations. 
const OFFSET_COUNTER: u32 = 250;

 
#[derive(Debug)]
pub enum Error<I2cError> {
    I2cError(I2cError),
}

/// #### Puts it the same place the Mpu6050 object and filters associated with an IMU.
/// 
/// This will allow you to: 
/// - Easily read data
/// - Performs a Kalman filter:
///     - Angle for X and Y-axis
///     - Angular velocity for Z-Axis
/// - Get the Z-axis angle traveled
/// - Perform a series of filters X and Y-axis to:
///     - Get the velocity on each axis
///     - Get the distance traveled
pub struct Gy521<I2C> {
    /// ## MPU6050 object. 
    /// Used to communicate with the MPU6050.
    /// Set to public so you can tweak even more the behavior.
    pub mpu: Mpu6050<I2C>,
    //Generic Variables
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
    gyro_x: f32,
    gyro_y: f32,
    gyro_z: f32,
    roll: f32,
    pitch: f32,
    yaw: f32,
    //Kalman Variables
    /// Final X-Axis angle.
    kalman_x_angle: f32,
    /// Final Y-Axis angle.
    kalman_y_angle: f32,
    /// Intermediate Z-Axis angle. As we don't have a magnetometer, this is actually the angular velocity around the Z-axis.
    kalman_z_angle: f32,
    /// Kalman filter used to correct the output value on the X-axis.
    kalman_x: Kalman,
    /// Kalman filter used to correct the output value on the Y-axis.
    kalman_y: Kalman,
    /// Kalman filter used to correct the output value on the Z-axis.
    kalman_z: Kalman,
    /// Corrects the final Z-axis angle.
    offset_kalman: f32,
    /// In which position the point is relatively to the abscissa. -1 is left, 0 is neutral, 1 is right.
    sens: i8,
    /// Last recorded sens.
    sens_prec: i8,
    /// How many times the curve has crossed the abscissa.
    sens_count: i32,
    /// Final Z-Axis angle. As we don't have a magnetometer, this is actually the angular velocity around the Z-axis.
    total_z_angle: f32,
    //Distance Variables
    //X-AXIS
    //SPEED
    /// High pass filter for the speed of the X-axis.
    hp_vx: HPFilter,
    /// High pass filter entry for the speed of the X-axis.
    hp_e_vx: f32,
    /// Old high pass filter entry for the speed of the X-axis.
    old_hp_e_vx: f32,
    /// High pass filter output for the speed of the X-axis.
    hp_s_vx: f32,
    /// Speed on the X-axis.
    speed_x: f32,
    /// Distance on the X-axis.
    distance_x: f32,
    /// Low pass filter for the speed of the X-axis.
    lp_vx: LPFilter,
    /// Low pass filter entry for the speed of the X-axis.
    lp_e_vx: f32,
    /// Low pass filter output for the speed of the X-axis.
    lp_s_vx: f32,
    //DISTANCE
    /// High pass filter for the distance of the X-axis.
    hp_dx: HPFilter,
    /// High pass filter entry for the distance of the X-axis.
    hp_e_dx: f32,
    /// Old high pass filter entry for the distance of the X-axis.
    old_hp_e_dx: f32,
    /// High pass filter output for the distance of the X-axis.
    hp_s_dx: f32,
    /// Low pass filter for the distance of the X-axis.
    lp_dx: LPFilter,
    /// Low pass filter entry for the distance of the X-axis.
    lp_e_dx: f32,
    /// Low pass filter output for the distance of the X-axis.
    lp_s_dx: f32,
    //MOVING AVERAGE
    /// Moving average filter of the X-axis.
    ma_x: Smooth,
    /// Moving average filter output value of the X-axis.
    average_x: f32,
    /// Offset of the X-axis.
    offset_x: f32,
    //Y-AXIS
    //SPEED
    /// High pass filter for the speed of the Y-axis.
    hp_vy: HPFilter,
    /// High pass filter entry for the speed of the Y-axis.
    hp_e_vy : f32,
    /// Old high pass filter entry for the speed of the Y-axis.
    old_hp_e_vy: f32,
    /// High pass filter output for the speed of the Y-axis.
    hp_s_vy: f32,
    /// Speed on the Y-axis.
    speed_y: f32,
    /// Distance on the Y-axis.
    distance_y: f32,
    /// Low pass filter for the speed of the Y-axis.
    lp_vy: LPFilter,
    /// Low pass filter entry for the speed of the Y-axis.
    lp_e_vy: f32,
    /// Low pass filter output for the speed of the Y-axis.
    lp_s_vy: f32,
    //DISTANCE
    //High pass filter for the distance of the Y-axis.
    hp_dy: HPFilter,
    /// High pass filter entry for the distance of the Y-axis.
    hp_e_dy: f32,
    /// Old high pass filter entry for the distance of the Y-axis.
    old_hp_e_dy: f32,
    /// High pass filter output for the distance of the Y-axis.
    hp_s_dy: f32,
    /// Low pass filter for the distance of the Y-axis.
    lp_dy: LPFilter,
    /// Low pass filter entry for the distance of the Y-axis.
    lp_e_dy: f32,
    /// Low pass filter output for the distance of the Y-axis.
    lp_s_dy: f32,
    //MOVING AVERAGE
    /// Moving average filter of the Y-axis.
    ma_y: Smooth,
    /// Moving average filter output value of the Y-axis.
    average_y: f32,
    /// Offset of the Y-axis.
    offset_y: f32,
    //Other variables
    /// Counter used to set both X and Y-Axis offset.
    counter: u32,
    /// When the offsets are sets we start the filters.
    ready_2_go: bool,
    /// Final distance traveled by the sensor.
    final_distance: f32,
}

impl<I2C, E: core::fmt::Debug> Gy521<I2C> 
where
    I2C: Write<Error = E> + WriteRead<Error = E>,
{
    /// ### Create a new Gy521 object. 
    /// ***You must provide an I2C interface.***
    /// The device will be set to the default configuration. (More customization might come in the future)  
    /// 
    /// *hp_fc* and *lp_fc* are the cut-off frequencies of the high-pass and low-pass filters.
    /// We recommend 0.05 Hz for the high pass filter and 20.0 for the low pass filter.
    /// 
    /// *num_readings* is the number of readings used to compute the moving average.
    /// As the number of readings increases, more information about the signal will be lost. 
    /// On the other hand, the lower the number of readings, the rougher the signal and the more approximate the measurement. 
    /// It is therefore necessary to find a middle ground, so we recommend a value between 25 and 50.
    /// 
    /// ***Usage:***
    /// ```no_run
    /// use gy521::{self as _, Gy521};
    /// use rppal::i2c::I2c; // On Raspberry Pi
    /// let i2c = I2c::new().unwrap();
    /// let mut gy = Gy521::new(i2c, 0.05, 20., 50);
    /// ```
    pub fn new(i2c: I2C, hp_fc: f32, lp_fc: f32, num_readings: u8) -> Self {
        Gy521 {
            mpu: Mpu6050::new(i2c),
            //General Variables
            acc_x: 0.,
            acc_y: 0.,
            acc_z: 0.,
            gyro_x: 0.,
            gyro_y: 0.,
            gyro_z: 0.,
            roll: 0.,
            pitch: 0.,
            yaw: 0.,
            //Kalman Variables
            kalman_x_angle: 0.,
            kalman_y_angle: 0.,
            kalman_z_angle: 0.,
            kalman_x: Kalman::new(),
            kalman_y: Kalman::new(),
            kalman_z: Kalman::new(),
            offset_kalman: 0.,
            sens: 0,
            sens_prec: 0,
            sens_count: 0,
            total_z_angle: 0.,
            //Distance Variables
            //X-AXIS
            //SPEED
            //High pass filter for the speed of the X-axis
            hp_vx: HPFilter::new(hp_fc),
            hp_e_vx: 0.,
            old_hp_e_vx: 0.,
            hp_s_vx: 0.,
            //We define speed and distance variables
            speed_x: 0.,
            distance_x: 0.,
            //Low pass filter for the speed of the X-axis
            lp_vx: LPFilter::new(lp_fc),
            lp_e_vx: 0.,
            lp_s_vx: 0.,
            //DISTANCE
            //High pass filter for the distance of the X-axis
            hp_dx: HPFilter::new(hp_fc),
            hp_e_dx: 0.,
            old_hp_e_dx: 0.,
            hp_s_dx: 0.,
            //Low pass filter for the distance of the X-axis
            lp_dx: LPFilter::new(lp_fc),
            lp_e_dx: 0.,
            lp_s_dx: 0.,
            //MOVING AVERAGE
            //Moving average filter of the X-axis
            ma_x: Smooth::new(num_readings),
            average_x: 0.,
            offset_x: 0.,
            //Y-AXIS
            //SPEED
            //High pass filter for the speed of the Y-axis
            hp_vy: HPFilter::new(hp_fc),
            hp_e_vy : 0.,
            old_hp_e_vy: 0.,
            hp_s_vy: 0.,
            //We define speed and distance variables
            speed_y: 0.,
            distance_y: 0.,
            //Low pass filter for the speed of the Y-axis
            lp_vy: LPFilter::new(lp_fc),
            lp_e_vy: 0.,
            lp_s_vy: 0.,
            //DISTANCE
            //High pass filter for the distance of the Y-axis
            hp_dy: HPFilter::new(hp_fc),
            hp_e_dy: 0.,
            old_hp_e_dy: 0.,
            hp_s_dy: 0.,
            //Low pass filter for the distance of the Y-axis
            lp_dy: LPFilter::new(lp_fc),
            lp_e_dy: 0.,
            lp_s_dy: 0.,
            //MOVING AVERAGE
            //Moving average filter of the Y-axis
            ma_y: Smooth::new(num_readings),
            average_y: 0.,
            offset_y: 0.,
            //Other variables
            counter: 0,
            ready_2_go: false,
            final_distance: 0.,
        }
    }

    /// ### Initialize the device. 
    /// ***You must provide a Delay interface that implements embedded-hal::delay::DelayMs.***
    /// - Collect the first raw data from the device.
    /// - Initialize the Kalman filter.
    /// - The distance filters have been initialized with the default values within the creation of the device.
    /// ***Usage:***
    /// ```no_run
    /// use gy521::{self as _, Gy521};
    /// use rppal::hal::Delay;
    /// use embedded_hal::blocking::delay::*;
    /// // Initialize the sensor
    /// gy.init(&mut delay).unwrap();
    /// ```
    pub fn init<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), Error<E>> {

        // Initialization
        self.mpu.init(delay).unwrap();
        self.mpu.set_accel_range(mpu6050::device::AccelRange::G8).unwrap();
        self.mpu.set_gyro_range(mpu6050::device::GyroRange::D500).unwrap();

        // Collect data
        let _res = self.read_data();

        // Initialize the Kalman filter
        self.roll = self.get_roll();
        self.pitch = self.get_pitch();
        self.yaw = self.get_yaw();

        self.kalman_x.set_angle(self.roll);
        self.kalman_y.set_angle(self.pitch);
        self.kalman_z.set_angle(self.yaw);

        self.offset_kalman = self.yaw;

        Ok(())
    }

    /// ### Performs a step of all filters.
    /// ***You must provide your own delta time in seconds.*** The more accurate the delta time, the more accurate the results.
    /// - Starts by updating the raw data.
    /// - Then, it updates the Kalman filters on all axis.
    /// - Then, it updates the speed and distance filters on all axis.
    /// - Finally, it updates the moving average filter on all axis.
    /// 
    /// ***Usage:***
    /// ```no_run
    /// // Init a delay used in certain functions and between each loop.
    /// let mut delay = Delay::new();
    /// // Setting up the delta time within a std environment
    /// let mut time = Instant::now();
    /// loop {
    ///     // Calculate delta time in seconds
    ///    let dt = time.elapsed().as_micros() as f32 / 1_000_000.;
    ///    time = Instant::now();
    /// 
    ///     // Perform a step of the algorithm
    ///     gy.step(dt);
    /// 
    ///     delay.delay_us(10_u32);
    /// }
    /// ```
    /// ***You can get all the results by calling the get_\* functions.***
    pub fn step(&mut self, dt: f32) {
        // Read data from sensor
        let _res = self.read_data();

        // KALMAN
        // Calculate roll, pitch and yaw
        self.roll = self.get_roll();
        self.pitch = self.get_pitch();
        self.yaw = self.get_yaw();

        // Calculate the angular speed of the x, y, z axes
        let mut gyro_x_rate = self.gyro_x * RAD2DEG;
        let gyro_y_rate = self.gyro_y * RAD2DEG;
        let gyro_z_rate = self.gyro_z * RAD2DEG;

        // Restrict Pitch
        // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
        if ((self.pitch < -90.0) && (self.kalman_y_angle > 90.0)) || ((self.pitch > 90.0) && (self.kalman_y_angle < -90.0)) {
            self.kalman_y.set_angle(self.pitch);
            self.kalman_y_angle = self.pitch;
        } else {
            self.kalman_y.compute_angle(self.pitch, gyro_y_rate, dt);
            self.kalman_y_angle = self.kalman_y.get_angle(); 
        }

        // Invert rate, so it fits the restricted accelerometer reading
        if self.kalman_y_angle.abs() > 90.0 {
            gyro_x_rate = -gyro_x_rate;
        }

        // Update angles for x and z axis
        self.kalman_x.compute_angle(self.roll, gyro_x_rate, dt);
        self.kalman_x_angle = self.kalman_x.get_angle();
        self.kalman_z.compute_angle(self.yaw, gyro_z_rate, dt);
        self.kalman_z_angle = self.kalman_z.get_angle();

        let angle_z = self.kalman_z_angle - self.offset_kalman;

        // Tries to correct the drift of the angle
        // Acts like a buffer zone
        if angle_z < -BUFFER_ZONE_KALMAN {
            self.sens = -1;
        } else if angle_z > BUFFER_ZONE_KALMAN {
            self.sens = 1;
        } else {
            self.sens = 0;
        }

        // Counts the number of times the angle has changed
        if (self.sens != self.sens_prec) && (self.sens != 0) {
            self.sens_count += 1;
            self.sens_prec = self.sens;
        }

        // Movement is done when the angle has changed more than 3 times
        if self.sens_count > 3 {
            self.sens_count = 0;
            self.sens_prec = 0;
        } else {
            self.total_z_angle += self.compute_integral(angle_z, dt) * SQRT_G_KALMAN;
        }


        // DISTANCE
        // Offset management
        if self.counter == OFFSET_COUNTER {
            self.offset_x = self.average_x;
            self.offset_y = self.average_y;
            self.ready_2_go = true;
            self.distance_x = 0.;
            self.distance_y = 0.;
        }
        if self.counter < OFFSET_COUNTER + 1 {
            self.counter += 1;
        }

        //Scale acceleration
        let accel_x = self.acc_x * G_TO_SCALE;
        let accel_y = self.acc_y * G_TO_SCALE;

        // X-AXIS
        // We update the old value of the speed HP filter
        self.old_hp_e_vx = self.hp_e_vx;

        // HP filter for the speed of the X-axis
        self.hp_e_vx = accel_x;
        self.hp_s_vx = self.hp_vx.compute(self.hp_e_vx, self.old_hp_e_vx, self.hp_s_vx, dt);

        // LP filter for the speed of the X-axis
        self.lp_e_vx = self.hp_s_vx;
        self.lp_s_vx = self.lp_vx.compute(self.lp_e_vx, self.lp_s_vx, dt);

        // We compute VX
        self.speed_x = self.compute_integral(self.lp_s_vx, dt);

        // we update the old value of the distance HP filter
        self.old_hp_e_dx = self.hp_e_dx;

        // HP filter for the distance of the X-axis
        self.hp_e_dx = self.speed_x;
        self.hp_s_dx = self.hp_dx.compute(self.hp_e_dx, self.old_hp_e_dx, self.hp_s_dx, dt);

        // LP filter for the distance of the X-axis
        self.lp_e_dx = self.hp_s_dx;
        self.lp_s_dx = self.lp_dx.compute(self.lp_e_dx, self.lp_s_dx, dt);

        // Y-AXIS
        // We update the old value of the speed HP filter
        self.old_hp_e_vy = self.hp_e_vy;

        // HP filter for the speed of the Y-axis
        self.hp_e_vy = accel_y;
        self.hp_s_vy = self.hp_vy.compute(self.hp_e_vy, self.old_hp_e_vy, self.hp_s_vy, dt);

        // LP filter for the speed of the Y-axis
        self.lp_e_vy = self.hp_s_vy;
        self.lp_s_vy = self.lp_vy.compute(self.lp_e_vy, self.lp_s_vy, dt);

        // We compute VY
        self.speed_y = self.compute_integral(self.lp_s_vy, dt);

        // we update the old value of the distance HP filter
        self.old_hp_e_dy = self.hp_e_dy;

        // HP filter for the distance of the Y-axis
        self.hp_e_dy = self.speed_y;
        self.hp_s_dy = self.hp_dy.compute(self.hp_e_dy, self.old_hp_e_dy, self.hp_s_dy, dt);

        // LP filter for the distance of the Y-axis
        self.lp_e_dy = self.hp_s_dy;
        self.lp_s_dy = self.lp_dy.compute(self.lp_e_dy, self.lp_s_dy, dt);

        // Smoothing the curves
        self.ma_x.add_reading(self.lp_s_dx);
        self.average_x = self.ma_x.get_average() * SCALE_FACTOR - self.offset_x;
        self.ma_y.add_reading(self.lp_s_dy);
        self.average_y = self.ma_y.get_average() * SCALE_FACTOR - self.offset_y;

        // We compute the distance on the X-axis
        if (self.average_x.abs() > BUFFER_ZONE_X) && self.ready_2_go {
            self.distance_x += self.compute_integral(self.average_x, dt) * CORRECTION_FACTOR/SCALE_FACTOR;
        }

        // We compute the distance on the Y-axis
        if (self.average_y.abs() > BUFFER_ZONE_Y) && self.ready_2_go {
            self.distance_y += self.compute_integral(self.average_y, dt) * CORRECTION_FACTOR/SCALE_FACTOR;
        }

        // We compute the final distance
        self.final_distance = self.distance(self.distance_x, self.distance_y);
    }

    /// #### Read raw data from the device.
    /// Acceleration in g. 
    /// 
    /// Use ***G_TO_SCALE*** to convert to *cm/s²*.
    /// 
    /// Gyroscope data in *rad/s*.
    /// 
    /// You don't need to call this function before calling the step function.
    pub fn read_data(&mut self) -> Result<(), Error<E>> {
        let acc = self.mpu.get_acc().unwrap();
        self.acc_x = acc.x;
        self.acc_y = acc.y;
        self.acc_z = acc.z;
        let gyro = self.mpu.get_gyro().unwrap();
        self.gyro_x = gyro.x;
        self.gyro_y = gyro.y;
        self.gyro_z = gyro.z;

        Ok(())
    }

    /// Returns raw acceleration data within a Vector3<f32> object.
    pub fn get_accel(&self) -> Vector3<f32> {
        Vector3::new(self.acc_x, self.acc_y, self.acc_z)
    }

    /// Returns raw acceleration data within a Vector3<f64> object.
    pub fn get_accel_f64(&self) -> Vector3<f64> {
        Vector3::new(self.acc_x as f64, self.acc_y as f64, self.acc_z as f64)
    }

    /// Returns raw acceleration on the x-axis
    pub fn get_accel_x(&self) -> f32 {
        self.acc_x
    }

    /// Returns raw acceleration on the y-axis
    pub fn get_accel_y(&self) -> f32 {
        self.acc_y
    }

    /// Returns raw acceleration on the z-axis
    pub fn get_accel_z(&self) -> f32 {
        self.acc_z
    }

    /// Returns raw gyroscope data within a Vector3<f32> object.
    pub fn get_gyro(&self) -> Vector3<f32> {
        Vector3::new(self.gyro_x, self.gyro_y, self.gyro_z)
    }

    /// Returns raw gyroscope data within a Vector3<f64> object.
    pub fn get_gyro_f64(&self) -> Vector3<f64> {
        Vector3::new(self.gyro_x as f64, self.gyro_y as f64, self.gyro_z as f64)
    }

    /// Returns raw gyroscope data on the x-axis
    pub fn get_gyro_x(&self) -> f32 {
        self.gyro_x
    }

    /// Returns raw gyroscope data on the y-axis
    pub fn get_gyro_y(&self) -> f32 {
        self.gyro_y
    }

    /// Returns raw gyroscope data on the z-axis
    pub fn get_gyro_z(&self) -> f32 {
        self.gyro_z
    }

    /// Computes the integral of the angular velocity.
    /// Returns the integral in degrees.
    /// Angle should be in degrees/second.
    /// dt is the time in seconds since the last call to this function.
    fn compute_integral(&self, value: f32, dt: f32) -> f32 {
        value * dt
    }

    /// Computes the distance
    fn distance(&self, x: f32, y: f32) -> f32 {
        (x * x + y * y).sqrt()
    }

    /// #### Computes roll from accelerometer data.
    /// Returns the roll in degrees.
    pub fn get_roll(&self) -> f32 {
        self.acc_x.atan2(self.acc_z) * RAD2DEG
    }

    /// #### Computes pitch from accelerometer data.
    /// Returns the pitch in degrees.
    pub fn get_pitch(&self) -> f32 {
        (-self.acc_x / (self.acc_y * self.acc_y + self.acc_z * self.acc_z).sqrt()).atan() * RAD2DEG
    }

    /// #### Computes yaw from accelerometer data.
    /// Returns the yaw in degrees.
    pub fn get_yaw(&self) -> f32 {
        (self.acc_z / (self.acc_x * self.acc_x + self.acc_z * self.acc_z).sqrt()).atan() * RAD2DEG
    }

    /// #### Set the measured distance to 0.
    /// Acts like an artificial reset
    pub fn reset_distance(&mut self) {
        self.distance_x = 0.;
        self.distance_y = 0.;
        self.final_distance = 0.;
    }

    /// #### Set the measured angle to 0.
    /// Acts like an artificial reset
    pub fn reset_angle_z(&mut self) {
        self.total_z_angle = 0.;
    }

    /// #### Set the distance offsets to the current state of the system. 
    /// 
    /// ***Warning: you can't go back to the previous state once changed.***
    /// 
    /// Usually used when auto-resetting the distance variables. 
    /// Must be called when the sensor does NOT move. Otherwise, next data won't be accurate.
    pub fn reset_offset_distance(&mut self) {
        self.offset_x = self.average_x;
        self.offset_y = self.average_y;
    }

    /// #### Set the Kalman offsets to the current state of the system. 
    /// 
    /// ***Warning: you can't go back to the previous state once changed.***
    /// 
    /// Usually used when auto-resetting the angle variables.  
    /// Must be called when the sensor does ***NOT*** move. Otherwise, next data won't be accurate.
    pub fn reset_offset_angle(&mut self) {
        self.offset_kalman = self.yaw;
    }

    /// #### Get the Final X-Axis angle in *degrees*.
    /// Use DEG2RAD const to convert to *rad*.
    pub fn get_x_angle(&self) -> f32 {
        self.kalman_x_angle
    }

    /// #### Get the Final Y-Axis angle in *degrees*.
    /// Use DEG2RAD const to convert to *rad*.
    pub fn get_y_angle(&self) -> f32 {
        self.kalman_y_angle
    }

    /// #### Get the Final Z-Axis angle in *degrees*.
    /// Use DEG2RAD const to convert to *rad*.
    pub fn get_z_angle(&self) -> f32 {
        self.total_z_angle
    }

    /// #### Get the Z-axis angle velocity in *degrees/s*.
    /// Use DEG2RAD const to convert to *rad/s*.
    pub fn get_z_angular_velocity(&self) -> f32 {
        self.kalman_z_angle - self.offset_kalman
    }

    /// Get the current speed on the X-axis in *cm/s*.
    pub fn get_x_speed(&self) -> f32 {
        self.speed_x
    }

    /// Get the current speed on the Y-axis in *cm/s*.
    pub fn get_y_speed(&self) -> f32 {
        self.speed_y
    }

    /// Get the current distance traveled on the X-axis in *cm*.
    pub fn get_x_distance(&self) -> f32 {
        self.distance_x
    }

    /// Get the current distance traveled on the Y-axis in *cm*.
    pub fn get_y_distance(&self) -> f32 {
        self.distance_y
    }

    /// Get the current final distance traveled in *cm*.
    pub fn get_final_distance(&self) -> f32 {
        self.final_distance
    }

    /// #### Reset the device.  
    /// This function may be called when periodically when data starts to deviate from the normal behavior.
    pub fn reset_device<D: DelayMs<u8>>(&mut self, delay: &mut D) -> Result<(), Error<E>> {
        self.mpu.reset_device(delay).unwrap();
        // To test
        Ok(())
    }
}
