//! This is a module used to represent a sensor.

#[derive(Debug)]
/// Structure used to represent a sensor.
pub struct Sensor<'a> {
    /// Sensor name.
    pub name: &'a str,
    /// Version of the hardware + driver.
    pub version: i32,
    /// Unique sensor identifier.
    pub sensor_id: i32,
    /// Sensor type.
    pub sensor_type: SensorType,
    /// Maximum value of this sensor's value in SI units.
    pub max_value: f32,
    /// Minimum value of this sensor's value in SI units.
    pub min_value: f32,
    /// Smallest difference between two values reported by this sensor.
    pub resolution: f32,
    /// Min delay in microseconds between events. zero = not a constant rate.
    pub min_delay: i32,
}

#[derive(Debug)]
/// Enumeration used to represent a sensor's type.
pub enum SensorType {
    /// Accelerometer sensor.
    Accelerometer = 1,
    /// Gyroscope sensor.
    MagneticField = 2,
    /// Gyroscope sensor.
    Gyroscope = 3,
}