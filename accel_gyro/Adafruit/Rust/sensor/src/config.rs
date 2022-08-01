//! This is a part of our driver use to place all configuration structures, enumerations and constants in one place.

/// ### 7-bit I2C address for this sensor.
/// Device ID for this sensor (used as sanity check during init).
/// 0b1100 0111
pub const ACCEL_MAG_ID: u8 = 0xC7;
/// 7_bit address for this sensor.
pub const ACCEL_MAG_ADDR: u8 = 0x1f;
/// Macro for mg per LSB at +/- 2g sensitivity (1 LSB = 0.000244mg)
pub const ACCEL_MG_LSB_2G: f32 = 1.0 / 4096.0;
/// Macro for mg per LSB at +/- 4g sensitivity (1 LSB = 0.000488mg)
pub const ACCEL_MG_LSB_4G: f32 = 1.0 / 2048.0;
/// Macro for mg per LSB at +/- 8g sensitivity (1 LSB = 0.000976mg)
pub const ACCEL_MG_LSB_8G: f32 = 1.0 / 1024.0;
/// Macro for micro tesla (uT) per LSB (1 LSB = 0.1uT)
pub const MAG_UT_LSB: f32 = 0.1;

/// ### 7-bit I2C address for this sensor.
/// Device ID for this sensor (used as sanity check during init).
/// 0b1100 0111
pub const GYRO_ID: u8 = 0xD7;
/// 7_bit address for this sensor.
pub const GYRO_ADDR: u8 = 0x21;
/// Gyroscope sensitivity at 250dps
pub const GYRO_SENSITIVITY_250DPS: f32 = 0.0078125;
/// Gyroscope sensitivity at 500dps
pub const GYRO_SENSITIVITY_500DPS: f32 = 0.015625;
/// Gyroscope sensitivity at 1000dps
pub const GYRO_SENSITIVITY_1000DPS: f32 = 0.03125;
/// Gyroscope sensitivity at 2000dps
pub const GYRO_SENSITIVITY_2000DPS: f32 = 0.0625;


#[derive(Debug, PartialEq, Clone, Copy)]
/// Raw register addresses used to communicate with the AccelMag sensor.
pub enum AccelMagRegisters {
    /// Real-time data-ready status.
    Status = 0x00,
    /// 8 MSBs of 14-bit sample for the X_axis.
    OutXMSB = 0x01,
    /// 6 LSBs of 14-bit sample for the X_axis.
    OutXLSB = 0x02,
    /// 8 MSBs of 14-bit sample for the Y_axis.
    OutYMSB = 0x03,
    /// 6 LSBs of 14-bit sample for the Y_axis.
    OutYLSB = 0x04,
    /// 8 MSBs of 14-bit sample for the Z_axis.
    OutZMSB = 0x05,
    /// 6 LSBs of 14-bit sample for the Z_axis.
    OutZLSB = 0x06,
    /// Current system mode.
    Sysmod = 0x0B,
    /// Device ID.
    WhoAmI = 0x0D,
    /// Acceleration dynamic range and filter enable settings.
    XyzDataCFG = 0x0E,
    /// System ODR, accelerometer OSR, operating mode settings.
    CtrlReg1 = 0x2A,
    /// Self-test, reset, accelerometer OSR and sleep mode settings.
    CtrlReg2 = 0x2B,
    /// Sleep mode interrupt wake enable, interrupt polarity, push-pull/open-drain configuration settings.
    CtrlReg3 = 0x2C,
    /// Interrupt enable register.
    CtrlReg4 = 0x2D,
    /// Interrupt pin (INT1/INT2) map
    CtrlReg5 = 0x2E,
    /// Magnetic data ready status.
    Mstatus = 0x32,
    /// MSB of 16-bit magnetic data for X-axis.
    MoutXMSB = 0x33,
    /// LSB of 16-bit magnetic data for X-axis.
    MoutXLSB = 0x34,
    /// MSB of 16-bit magnetic data for Y-axis.
    MoutYMSB = 0x35,
    /// LSB of 16-bit magnetic data for Y-axis.
    MoutYLSB = 0x36,
    /// MSB of 16-bit magnetic data for Z-axis.
    MoutZMSB = 0x37,
    /// LSB of 16-bit magnetic data for Z-axis.
    MoutZLSB = 0x38,
    /// Control for magnetic sensor functions.
    MctrlReg1 = 0x5B,
    /// Control for magnetic sensor functions.
    MctrlReg2 = 0x5C,
    /// Control for magnetic sensor functions.
    MctrlReg3 = 0x5D,
}

/// Raw register addresses used to communicate with the Gyro sensor.
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum GyroRegisters {
    /// Real-time data-ready status.
    Status = 0x00,
    /// 8 MSBs of 16-bit sample for the X_axis.
    OutXMSB = 0x01,
    /// 8 LSBs of 16-bit sample for the X_axis.
    OutXLSB = 0x02,
    /// 8 MSBs of 16-bit sample for the Y_axis.
    OutYMSB = 0x03,
    /// 8 LSBs of 16-bit sample for the Y_axis.
    OutYLSB = 0x04,
    /// 8 MSBs of 16-bit sample for the Z_axis.
    OutZMSB = 0x05,
    /// 8 LSBs of 16-bit sample for the Z_axis.
    OutZLSB = 0x06,
    /// Device ID.
    WhoAmI = 0x0C,
    /// Full-scale range selection, high-pass filter setting, SPI mode selection.
    CtrlReg0 = 0x0D,
    /// Operating mode, ODR selection, self-test and soft reset.
    CtrlReg1 = 0x13,
    ///  Interrupt configuration settings.
    CtrlReg2 = 0x14,
}

#[derive(Debug, PartialEq, Clone, Copy)]
/// #### System status for the overall AccelMag sensor. 
/// Gets the current mode as the 2bit sysmod\[1:0\] setting to ensure proper mode configuration.
/// Default to Standby mode.
pub enum AccelMagSystemStatus {
    /// Standby mode.
    Standby = 0b00,
    /// Wake mode.
    WakeUp = 0b01,
    /// Sleep mode.
    Sleep = 0b10,
}

#[derive(Debug, PartialEq, Clone, Copy)]
/// #### Sensor mode settings for the overall AccelMag sensor. 
/// Sets the sensor in accelerometer-only, magnetometer-only, or hybrid modes. 
/// Sent to AccelMag_REGISTER_MCTRL_REG1
pub enum AccelMagSensorMode {
    /// Only accelerometer sensor is active.
    AccelOnlyMode = 0b00,
    /// Only magnetometer sensor is active.
    MagOnlyMode = 0b01,
    /// Hybrid mode, both accelerometer and magnetometer sensors are active. 
    /// When operating in hybrid mode, the effective ODR for each sensor is half of the frequency selected in the CTRL_REG1\[dr\] and CTRL_REG1\[aslp_rate\] bit fields.
    HybridMode = 0b11,
}

#[derive(Debug, PartialEq, Clone, Copy)]
/// #### Output Data Rate (ODR) key for the overall AccelMag sensor. 
/// Called by user for convenient variable name matching.
pub enum AccelMagODR {
    /// 800Hz, only available in accel/mag-only modes
    ODR800HZ = 0x00,
    /// 400Hz, available in all modes
    ODR400HZ = 0x01,
    /// 200Hz, available in all modes
    ODR200HZ = 0x02,
    /// 100Hz, available in all modes
    ODR100HZ = 0x03,
    /// 50Hz, available in all modes
    ODR50HZ = 0x04,
    /// 25Hz, only available in hybrid mode
    ODR25HZ = 0x05,
    /// 12.5Hz, only available in accel/mag-only modes
    ODR12_5HZ = 0x06,
    /// 6.25Hz, available in all modes
    ODR6_25HZ = 0x07,
    /// 3.125Hz, only available in hybrid mode
    ODR3_125HZ = 0x08,
    /// 3.125Hz, only available in accel/mag-only modes
    ODR1_5625HZ = 0x09,
    /// 0.7813Hz, only available in hybrid mode
    ODR0_7813HZ = 0x0A,
}

#[derive(Debug, PartialEq, Clone, Copy)]
/// #### Output Data Rate (ODR) key for the overall Gyro sensor.
/// Called by user for convenient variable name matching.
pub enum GyroODR {
    /// 800Hz
    ODR800HZ = 0x00,
    /// 400Hz
    ODR400HZ = 0x01,
    /// 200Hz
    ODR200HZ = 0x02,
    /// 100Hz
    ODR100HZ = 0x03,
    /// 50Hz
    ODR50HZ = 0x04,
    /// 25Hz
    ODR25HZ = 0x05,
    /// 12.5Hz
    ODR12_5HZ = 0x06,
    /// 6.25Hz
    ODR6_25HZ = 0x07,
}

/// Output Data Rates (ODRs) available for ***accel/mag-only*** modes, array of user AccelMagODR.
pub const ACCEL_MAG_ONLY_AVAILABLE_ODR: [AccelMagODR; 8] = [
    AccelMagODR::ODR800HZ,
    AccelMagODR::ODR400HZ,
    AccelMagODR::ODR200HZ,
    AccelMagODR::ODR100HZ,
    AccelMagODR::ODR50HZ,
    AccelMagODR::ODR12_5HZ,
    AccelMagODR::ODR6_25HZ,
    AccelMagODR::ODR1_5625HZ,
];

/// Output Data Rates (ODRs) available for ***hybrid*** modes, array of user AccelMagODR.
pub const HYBRID_AVAILABLE_ODR: [AccelMagODR; 8] = [
    AccelMagODR::ODR400HZ, 
    AccelMagODR::ODR200HZ,
    AccelMagODR::ODR100HZ,
    AccelMagODR::ODR50HZ,
    AccelMagODR::ODR25HZ,
    AccelMagODR::ODR6_25HZ,
    AccelMagODR::ODR3_125HZ,
    AccelMagODR::ODR0_7813HZ,
];

/// Output Data Rate (ODR) settings to write to the dr\[2:0\] bits in
/// CTRL_REG_1, array type to pass the index from available accel/mag-only
/// and hybrid modes.
pub const ACCEL_MAG_ODR_DR_BITS:[u8; 8] = [
    0x00, // dr=0b000. 800Hz accel/mag-only modes, 400Hz hybrid mode.
    0x08, // dr=0b001. 400Hz accel/mag-only modes, 200Hz hybrid mode.
    0x10, // dr=0b010. 200Hz accel/mag-only modes, 100Hz hybrid mode.
    0x18, // dr=0b011. 100Hz accel/mag-only modes, 50Hz hybrid mode.
    0x20, // dr=0b100. 50Hz accel/mag-only modes, 25Hz hybrid mode.
    0x28, // dr=0b101. 12.5Hz accel/mag-only modes, 6.25Hz hybrid mode.
    0x30, // dr=0b110. 6.25Hz accel/mag-only modes, 3.125Hz hybrid mode.
    0x38, // dr=0b111. 1.5625Hz accel/mag-only modes, 0.7813Hz hybrid mode.
];

/// Output Data Rate (ODR) settings to write to the dr\[2:0\] bits in CTRL_REG_1.
pub const GYRO_ODR_DR_BITS:[u8; 8] = [
    0x00, // dr=0b000. 800Hz.
    0x04, // dr=0b001. 400Hz.
    0x08, // dr=0b010. 200Hz.
    0x0C, // dr=0b011. 100Hz.
    0x10, // dr=0b100. 50Hz.
    0x14, // dr=0b101. 25Hz.
    0x18, // dr=0b110. 12.5Hz.
    0x1C, // dr=0b111. 6.25Hz.
];

#[derive(Debug, PartialEq, Clone, Copy)]
/// #### Range settings for the accelerometer sensor.
pub enum AccelMagRange {
    /// +/- 2g range.
    Range2g = 0x00,
    /// +/- 4g range.
    Range4g = 0x01,
    /// +/- 8g range.
    Range8g = 0x02,
}

#[derive(Debug, PartialEq, Clone, Copy)]
/// Accelerometer sleep mode OSR mode selection. 
/// This setting, along with the CTRL_REG1[aslp_rate] ODR setting determines the sleep mode power and noise for acceleration measurements.
pub enum AccelMagOSRMode {
    /// Normal mode.
    Normal = 0b00,
    /// Low noise, low power mode.
    LowNoiseLowPower = 0b01,
    /// High resolution mode.
    HighResolution = 0b10,
    /// Low power mode.
    LowPower = 0b11,
}

#[derive(Debug, PartialEq, Clone, Copy)]
/// #### Enum to define valid gyroscope range values.
pub enum GyroRange {
    /// 250 dps.
    Range250dps = 250,
    /// 500 dps.
    Range500dps = 500,
    /// 1000 dps.
    Range1000dps = 1000,
    /// 2000 dps.
    Range2000dps = 2000,
}

#[derive(Debug, PartialEq, Clone, Copy)]
/// #### Range settings for the magnetometer sensor.
pub enum AccelMagMagOSR {
    /// Mag oversampling ratio = 0
    MagOSR0 = 0x00,
    /// Mag oversampling ratio = 1
    MagOSR1 = 0x01,
    /// Mag oversampling ratio = 2
    MagOSR2 = 0x02,
    /// Mag oversampling ratio = 3
    MagOSR3 = 0x03,
    /// Mag oversampling ratio = 4
    MagOSR4 = 0x04,
    /// Mag oversampling ratio = 5
    MagOSR5 = 0x05,
    /// Mag oversampling ratio = 6
    MagOSR6 = 0x06,
    /// Mag oversampling ratio = 7
    MagOSR7 = 0x07,
}

#[derive(Debug, PartialEq, Clone, Copy)]
/// Raw (integer) values from a 3dof sensor.
pub struct Data {
    /// Raw i16 value from the x axis.
    pub x: i16,
    /// Raw i16 value from the y axis.
    pub y: i16,
    /// Raw i16 value from the z axis.
    pub z: i16,
}

#[derive(Debug, PartialEq, Clone, Copy)]
/// Scaled (f32) values from a 3dof sensor.
pub struct ScaledData {
    /// Raw f32 value from the x axis.
    pub x: f32,
    /// Raw f32 value from the y axis.
    pub y: f32,
    /// Raw f32 value from the z axis.
    pub z: f32,
}
