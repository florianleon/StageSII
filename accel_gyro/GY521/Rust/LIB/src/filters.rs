#![allow(dead_code)]

use nalgebra::SVector;
use core::f32::consts::PI;

#[derive(Copy, Clone, Debug)]
/// This is a simple moving average filter.
pub struct Smooth {
    n: u8,
    readings: SVector<f32, 50>,
    read_index:u8,
    total: f32,
    average: f32,
}

impl Smooth {
    /// Create a new Smooth filter with the given number of readings.
    pub fn new(num_readings: u8) -> Self {
        Smooth { 
            n: num_readings, 
            readings: SVector::<f32, 50>::zeros(), 
            read_index: 0, 
            total: 0.0, 
            average: 0.0,
        }
    }

    /// Add a new reading to the filter.
    pub fn add_reading(&mut self, reading: f32) {
        let index = self.read_index as usize;
        self.total -= self.readings[index];
        self.readings[index] = reading;
        self.total += reading;
        self.read_index = (self.read_index + 1) % self.n;
        self.average = self.total / self.n as f32;
    }

    /// Get the current average reading.
    pub fn get_average(&self) -> f32 {
        self.average
    }
}

#[derive(Copy, Clone, Debug)]
/// High pass filter. Simply provide the cutoff frequency.
pub struct HPFilter {
    output: f32,
    tau: f32,
}

impl HPFilter {
    /// Create a new High Pass Filter with the given cutout frequency.
    pub fn new(fc: f32) -> Self {
        HPFilter { 
            output: 0.0, 
            tau : 1. / (2.0 * PI * fc),
        }
    }

    /// Computes the output of the filter.
    /// You shoud provide the input, the n-1 input, the n-1 output, and the time elapsed since the last call.
    pub fn compute(&mut self, input: f32, old_input: f32, old_output: f32, dt: f32) -> f32 {
        self.output = old_output + (input - old_input) - (dt / self.tau) * old_output;
        self.output
    } 

    /// Get the current output.
    pub fn get_output(&self) -> f32 {
        self.output
    }
}

#[derive(Copy, Clone, Debug)]
/// High pass filter. Simply provide the cutout frequency.
pub struct LPFilter {
    output: f32,
    tau: f32,
}

impl LPFilter {
    /// Create a new High Pass Filter with the given cutout frequency.
    pub fn new(fc: f32) -> Self {
        LPFilter { 
            output: 0.0, 
            tau : 1. / (2.0 * PI * fc),
        }
    }

    /// Computes the output of the filter.
    /// You shoud provide the input, the n-1 output, and the time elapsed since the last call.
    pub fn compute(&mut self, input: f32, old_output: f32, dt: f32) -> f32 {
        self.output = old_output + (input - old_output) * (dt / self.tau);
        self.output
    } 

    /// Get the current output.
    pub fn get_output(&self) -> f32 {
        self.output
    }
}