#![no_std]
#![no_main]

use gy521::{self as _, filters};
use filters::{Smooth, HPFilter, LPFilter};
use core::cell::RefCell;
use cortex_m::{
    delay::Delay,
    interrupt::{free, Mutex},
    peripheral::NVIC,
};
use mpu6050::*;

use stm32_hal2::{
    self,
    clocks::{Clocks, Pllp},
    gpio::{Pin, PinMode, Port, OutputType},
    pac::{self, interrupt}, 
    i2c::I2c, i2c_f4::I2cDevice,
    prelude::*,
    timer::{Timer, TimerConfig, TimerInterrupt}, 
};

use defmt_rtt as _;
use micromath::F32Ext;

// Used by the timer
make_globals!((MICROS, u64), (RESET, bool));

/// Reset the timer count to zero during the next interrupt.
fn reset_timer_count() {
    free(|cs| {
        access_global!(RESET, reset, cs);
        *reset = true;
    });
}

/// Do NOT forget to reset the timer count after using this function.
/// Returns the time elapsed since the last reset in u64 as Microseconds.
fn get_dt() -> u64 {
    let mut res:u64 = 0;
    free(|cs| {
        access_global!(MICROS, micros, cs);
        res = *micros;
    });
    res
}

/// Computes the integral of the angular velocity.
/// Returns the integral in degrees.
/// Angle should be in degrees/second.
/// dt is the time in seconds since the last call to this function.
fn compute_integral(value: f32, dt: f32) -> f32 {
    value * dt
}

/// Computes the distance
fn distance(x: f32, y: f32) -> f32 {
    (x * x + y * y).sqrt()
}


#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::println!("Program started.");

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();
    
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

    //Const variables
    //const RAW2G: f32 = 1.0/16384.0; //peut etre pas utile
    const BUFFER_ZONE_X: f32 = 7.;
    const BUFFER_ZONE_Y: f32 = 4.;
    //const G_TO_CM2: f32 = 980.665;
    const G: f32 = 9.80665;
    const G2: f32 = G * G;
    let sqrt_g = 9.80665.sqrt();
    let correction_factor = G2/sqrt_g;
    const OFFSET_COUNTER: u16 = 250;
    const RESET_COUNTER: u16 = 500;
    const PRINT_ACC: bool = true;
    

    //Timer Variables
    let tim_cfg = TimerConfig {
        ..Default::default()
    };
    let freq = 100.;
    let mut timer = Timer::new_tim2(dp.TIM2, freq, tim_cfg, &clock);
    timer.enable_interrupt(TimerInterrupt::Update);
    timer.enable();

    let micros:u64 = 0;
    let reset = true;
    // Set up the global static variables so we can access them during interrupts.
    free(|cs| {
        MICROS.borrow(cs).replace(Some(micros));
        RESET.borrow(cs).replace(Some(reset));
    });

    // Unmask the interrupt line.
    unsafe {
        NVIC::unmask(pac::Interrupt::TIM2);
    }
    defmt::println!("Timer setup done.");


    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, clock.systick());
    defmt::println!("I2C Configuration...");
    // Setup the I2C peripheral.
    let mut scl = Pin::new(Port::B, 8, PinMode::Alt(4));
    scl.output_type(OutputType::OpenDrain);
    let mut sda = Pin::new(Port::B, 9, PinMode::Alt(4));
    sda.output_type(OutputType::OpenDrain);

    defmt::println!("\t I2C creation...");
    let i2c = I2c::new(dp.I2C1, I2cDevice::One, 100_000, &clock);
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

    // Read data from the MPU6050.
    let acc = mpu.get_acc().unwrap();
    let acc_x = acc.x;
    let acc_y = acc.y;

    
    //X-AXIS
    //SPEED
    //High pass filter for the speed of the X-axis
    let mut hp_vx = HPFilter::new(0.05);
    let mut hp_e_vx = acc_x;
    let mut old_hp_e_vx: f32;
    let mut hp_s_vx = hp_e_vx;

    //We define speed and distance variables
    let mut speed_x: f32;
    let mut distance_x = 0.;

    //Low pass filter for the speed of the X-axis
    let mut lp_vx = LPFilter::new(20.);
    let mut lp_e_vx: f32;
    let mut lp_s_vx = hp_s_vx;

    //DISTANCE
    //High pass filter for the distance of the X-axis
    let mut hp_dx = HPFilter::new(0.05);
    let mut hp_e_dx = acc_x;
    let mut old_hp_e_dx: f32;
    let mut hp_s_dx = acc_x;

    //Low pass filter for the distance of the X-axis
    let mut lp_dx = LPFilter::new(20.);
    let mut lp_e_dx: f32;
    let mut lp_s_dx = hp_s_dx;

    //MOVING AVERAGE
    //Moving average filter of the X-axis
    let mut ma_x = Smooth::new(50);
    let mut average_x = 0.0;


    //Y-AXIS
    //SPEED
    //High pass filter for the speed of the Y-axis
    let mut hp_vy = HPFilter::new(0.05);
    let mut hp_e_vy = acc_y;
    let mut old_hp_e_vy: f32;
    let mut hp_s_vy = acc_y;

    //We define speed and distance variables
    let mut speed_y: f32;
    let mut distance_y = 0.;
    let mut offset_x = 0.;

    //Low pass filter for the speed of the Y-axis
    let mut lp_vy = LPFilter::new(20.);
    let mut lp_e_vy: f32;
    let mut lp_s_vy = hp_s_vy;

    //DISTANCE
    //High pass filter for the distance of the Y-axis
    let mut hp_dy = HPFilter::new(0.05);
    let mut hp_e_dy = acc_y;
    let mut old_hp_e_dy: f32;
    let mut hp_s_dy = acc_y;

    //Low pass filter for the distance of the Y-axis
    let mut lp_dy = LPFilter::new(20.);
    let mut lp_e_dy: f32;
    let mut lp_s_dy = hp_s_dy;

    //MOVING AVERAGE
    //Moving average filter of the Y-axis
    let mut ma_y = Smooth::new(50);
    let mut average_y = 0.0;
    let mut offset_y = 0.;

    defmt::println!("Filters created and configured.");

    // Reset the count
    let mut old_dx = 0.;
    let mut old_dy = 0.;
    let mut counter_reset = 0;

    //Other variables
    let mut counter = 0;
    let mut ready_2_go = false;
    let mut final_distance: f32;
    
    
    defmt::println!("Here we go!");
    delay.delay_ms(2500);
    reset_timer_count();
    
    loop {

        // Offset management
        if counter == OFFSET_COUNTER {
            offset_x = average_x;
            offset_y = average_y;
            ready_2_go = true;
            distance_x = 0.;
            distance_y = 0.;
            defmt::println!("Offset set.");
        }
        if counter < OFFSET_COUNTER + 1 {
            counter += 1;
        }

        // Read data from sensor
        let acc = mpu.get_acc().unwrap();
        let acc_x = acc.x * 960.665;
        let acc_y = acc.y * 960.665;
        
        //TODO: Refaire le bon calcul
        // Calculate delta time in seconds
        let mut dt = (get_dt()) as f32 / 1_000_000.0;
        dt = 15./1_000_000.;
        reset_timer_count();

        // X-AXIS
        // We update the old value of the speed HP filter
        old_hp_e_vx = hp_e_vx;

        // HP filter for the speed of the X-axis
        hp_e_vx = acc_x;
        hp_s_vx = hp_vx.compute(hp_e_vx, old_hp_e_vx, hp_s_vx, dt);

        // LP filter for the speed of the X-axis
        lp_e_vx = hp_s_vx;
        lp_s_vx = lp_vx.compute(lp_e_vx, lp_s_vx, dt);

        // We compute VX
        speed_x = compute_integral(lp_s_vx, dt);

        // we update the old value of the distance HP filter
        old_hp_e_dx = hp_e_dx;

        // HP filter for the distance of the X-axis
        hp_e_dx = speed_x;
        hp_s_dx = hp_dx.compute(hp_e_dx, old_hp_e_dx, hp_s_dx, dt);

        // LP filter for the distance of the X-axis
        lp_e_dx = hp_s_dx;
        lp_s_dx = lp_dx.compute(lp_e_dx, lp_s_dx, dt);


        // Y-AXIS
        // We update the old value of the speed HP filter
        old_hp_e_vy = hp_e_vy;

        // HP filter for the speed of the Y-axis
        hp_e_vy = acc_y;
        hp_s_vy = hp_vy.compute(hp_e_vy, old_hp_e_vy, hp_s_vy, dt);

        // LP filter for the speed of the Y-axis
        lp_e_vy = hp_s_vy;
        lp_s_vy = lp_vy.compute(lp_e_vy, lp_s_vy, dt);

        // We compute VY
        speed_y = compute_integral(lp_s_vy, dt);

        // we update the old value of the distance HP filter
        old_hp_e_dy = hp_e_dy;

        // HP filter for the distance of the Y-axis
        hp_e_dy = speed_y;
        hp_s_dy = hp_dy.compute(hp_e_dy, old_hp_e_dy, hp_s_dy, dt);

        // LP filter for the distance of the Y-axis
        lp_e_dy = hp_s_dy;
        lp_s_dy = lp_dy.compute(lp_e_dy, lp_s_dy, dt);


        // Smoothing the curves
        ma_x.add_reading(lp_s_dx);
        average_x = ma_x.get_average() * 100.0 - offset_x;
        ma_y.add_reading(lp_s_dy);
        average_y = ma_y.get_average() * 100.0 - offset_y;


        // We compute the distance on the X-axis
        if (average_x.abs() > BUFFER_ZONE_X) && ready_2_go {
            old_dx = distance_x;
            distance_x += compute_integral(average_x, dt) * correction_factor/100.;
        }

        // We compute the distance on the Y-axis
        if (average_y.abs() > BUFFER_ZONE_Y) && ready_2_go {
            old_dy = distance_y;
            distance_y += compute_integral(average_y, dt) * correction_factor/100.;
        }

        // Reset the distance measured if too much time has ellapsed withour change
        if ((old_dx - distance_x).abs() == 0. || (old_dy - distance_y).abs() == 0.) && ready_2_go {
            if counter_reset == RESET_COUNTER {
                counter_reset = 0;
                distance_x = 0.;
                distance_y = 0.;
                defmt::println!("Reset.");
            } else {
                counter_reset += 1;
            }
        } else {
            counter_reset = 0;
        }

        // We compute the final distance
        final_distance = distance(distance_x, distance_y);

        // We print the data
        if PRINT_ACC {
            defmt::println!("{:?}, {:?}", average_x.abs(), average_y.abs());
        } else {
            defmt::println!("{:?}, {:?}, {:?}", distance_x, distance_y, final_distance);
        }
        
        delay.delay_us(10);
    }
}

#[interrupt]
/// Timer interrupt handler; runs when the countdown period expires.
fn TIM2() {
    free(|cs| {
        // Clear the interrupt flag. If you ommit this, it will fire repeatedly.
        unsafe { (*pac::TIM2::ptr()).sr.modify(|_, w| w.uif().clear_bit()) }
        // If you have access to the timer variable, eg through a Mutex, you can do this instead:
        // countdown_timer_clear_interrupt(TimerInterrupt::Update);

        access_global!(RESET, reset, cs);
        access_global!(MICROS, micros, cs);
        if *reset == true{
            *reset = false;
            *micros = 0;
        } else {
            *micros += 1;
        }
    });
    
}
