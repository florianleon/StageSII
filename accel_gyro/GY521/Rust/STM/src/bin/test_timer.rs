#![no_std]
#![no_main]

use gy521 as _;

use core::cell::RefCell;
use cortex_m::{
    delay::Delay,
    interrupt::{free, Mutex},
    peripheral::NVIC,
};

//use cortex_m_rt::interrupt;

use stm32_hal2::{
    self,
    clocks::{Clocks, Pllp},
    gpio::{Pin, PinMode, Port},
    pac::{self, interrupt}, 
    timer::{Timer, TimerConfig, TimerInterrupt}, 
    prelude::*,

};


use defmt_rtt as _;


make_globals!((MICROS, u64), (RESET, bool));


fn reset_timer_count() {
    free(|cs| {
        access_global!(RESET, reset, cs);
        *reset = true;
    });
}

//A toujours utiliser avant de reset le timer !
fn get_dt() -> u64 {
    let mut res:u64 = 0;
    free(|cs| {
        access_global!(MICROS, micros, cs);
        res = *micros;
    });
    res
}

#[cortex_m_rt::entry]
fn main() -> ! {
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
    
    // Setup a delay, based on the Cortex-m systick.
    let mut delay = Delay::new(cp.SYST, clock.systick());
    let mut led = Pin::new(Port::A, 5, PinMode::Output);
    
    let tim_cfg = TimerConfig {
        auto_reload_preload: true,
        ..Default::default()
    };
    let freq = 1_000.;
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
        defmt::println!("Setting up NVIC");
        NVIC::unmask(pac::Interrupt::TIM2);
    }

    defmt::println!("Here we go!");
    reset_timer_count();
    timer.reset_countdown();
    // Now, enjoy the lightshow!
    loop {
        let execution_time = get_dt();
        reset_timer_count();
        timer.reset_countdown();
        defmt::println!("{}", execution_time);
        led.set_low();
        delay.delay_us(500_u32);
        led.set_high();
        delay.delay_us(500_u32);
        
        //defmt::println!("{}", res);
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


