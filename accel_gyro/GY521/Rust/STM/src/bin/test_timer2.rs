#![no_std]
#![no_main]

use gy521::{self as _, systick};

use cortex_m;

use stm32_hal2::{
    self,
    clocks::{Clocks, Pllp},
};


use defmt_rtt as _;

/// Sleep for a number of microseconds.
fn delay_us(value: u32) {
    let mut start = systick::micros();
    let stop = start + value as u64;
    while start < stop {
        start = systick::micros();
    }
}

fn update_delay() -> u64 {
    let btw = systick::micros();
    systick::reset();
    btw
}


#[cortex_m_rt::entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    
    let clock = Clocks {
        pllp: Pllp::Div4,
        plln: 84,
        ..Default::default()
    };

    clock.setup().unwrap();
    

    systick::init_with_frequency(cp.SYST, 84_000_000, 1_000);
    defmt::println!("Init Done.");
    systick::start();
    defmt::println!("Syst count started.");


    loop {
        let start = systick::micros();
        delay_us(500);
        let btw = update_delay();
        delay_us(600);
        let end = systick::micros() + btw;
        defmt::println!("start: {}, end: {}, elapsed: {}", start, end, end - start);
        systick::reset();
    }
}

