#![no_std]
#![no_main]

use gy521::{self as _, time::Instant};

use cortex_m::{
    delay::Delay,
};

use stm32_hal2::{
    self,
    clocks::{Clocks, Pllp},
    gpio::{Pin, PinMode, Port},
    pac,
};


use defmt_rtt as _;


#[cortex_m_rt::entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let _dp = pac::Peripherals::take().unwrap();
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

    let inst1 = Instant::new(0).unwrap();
    
    defmt::println!("Here we go!");
    let mut i = 0;
    // Now, enjoy the lightshow!
    loop {
        i+=1;
        let inst2 = Instant::new(i).unwrap();
        let inst = inst2.duration_since(inst1);
        defmt::println!("{}", inst.value());
        led.set_low();
        delay.delay_ms(500_u32);
        led.set_high();
        delay.delay_ms(500_u32);
        
        //defmt::println!("{}", res);
    }
}

