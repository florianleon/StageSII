#![no_main]
#![no_std]

pub mod time;
pub mod kalman;
pub mod filters;
pub mod systick;

use core::panic::PanicInfo;
use defmt_rtt as _; // global logger

// TODO(5) adjust HAL import
use stm32_hal2 as _;

//use panic_probe as _;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is invoked
//#[defmt::panic_handler]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    cortex_m::asm::udf()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

// defmt-test 0.3.0 has the limitation that this `#[tests]` attribute can only be used
// once within a crate. the module can be in any file but there can only be at most
// one `#[tests]` module in this library crate
#[cfg(test)]
#[defmt_test::tests]
mod unit_tests {
    use defmt::assert;

    #[test]
    fn it_works() {
        assert!(true)
    }
}
