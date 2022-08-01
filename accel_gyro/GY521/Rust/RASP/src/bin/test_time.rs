use std::time::Instant;

use rppal::gpio::Gpio;
use rppal::hal::Delay;

use embedded_hal::blocking::delay::*;

use rand::Rng;

//Dans les faits j'ai pas besoin de la led mais c'est pour l'effet visuel...
const GPIO_LED: u8 = 4; // -> pin 7
const TEST_MS:bool = false;

fn main() -> ! {

    let mut pin = Gpio::new().unwrap()
                                        .get(GPIO_LED)
                                        .unwrap()
                                        .into_output();

    let mut delay = Delay::new();
    let mut rng = rand::thread_rng();

    if TEST_MS {
        loop {
            let start = Instant::now();
            pin.toggle();
            let time: u32 = rng.gen_range(1..=10) * 100;
            std::println!("Rand time (ms): {}", time);
            delay.delay_ms(time);
            let diff = start.elapsed().as_millis();
            std::println!("Total time (ms): {}", diff);
            std::println!("-------------------------------------");
        }
    } else {
        loop {
            let start = Instant::now();
            pin.toggle();
            let time: u32 = rng.gen_range(1..=10) * 100;
            std::println!("Rand time (us): {}", time);
            delay.delay_us(time);
            let diff = start.elapsed().as_micros();
            std::println!("Total time (us): {}", diff);
            std::println!("-------------------------------------");
        }
    }

}