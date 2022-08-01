use std::thread::sleep;
use std::time::Duration;
use rppal::gpio::Gpio;

const GPIO_LED: u8 = 4; // -> pin 7
// Ne as oublier la résistance

fn main() -> ! {

    let mut pin = Gpio::new().unwrap()
                                        .get(GPIO_LED)
                                        .unwrap()
                                        .into_output();

    loop {
        pin.toggle();
        sleep(Duration::from_millis(500));
    }

}