use std::error::Error;

use linux_embedded_hal::{Delay, I2cdev};
use ltc2497::LTC2497;
use ltc2497::{AddressPinState::Low, Channel};

fn main() -> Result<(), Box<dyn Error>> {
    let i2c = I2cdev::new("/dev/i2c-1").unwrap();

    let mut adc = LTC2497::new_from_pins(i2c, Low, Low, Low, 5.0, 0.0, Delay);

    for ch in 0..=15 {
        match adc.read_channel(Channel::from(ch)) {
            Ok(v) => println!("Channel {ch}: {v}V"),
            Err(e) => println!("error {e:?} at channel {ch}"),
        };
    }

    Ok(())
}
