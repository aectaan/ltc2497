use std::error::Error;

use ltc2497::LTC2497;
use ltc2497::{AddressPinState::Low, Channel};
use rppal::hal::Delay;
use rppal::i2c::I2c;

fn main() -> Result<(), Box<dyn Error>> {
    let i2c = I2c::new()?;

    let mut adc = LTC2497::new_from_pins(i2c, Low, Low, Low, 5.0, 0.0, Delay);

    for ch in 0..=15 {
        match adc.read_channel(Channel::from(ch)) {
            Ok(v) => println!("Channel {ch}: {v}V"),
            Err(e) => println!("error {e:?} at channel {ch}"),
        };
    }

    Ok(())
}
