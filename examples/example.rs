use std::error::Error;

use ltc2497::LTC2497;
use ltc2497::{AddressPinState::Low, Channel};
use rppal::{hal::Delay, i2c::I2c};

fn main() -> Result<(), Box<dyn Error>> {
    let mut i2c = I2c::new()?;

    let adc = LTC2497::new_from_pins(i2c, Low, Low, Low, Delay, 5.0, 0);

    for channel in Channel::SingleEndedCh0..=Channel::SingleEndedCh15 {
        let voltage = adc.read_channel(channel)?;
        println!("measured {}V at {}", voltage, channel);
    }

    Ok(())
}
