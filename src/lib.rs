//! A platform agnostic Rust driver for the LTC2497 ADC, based on the [`embedded-hal`](https://github.com/rust-embedded/embedded-hal) traits.
//! Device can be used with embedded HAL directly or with RPPAL (Raspberry Pi Peripheral Access Library) with "hal" feature.
//!
//! ## Usage
//!
//! Import this crate and one of embedded HAL implementation (e.g. [linux-embedded-hal](https://crates.io/crates/linux-embedded-hal)
//! or [rppal](https://crates.io/crates/rppal)). Then instantiate the device.
//!
//! ```no_run
//! use std::error::Error;
//! use ltc2497::LTC2497;
//! use ltc2497::{AddressPinState::Low, Channel};
//! use rppal::hal::Delay;
//! use rppal::i2c::I2c;
//!
//! # fn main() -> Result<(), Box<dyn Error>> {
//! let i2c = I2c::new()?;
//!
//! let mut adc = LTC2497::new_from_pins(i2c, Low, Low, Low, 5.0, 0.0, Delay);
//!
//! for ch in 0..=15 {
//!     match adc.read_channel(Channel::from(ch)) {
//!         Ok(v) => println!("Channel {ch}: {v}V"),
//!         Err(e) => println!("error {e:?} at channel {ch}"),
//!     };
//! }
//!
//! Ok(())
//! # }
//! ```
//!
//! Also you can instantiate device with address directly. For example, if you configured pins to set device address 0x20:
//! ```no_run
//! let mut adc = LTC2497::new(i2c, 0x20, 5.0, 0.0, Delay);
//! ```
//!

#![no_std]

use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Read, Write, WriteRead},
};

/// Maximum conversion duration by datasheet is claimed as 149.9ms. This delay is used to prevent I2C NACKs caused by trying to access registers during conversion.
const CONVERSION_DELAY: u8 = 160;

#[derive(Debug)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),
    /// Voltage is too high
    Overvoltage,
    /// Voltage is too low
    Undervoltage,
}

/// LTC2497 provides a possibility to make differential measurements between two adjacent channels. You can get voltage between channel 0 and channel 1,
/// between channel 2 and channel 3, but you can't measure voltage between channel 1 and channel 2 or between channel 4 and channel 8, because they belongs to different pairs.
/// Also you can make single-ended measurements (between any channel and GND).
#[derive(Debug, Clone, Copy)]
pub enum Channel {
    DiffCh0Ch1 = 0b10100000,
    DiffCh2Ch3 = 0b10100001,
    DiffCh4Ch5 = 0b10100010,
    DiffCh6Ch7 = 0b10100011,
    DiffCh8Ch9 = 0b10100100,
    DiffCh10Ch11 = 0b10100101,
    DiffCh12Ch13 = 0b10100110,
    DiffCh14Ch15 = 0b10100111,
    DiffCh1Ch0 = 0b10101000,
    DiffCh3Ch2 = 0b10101001,
    DiffCh5Ch4 = 0b10101010,
    DiffCh7Ch6 = 0b10101011,
    DiffCh9Ch8 = 0b10101100,
    DiffCh11Ch10 = 0b10101101,
    DiffCh13Ch12 = 0b10101110,
    DiffCh15Ch14 = 0b10101111,
    SingleEndedCh0 = 0b10110000,
    SingleEndedCh1 = 0b10111000,
    SingleEndedCh2 = 0b10110001,
    SingleEndedCh3 = 0b10111001,
    SingleEndedCh4 = 0b10110010,
    SingleEndedCh5 = 0b10111010,
    SingleEndedCh6 = 0b10110011,
    SingleEndedCh7 = 0b10111011,
    SingleEndedCh8 = 0b10110100,
    SingleEndedCh9 = 0b10111100,
    SingleEndedCh10 = 0b10110101,
    SingleEndedCh11 = 0b10111101,
    SingleEndedCh12 = 0b10110110,
    SingleEndedCh13 = 0b10111110,
    SingleEndedCh14 = 0b10110111,
    SingleEndedCh15 = 0b10111111,
}

/// Small cheat for easier iteration over all single-ended channels
/// ```no_run
/// for ch in 0..=15 {
///     match adc.read_channel(Channel::from(ch)) {
///         Ok(v) => println!("Channel {ch}: {v}V"),
///         Err(e) => println!("error {e:?} at channel {ch}"),
///     };
/// }
/// ```
impl From<u8> for Channel {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::SingleEndedCh0,
            1 => Self::SingleEndedCh1,
            2 => Self::SingleEndedCh2,
            3 => Self::SingleEndedCh3,
            4 => Self::SingleEndedCh4,
            5 => Self::SingleEndedCh5,
            6 => Self::SingleEndedCh6,
            7 => Self::SingleEndedCh7,
            8 => Self::SingleEndedCh8,
            9 => Self::SingleEndedCh9,
            10 => Self::SingleEndedCh10,
            11 => Self::SingleEndedCh11,
            12 => Self::SingleEndedCh12,
            13 => Self::SingleEndedCh13,
            14 => Self::SingleEndedCh14,
            15 => Self::SingleEndedCh15,
            _ => unimplemented!(),
        }
    }
}

/// At startup LTC2497 configured for differential ch0-ch1 measurements.
impl Default for Channel {
    fn default() -> Self {
        Self::DiffCh0Ch1
    }
}

/// Address pins haven't any pull-ups/pull-downs and are tri-state by nature. So address is configured by these states.
#[derive(Debug)]
pub enum AddressPinState {
    Low,
    High,
    Floating,
}

/// Simple lookup table for easier initialization
fn address_from_pins(ca2: AddressPinState, ca1: AddressPinState, ca0: AddressPinState) -> u8 {
    use AddressPinState::Floating as F;
    use AddressPinState::High as H;
    use AddressPinState::Low as L;
    match (ca2, ca1, ca0) {
        (L, L, L) => 0b0010100,
        (L, L, H) => 0b0010110,
        (L, L, F) => 0b0010101,
        (L, H, L) => 0b0100110,
        (L, H, H) => 0b0110100,
        (L, H, F) => 0b0100111,
        (L, F, L) => 0b0010111,
        (L, F, H) => 0b0100101,
        (L, F, F) => 0b0100100,
        (H, L, L) => 0b1010110,
        (H, L, H) => 0b1100100,
        (H, L, F) => 0b1010111,
        (H, H, L) => 0b1110100,
        (H, H, H) => 0b1110110,
        (H, H, F) => 0b1110101,
        (H, F, L) => 0b1100101,
        (H, F, H) => 0b1100111,
        (H, F, F) => 0b1100110,
        (F, L, L) => 0b0110101,
        (F, L, H) => 0b0110111,
        (F, L, F) => 0b0110110,
        (F, H, L) => 0b1000111,
        (F, H, H) => 0b1010101,
        (F, H, F) => 0b1010100,
        (F, F, L) => 0b1000100,
        (F, F, H) => 0b1000110,
        (F, F, F) => 0b1000101,
    }
}

/// Driver for the LTC2497 ADC
#[derive(Debug, Default)]
pub struct LTC2497<I2C, D> {
    i2c: I2C,
    address: u8,
    channel: Channel,
    vref: f32,
    delay: D,
}

impl<I2C, D, E> LTC2497<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u8>,
{
    /// Initialize the driver by providing exact address.
    /// This constructor has no side effects and writes nothing to device.
    pub fn new(i2c: I2C, address: u8, vref_p: f32, vref_n: f32, delay: D) -> Self {
        assert!(vref_p - vref_n >= 0.1);
        LTC2497 {
            i2c,
            address,
            channel: Channel::default(),
            vref: vref_p - vref_n,
            delay,
        }
    }

    /// Initialize the driver by providing address pins states.
    /// This constructor has no side effects and writes nothing to device.
    pub fn new_from_pins(
        i2c: I2C,
        ca2: AddressPinState,
        ca1: AddressPinState,
        ca0: AddressPinState,
        vref_p: f32,
        vref_n: f32,
        delay: D,
    ) -> Self {
        let address = address_from_pins(ca2, ca1, ca0);
        Self::new(i2c, address, vref_p, vref_n, delay)
    }

    /// Changes current measurement channel.
    pub fn set_channel(&mut self, channel: Channel) -> Result<(), Error<E>> {
        self.channel = channel;

        self.delay.delay_ms(CONVERSION_DELAY);

        self.i2c
            .write(self.address, &[channel as u8])
            .map_err(Error::I2c)?;

        Ok(())
    }

    /// Returns active measurement channel.
    pub fn channel(&self) -> Channel {
        self.channel
    }

    /// Returns measurement result for selected channel.
    /// Triggers new measurement sequence after successful read.
    pub fn read(&mut self) -> Result<f32, Error<E>> {
        let mut buf = [0; 3];

        self.delay.delay_ms(CONVERSION_DELAY);

        self.i2c.read(self.address, &mut buf).map_err(Error::I2c)?;
        let sign: f32 = match (buf[0] & 0b11000000) >> 6 {
            0b00 => return Err(Error::Undervoltage),
            0b01 => -0.5,
            0b10 => 0.5,
            0b11 => return Err(Error::Overvoltage),
            _ => unreachable!(),
        };

        let adc_code =
            (((buf[0] as u32) << 16 | (buf[1] as u32) << 8 | buf[2] as u32) & 0x3FFFFF) >> 6;

        let voltage = if sign.is_sign_positive() {
            sign * self.vref * ((adc_code & 0xFFFF) as f32) / 65535.0
        } else {
            sign * self.vref * ((65536.0 - adc_code as f32) / 65535.0)
        };

        Ok(voltage)
    }

    /// Select new channel and return measurement result.
    pub fn read_channel(&mut self, channel: Channel) -> Result<f32, Error<E>> {
        self.set_channel(channel)?;

        let voltage = self.read()?;

        Ok(voltage)
    }
}
