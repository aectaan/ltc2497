use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Read, Write, WriteRead},
};

const CONVERSION_DELAY: u8 = 150;

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    Overvoltage,
    Undervoltage,
}

#[derive(Debug, Clone, Copy)]
pub enum Channel {
    LastSelected = 0b00000000,
    DiffCh0Ch1 = 0b101_00_000,
    DiffCh2Ch3 = 0b101_00_001,
    DiffCh4Ch5 = 0b101_00_010,
    DiffCh6Ch7 = 0b101_00_011,
    DiffCh8Ch9 = 0b101_00_100,
    DiffCh10Ch11 = 0b101_00_101,
    DiffCh12Ch13 = 0b101_00_110,
    DiffCh14Ch15 = 0b101_00_111,
    DiffCh1Ch0 = 0b101_01_000,
    DiffCh3Ch2 = 0b101_01_001,
    DiffCh5Ch4 = 0b101_01_010,
    DiffCh7Ch6 = 0b101_01_011,
    DiffCh9Ch8 = 0b101_01_100,
    DiffCh11Ch10 = 0b101_01_101,
    DiffCh13Ch12 = 0b101_01_110,
    DiffCh15Ch14 = 0b101_01_111,
    SingleEndedCh0 = 0b101_10_000,
    SingleEndedCh1 = 0b101_11_000,
    SingleEndedCh2 = 0b101_10_001,
    SingleEndedCh3 = 0b101_11_001,
    SingleEndedCh4 = 0b101_10_010,
    SingleEndedCh5 = 0b101_11_010,
    SingleEndedCh6 = 0b101_10_011,
    SingleEndedCh7 = 0b101_11_011,
    SingleEndedCh8 = 0b101_10_100,
    SingleEndedCh9 = 0b101_11_100,
    SingleEndedCh10 = 0b101_10_101,
    SingleEndedCh11 = 0b101_11_101,
    SingleEndedCh12 = 0b101_10_110,
    SingleEndedCh13 = 0b101_11_110,
    SingleEndedCh14 = 0b101_10_111,
    SingleEndedCh15 = 0b101_11_111,
}

impl TryFrom<u8> for Channel {
    type Error = String;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0 => Ok(Self::SingleEndedCh0),
            1 => Ok(Self::SingleEndedCh1),
            2 => Ok(Self::SingleEndedCh2),
            3 => Ok(Self::SingleEndedCh3),
            4 => Ok(Self::SingleEndedCh4),
            5 => Ok(Self::SingleEndedCh5),
            6 => Ok(Self::SingleEndedCh6),
            7 => Ok(Self::SingleEndedCh7),
            8 => Ok(Self::SingleEndedCh8),
            9 => Ok(Self::SingleEndedCh9),
            10 => Ok(Self::SingleEndedCh10),
            11 => Ok(Self::SingleEndedCh11),
            12 => Ok(Self::SingleEndedCh12),
            13 => Ok(Self::SingleEndedCh13),
            14 => Ok(Self::SingleEndedCh14),
            15 => Ok(Self::SingleEndedCh15),
            _ => Err(format!("Channel with index {value} doesn't exist")),
        }
    }
}

impl Default for Channel {
    fn default() -> Self {
        Self::DiffCh0Ch1
    }
}

#[derive(Debug)]
pub enum AddressPinState {
    Low,
    High,
    Floating,
}

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

    pub fn set_channel(&mut self, channel: Channel) -> Result<(), Error<E>> {
        self.channel = channel;

        self.delay.delay_ms(CONVERSION_DELAY);

        self.i2c
            .write(self.address, &[channel as u8])
            .map_err(Error::I2c)?;

        Ok(())
    }

    pub fn channel(&self) -> Channel {
        self.channel
    }

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

    pub fn read_channel(&mut self, channel: Channel) -> Result<f32, Error<E>> {
        self.set_channel(channel)?;

        let voltage = self.read()?;

        Ok(voltage)
    }
}
