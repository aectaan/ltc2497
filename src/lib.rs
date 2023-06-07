use embedded_hal::blocking::{
    delay::DelayMs,
    i2c::{Read, Write, WriteRead},
};

pub enum Error<E> {
    I2c(E),
    Overvoltage,
    Undervoltage,
    NotReady,
}

#[derive(Debug, Clone, Copy)]
pub enum Channel {
    LastSelected = 0b00000000,
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
    SingleEndedCh1 = 0b10110001,
    SingleEndedCh2 = 0b10110010,
    SingleEndedCh3 = 0b10110011,
    SingleEndedCh4 = 0b10110100,
    SingleEndedCh5 = 0b10110101,
    SingleEndedCh6 = 0b10110110,
    SingleEndedCh7 = 0b10110111,
    SingleEndedCh8 = 0b10111000,
    SingleEndedCh9 = 0b10111001,
    SingleEndedCh10 = 0b10111010,
    SingleEndedCh11 = 0b10111011,
    SingleEndedCh12 = 0b10111100,
    SingleEndedCh13 = 0b10111101,
    SingleEndedCh14 = 0b10111110,
    SingleEndedCh15 = 0b10111111,
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
    delay: D,
    channel: Channel,
    vref: f32,
}

impl<I2C, D, E> LTC2497<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u8>,
{
    pub fn new(i2c: I2C, address: u8, delay: D, vref_p: f32, vref_n: f32) -> Self {
        assert!(vref_p - vref_n >= 0.1);
        LTC2497 {
            i2c,
            address,
            delay,
            channel: Channel::default(),
            vref: vref_p - vref_n,
        }
    }

    pub fn new_from_pins(
        i2c: I2C,
        ca2: AddressPinState,
        ca1: AddressPinState,
        ca0: AddressPinState,
        delay: D,
        vref_p: f32,
        vref_n: f32,
    ) -> Self {
        let address = address_from_pins(ca2, ca1, ca0);
        Self::new(i2c, address, delay, vref_p, vref_n)
    }

    pub fn set_channel(&mut self, channel: Channel) -> Result<(), E> {
        self.channel = channel;

        self.i2c.write(self.address, &[channel as u8])
    }

    pub fn channel(&self) -> Channel {
        self.channel
    }

    pub fn read(&mut self) -> Result<f32, Error<E>> {
        let mut buf = [0; 3];
        self.i2c.read(self.address, &mut buf).map_err(Error::I2c)?;
        let sign: f32 = match (buf[0] & 0b11000000) >> 6 {
            0b00 => return Err(Error::Undervoltage),
            0b01 => -0.5,
            0b10 => 0.5,
            0b11 => return Err(Error::Overvoltage),
            _ => unreachable!(),
        };

        let adc_code =
            (((buf[2] as u32) << 16 | (buf[1] as u32) << 8 | buf[0] as u32) & 0x3FFFFF) >> 6;

        //https://github.com/DuyTrandeLion/peripheral-drivers/blob/fb1dc6f390839b7f8ee52f8b14bd91ad4c8f3555/LTC2497/ltc2497.c#L80
        //https://github.com/analogdevicesinc/Linduino/blob/bff9185178d2bf694d0fed14a85392f21655c7de/LTSketchbook/libraries/LTC24XX_general/LTC24XX_general.cpp#L389
        let voltage = if sign.is_sign_positive() {
            sign * self.vref * ((adc_code & 0xFFFF) as f32) / 65535.0
        } else {
            sign * self.vref * ((65536.0 - adc_code as f32) / 65535.0)
        };

        Ok(voltage)
    }

    pub fn read_channel(&mut self, channel: Channel) -> Result<f32, Error<E>> {
        let mut buf = [0; 3];
        self.i2c
            .write_read(self.address, &[channel as u8], &mut buf)
            .map_err(Error::I2c)?;

        let sign: f32 = match (buf[0] & 0b11000000) >> 6 {
            0b00 => return Err(Error::Undervoltage),
            0b01 => -0.5,
            0b10 => 0.5,
            0b11 => return Err(Error::Overvoltage),
            _ => unreachable!(),
        };

        let adc_code =
            (((buf[2] as u32) << 16 | (buf[1] as u32) << 8 | buf[0] as u32) & 0x3FFFFF) >> 6;

        let voltage = if sign.is_sign_positive() {
            sign * self.vref * ((adc_code & 0x1FFFF) as f32) / 65535.0
        } else {
            sign * self.vref * ((65536.0 - adc_code as f32) / 65535.0)
        };

        Ok(voltage)
    }
}
