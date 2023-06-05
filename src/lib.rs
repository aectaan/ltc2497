pub enum Error<E> {
    I2c(E),
    Overvoltage,
    Undervoltage,
    NotReady,
}

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

pub enum AddressPinState {
    Low,
    High,
    Floating,
}

fn address_from_pins(ca2: AddressPinState, ca1: AddressPinState, ca0: AddressPinState) -> u8 {
    let pins = (ca2, ca1, ca0);
    use AddressPinState::Floating as F;
    use AddressPinState::High as H;
    use AddressPinState::Low as L;
    match pins {
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
