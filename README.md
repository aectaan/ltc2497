# LTC2497 Rust Driver

This is a platform agnostic driver for the Analog Devices LTC2497 16-Bit 8-/16-Channel DSADC with I2C Interface.

## Device

LTC2497 is a low power 16-bit analog-to-digital converter with 16 single-ended inputs, that can be used as 8 differential inputs.

The device has an I2C interface with possibility to set one of 27 addresses via GPIO pins and possibility to synchronise readings on the batch of devices via hardcoded global address. Device has internal oscillator but may be configured to use external one. 

[Device info](https://www.analog.com/en/products/ltc2497.html) and [Datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/2497fb.pdf)