use typed_builder::TypedBuilder;
use embedded_hal as ehal;

use crate::{
    Blocking, Filter, InterruptConfig, OversamplingConfig, PowerControl, SamplingRate, BMP388,
};

#[derive(Debug, PartialEq, TypedBuilder)]
#[builder(doc, field_defaults(default))]
pub struct Config {
    /// Override default Oversampling for pressure and temperature measurements
    // #[builder(default)]
    oversampling: OversamplingConfig,

    // #[builder(default)] without parameter - use the type's default
    // #[builder(setter(strip_option))] - wrap the setter argument with `Some(...)`
    // #[builder(default, setter(strip_option))]
    // y: Option<i32>,
    sampling_rate: SamplingRate,

    // Or you can set the default
    // #[builder(default=20)]
    // #[builder(default)]
    filter: Filter,

    // #[builder(default)]
    interrupt_config: InterruptConfig,

    // #[builder(default)]
    power_control: PowerControl,
}

impl Config {
    pub fn setup_blocking<I2C, E>(&self, i2c: I2C) -> Result<BMP388<I2C, Blocking>, E>
    where
        I2C: ehal::blocking::i2c::WriteRead<Error = E>,
    {
        let mut bmp388 = BMP388::new_blocking(i2c, address, delay)?;

        if self.filter != Filter::default() {
            bmp388.set_filter(self.filter)?;
        }

        Ok(bmp388)
    }
}

///Oversampling Config (OSR)
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct OversamplingConfig {
    /// Pressure oversampling
    pub osr_p: Oversampling,
    /// Temperature oversampling
    pub osr4_t: Oversampling,
}

impl OversamplingConfig {
    pub(crate) fn to_reg(self) -> u8 {
        let osr_t: u8 = (self.osr4_t as u8) << 3;
        let osr_p: u8 = self.osr_p as u8;

        osr_t | osr_p
    }
}

/// Standby time in ms
///
/// Register 0x1D “ODR”
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum SamplingRate {
    /// Prescaler 1 (5ms, 200 Hz)
    ///
    /// Description: ODR 200 Hz
    ms5 = 0x00,
    /// Prescaler 2 (10ms, 100 Hz)
    ///
    /// Description: ODR 100 Hz
    ms10 = 0x01,
    /// Prescaler 4 (20ms, 50 Hz)
    ///
    /// Description: ODR 50 Hz
    ms20 = 0x02,
    /// Prescaler 8 (40ms, 25 Hz)
    ///
    /// Description: ODR 25 Hz
    ms40 = 0x03,
    /// Prescaler 16 (80ms, 25/2 Hz)
    ///
    /// Description: ODR 25/2 Hz
    ms80 = 0x04,
    /// Prescaler 32 (160ms, 25/4 Hz)
    ///
    /// Description: ODR 25/4 Hz
    ms160 = 0x05,
    /// Prescaler 64 (320ms, 25/8 Hz)
    ///
    /// Description: ODR 25/8 Hz
    ms320 = 0x06,
    /// Prescaler 128 (640ms, 25/18 Hz)
    ///
    /// Description: ODR 25/18 Hz
    ms640 = 0x07,
    /// Prescaler 256 (1.280s, 25/32 Hz)
    ///
    /// Description: ODR 25/32 Hz
    ms1_280 = 0x08,
    /// Prescaler 512 (2.560s, 25/64 Hz)
    ///
    /// Description: ODR 25/64 Hz
    ms2_560 = 0x09,
    /// Prescaler 1024 (5.120s, 25/128 Hz)
    ///
    /// Description: ODR 25/128 Hz
    ms5_120 = 0x0A,
    /// Prescaler 2048 (10.24s, 25/256 Hz)
    ///
    /// Description: ODR 25/256 Hz
    ms1_024 = 0x0B,
    /// Prescaler 4096 (20.48s, 25/512 Hz)
    ///
    /// Description: ODR 25/512 Hz
    ms2_048 = 0x0C,
    /// Prescaler 8192 (40.96s, 25/1024 Hz)
    ///
    /// Description: ODR 25/1024 Hz
    ms4_096 = 0x0D,
    /// Prescaler 16384 (81.92s, 25/2048 Hz)
    ///
    /// Description: ODR 25/2048 Hz
    ms8_192 = 0x0E,
    /// Prescaler 32768 (163.84s, 25/4096 Hz)
    ///
    /// Description: ODR 25/4096 Hz
    ms16_384 = 0x0F,
    /// Prescaler 65536 (327.68s, 25/8192 Hz)
    ///
    /// Description: ODR 25/8192 Hz
    ms32_768 = 0x10,
    /// Prescaler 131072 (655.36s, 25/16384 Hz)
    ///
    /// Description: ODR 25/16384 Hz
    ms65_536 = 0x11,
}

/// The time constant of IIR filter
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum Filter {
    #[default]
    ///off
    c0 = 0b000,
    ///Coefficient 1
    c1 = 0b001,
    ///Coefficient 3
    c3 = 0b010,
    ///Coefficient 7
    c7 = 0b011,
    ///Coefficient 15
    c15 = 0b100,
    ///Coefficient 31
    c31 = 0b101,
    ///Coefficient 63
    c63 = 0b110,
    ///Coefficient 127
    c127 = 0b111,
}

/// Oversampling
#[derive(Debug, Copy, Clone, , PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum Oversampling {
    /// x1
    ///
    /// Pressure setting - Ultra low power
    /// Typical pressure resolution - 16 bit / 2.64 Pa
    /// Recommended temperature oversampling: x1
    ///
    /// Typical temperature resolution - 16 bit / 0.0050 °C
    x1 = 0b000,
    /// x2
    ///
    /// Pressure setting - Low power
    /// Typical pressure resolution - 17 bit / 1.32 Pa
    /// Recommended temperature oversampling: x1
    ///
    /// Typical temperature resolution - 17 bit / 0.0025 °C
    x2 = 0b001,
    /// x4
    ///
    /// Pressure setting - Standard resolution
    /// Typical pressure resolution - 18 bit / 0.66 Pa
    /// Recommended temperature oversampling: x1
    ///
    /// Typical temperature resolution - 18 bit / 0.0012 °C
    x4 = 0b010,
    /// x8
    ///
    /// Pressure setting - High resolution
    /// Typical pressure resolution - 19 bit / 0.33 Pa
    /// Recommended temperature oversampling: x1
    ///
    /// Typical temperature resolution - 19 bit / 0.0006 °C
    x8 = 0b011,
    /// x16
    ///
    /// Pressure setting - Ultra high resolution
    /// Typical pressure resolution - 20 bit / 0.17 Pa
    /// Recommended temperature oversampling: x2
    ///
    /// Typical temperature resolution - 20 bit / 0.0003 °C
    x16 = 0b100,
    /// x32
    ///
    /// Pressure setting - Highest resolution
    /// Typical pressure resolution - 21 bit / 0.085 Pa
    /// Recommended temperature oversampling: x2
    ///
    /// Typical temperature resolution - 21 bit / 0.00015 °C
    x32 = 0b101,
}

/// PowerMode
///
/// ```
/// use bmp388::PowerMode;
///
/// let default = PowerMode::default();
/// assert_eq!(PowerMode::Sleep, default);
/// ```
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerMode {
    /// Sleep
    ///
    /// Default Power model on start-up
    #[default]
    Sleep = 0b00,
    /// Forced
    Forced = 0b01,
    /// Normal
    Normal = 0b11,
}

#[allow(non_camel_case_types)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub(crate) enum Register {
    id = 0x00,
    sensor_data = 0x04,
    config = 0x1F,
    odr = 0x1D,
    osr = 0x1C,
    pwr_ctrl = 0x1B,
    int_ctrl = 0x19,
    calib00 = 0x31,
    cmd = 0x7E,
    status = 0x03,
    err = 0x02,
}
