use embedded_hal as ehal;
use typed_builder::TypedBuilder;

use crate::{Blocking, BMP388};

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

#[derive(Debug, PartialEq, TypedBuilder)]
#[builder(doc, field_defaults(default))]
pub struct Config {
    #[builder(setter(into))]
    pub address: u8,
    /// Override default Oversampling for pressure and temperature measurements
    pub oversampling: OversamplingConfig,
    pub sampling_rate: SamplingRate,
    pub filter: Filter,
    pub interrupt_config: InterruptConfig,
    pub power_control: PowerControl,
}

impl Config {
    pub fn setup_blocking<I2C, E>(
        &self,
        i2c: I2C,
        delay: &mut impl ehal::blocking::delay::DelayMs<u8>,
    ) -> Result<BMP388<I2C, Blocking>, E>
    where
        I2C: ehal::blocking::i2c::WriteRead<Error = E>,
    {
        let mut bmp388 = BMP388::new_blocking(i2c, self.address, delay)?;

        if self.filter != Filter::default() {
            bmp388.set_filter(self.filter)?;
        }

        // TODO: power control
        // TODO: oversampling
        // TODO: sampling rate
        // TODO: interrupt config
        // TODO: power control

        Ok(bmp388)
    }
}

/// Power Control
///
/// Register: `PWR_CTRL`
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct PowerControl {
    /// Pressure sensor enable
    pub pressure_enable: bool,
    /// Temperature sensor enable
    pub temperature_enable: bool,
    /// Power mode
    pub mode: PowerMode,
}

impl PowerControl {
    /// Create a new PowerControl with `PowerMode::Normal` and enable both sensors.
    pub fn normal() -> Self {
        Self {
            pressure_enable: true,
            temperature_enable: true,
            mode: PowerMode::Normal,
        }
    }

    pub(crate) fn from_reg(reg: u8) -> Self {
        let pressure_enable = (reg & 0b1) != 0;
        let temperature_enable = (reg & 0b10) != 0;
        let mode = match reg & (0b11 << 4) >> 4 {
            x if x == PowerMode::Forced as u8 => PowerMode::Forced,
            x if x == PowerMode::Normal as u8 => PowerMode::Normal,
            // in any other case, we assume Sleep as it's the default
            _ => PowerMode::Sleep,
        };

        Self {
            pressure_enable,
            temperature_enable,
            mode,
        }
    }

    pub(crate) fn to_reg(self) -> u8 {
        let mode = (self.mode as u8) << 4;
        let temp_en = (self.temperature_enable as u8) << 1;
        let press_en = self.pressure_enable as u8;

        mode | temp_en | press_en
    }
}

impl Default for PowerControl {
    /// default value for the register is `0x00`
    fn default() -> Self {
        Self {
            pressure_enable: false,
            temperature_enable: false,
            mode: PowerMode::default(),
        }
    }
}

/// Interrupt configuration
///
/// ```
/// use bmp388::InterruptConfig;
///
/// let interrupt_config = InterruptConfig::default();
/// assert_eq!(0x02, interrupt_config.to_reg());
/// assert_eq!(0b000010, interrupt_config.to_reg());
///
/// assert_eq!(InterruptConfig::from_reg(0x02), interrupt_config);
/// ```
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct InterruptConfig {
    ///Output mode of interrupt pin
    pub output: OutputMode,
    ///Level of interrupt pin
    pub active_high: bool,
    ///Latching for interrupt
    pub latch: bool,
    ///Data ready interrupt
    pub data_ready_interrupt_enable: bool,
}

impl InterruptConfig {
    pub fn to_reg(&self) -> u8 {
        let output_mode = self.output as u8;
        let active_high = (self.active_high as u8) << 1;
        let latch = (self.latch as u8) << 2;
        let data_ready = (self.data_ready_interrupt_enable as u8) << 6;

        output_mode | active_high | latch | data_ready
    }

    pub fn from_reg(register: u8) -> Self {
        let output = match register & 0b1 {
            0 => OutputMode::PushPull,
            _ => OutputMode::OpenDrain,
        };
        let active_high = register & (1 << 1) != 0;
        let latch = register & (1 << 2) != 0;
        let data_ready = register & (1 << 6) != 0;

        InterruptConfig {
            output,
            active_high,
            latch,
            data_ready_interrupt_enable: data_ready,
        }
    }
}
impl Default for InterruptConfig {
    fn default() -> Self {
        Self {
            output: OutputMode::PushPull,
            active_high: true,
            latch: false,
            data_ready_interrupt_enable: false,
        }
    }
}

///Output mode for interrupt pin
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum OutputMode {
    ///Push-pull output mode
    PushPull = 0,
    ///Open-drain output mode
    OpenDrain = 1,
}

///Oversampling Config (OSR)
///
/// OSR reg = 0x02 default - Oversampling
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct OversamplingConfig {
    /// Pressure oversampling
    pub osr_pressure: Oversampling,
    /// Temperature oversampling
    pub osr_temperature: Oversampling,
}

impl OversamplingConfig {
    pub(crate) fn to_reg(self) -> u8 {
        let osr_temperature: u8 = (self.osr_temperature as u8) << 3;
        let osr_pressure: u8 = self.osr_pressure as u8;

        osr_temperature | osr_pressure
    }

    pub(crate) fn from_reg(value: u8) -> Self {
        let osr_temperature = match (value & (0b111 << 3)) >> 3 {
            x if x == Oversampling::x1 as u8 => Oversampling::x1,
            x if x == Oversampling::x2 as u8 => Oversampling::x2,
            x if x == Oversampling::x4 as u8 => Oversampling::x4,
            x if x == Oversampling::x8 as u8 => Oversampling::x8,
            x if x == Oversampling::x16 as u8 => Oversampling::x16,
            _ => Oversampling::x32,
        };
        let osr_pressure = match value & 0b111 {
            x if x == Oversampling::x1 as u8 => Oversampling::x1,
            x if x == Oversampling::x2 as u8 => Oversampling::x2,
            x if x == Oversampling::x4 as u8 => Oversampling::x4,
            x if x == Oversampling::x8 as u8 => Oversampling::x8,
            x if x == Oversampling::x16 as u8 => Oversampling::x16,
            _ => Oversampling::x32,
        };

        Self {
            osr_pressure,
            osr_temperature,
        }
    }
}

impl Default for OversamplingConfig {
    fn default() -> Self {
        Self {
            osr_pressure: Oversampling::x1,
            osr_temperature: Oversampling::x1,
        }
    }
}

/// Standby time in ms (ODR reg)
///
/// ODR register (0x1D) = 0x00 - values from 0-17 - Subsampling
///
/// Default value: 0x00
#[derive(Default, Debug, Copy, Clone, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum SamplingRate {
    /// Prescaler 1 (5ms, 200 Hz)
    ///
    /// Description: ODR 200 Hz
    #[default]
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

impl SamplingRate {
    /// For any unknown value it will use the default of `5 ms` or `200 Hz`.
    pub fn from_reg(value: u8) -> Self {
        match value {
            x if x == SamplingRate::ms5 as u8 => SamplingRate::ms5,
            x if x == SamplingRate::ms10 as u8 => SamplingRate::ms10,
            x if x == SamplingRate::ms20 as u8 => SamplingRate::ms20,
            x if x == SamplingRate::ms40 as u8 => SamplingRate::ms40,
            x if x == SamplingRate::ms80 as u8 => SamplingRate::ms80,
            x if x == SamplingRate::ms160 as u8 => SamplingRate::ms160,
            x if x == SamplingRate::ms320 as u8 => SamplingRate::ms320,
            x if x == SamplingRate::ms640 as u8 => SamplingRate::ms640,
            x if x == SamplingRate::ms1_280 as u8 => SamplingRate::ms1_280,
            x if x == SamplingRate::ms2_560 as u8 => SamplingRate::ms2_560,
            x if x == SamplingRate::ms5_120 as u8 => SamplingRate::ms5_120,
            x if x == SamplingRate::ms1_024 as u8 => SamplingRate::ms1_024,
            x if x == SamplingRate::ms2_048 as u8 => SamplingRate::ms2_048,
            x if x == SamplingRate::ms4_096 as u8 => SamplingRate::ms4_096,
            x if x == SamplingRate::ms8_192 as u8 => SamplingRate::ms8_192,
            x if x == SamplingRate::ms16_384 as u8 => SamplingRate::ms16_384,
            x if x == SamplingRate::ms32_768 as u8 => SamplingRate::ms32_768,
            x if x == SamplingRate::ms65_536 as u8 => SamplingRate::ms65_536,
            // for any other value, use the default of 5 ms
            _ => SamplingRate::default(),
        }
    }
}

/// The time constant of IIR filter
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
#[allow(non_camel_case_types)]
// #[repr(u8)]
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

impl Filter {
    /// Parses the filter value from a the Config register
    pub fn from_reg(config_reg: u8) -> Self {
        match config_reg << 1 {
            x if x == Filter::c0 as u8 => Filter::c0,
            x if x == Filter::c1 as u8 => Filter::c1,
            x if x == Filter::c3 as u8 => Filter::c3,
            x if x == Filter::c7 as u8 => Filter::c7,
            x if x == Filter::c15 as u8 => Filter::c15,
            x if x == Filter::c31 as u8 => Filter::c31,
            x if x == Filter::c63 as u8 => Filter::c63,
            x if x == Filter::c127 as u8 => Filter::c127,
            // for any other value use default of c0 (off)
            _ => Filter::c0,
        }
    }

    /// Creates a value for the Config register
    pub fn to_reg(&self) -> u8 {
        (*self as u8) << 1
    }
}

/// Oversampling
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
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
