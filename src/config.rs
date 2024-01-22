#[cfg(feature = "config-builder")]
pub use config_builder::*;

use crate::{OutputMode, Oversampling};

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
    pub fn to_reg(self) -> u8 {
        let osr_temperature: u8 = (self.osr_temperature as u8) << 3;
        let osr_pressure: u8 = self.osr_pressure as u8;

        osr_temperature | osr_pressure
    }

    pub fn from_reg(value: u8) -> Self {
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

/// Interrupt configuration
///
/// ```
/// use bmp388::{config::InterruptConfig, OutputMode};
///
/// let interrupt_config = InterruptConfig::default();
/// assert_eq!(0x02, interrupt_config.to_reg());
/// assert_eq!(0b000010, interrupt_config.to_reg());
///
/// let default = InterruptConfig {
///     output: OutputMode::PushPull,
///     active_high: true,
///     latch: false,
///     data_ready_interrupt_enable: false,
/// };
/// assert_eq!(default, interrupt_config);
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

#[cfg(feature = "config-builder")]
mod config_builder {

    use embedded_hal as ehal;

    use typed_builder::TypedBuilder;

    use super::*;
    use crate::{Blocking, Filter, PowerControl, SamplingRate, BMP388};

    /// Configuration for initial setup of [`BMP388`]
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
        /// Create a new blocking instance of [`BMP388`]
        ///
        /// If a provided configuration value is different than the chip's default
        /// register value defined in the datasheet it will set it up using the
        /// I2C bus, returning an error if any of the configuration values
        /// fails to be set.
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

            if self.power_control != PowerControl::default() {
                bmp388.set_power_control(self.power_control)?;
            }

            if self.oversampling != OversamplingConfig::default() {
                bmp388.set_oversampling(self.oversampling)?;
            }

            if self.sampling_rate != SamplingRate::default() {
                bmp388.set_sampling_rate(self.sampling_rate)?;
            }

            if self.interrupt_config != InterruptConfig::default() {
                bmp388.set_interrupt_config(self.interrupt_config)?;
            }

            Ok(bmp388)
        }
    }
}

#[cfg(test)]
mod test {
    use crate::{Oversampling, OversamplingConfig};
    #[test]
    fn test_oversampling_config_to_reg_value() {
        let config = OversamplingConfig {
            // bits 0 to 2 - 101
            osr_pressure: Oversampling::x32,
            // bits 3 to 5 - 001
            osr_temperature: Oversampling::x2,
        };

        assert_eq!(0b001101, config.to_reg());
    }
}
