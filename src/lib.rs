//! A platform agnostic driver to interface with the BMP388 (pressure sensor)
//!
//! This driver is built using [`embedded-hal`] traits.
//!
//! # Examples
//!
//! ```ignore
//! use bmp388::{BMP388, Addr, SensorData, PowerControl, PowerMode};
//! // depends on your board/chip
//! let i2c = todo!("Create the I2C interface");
//! // depends on your board/chip and/or crates you use.
//! let delay = todo!("Create a delay that implements `embedded_hal::delay::DelayMs`");
//! let mut sensor = BMP388::new(i2c, Addr::Primary as u8, delay).unwrap();
//!
//! // default `PowerMode` is `Sleep` so the first thing we need to change is the `PowerControl`.
//! // For continues reading using `Normal` mode,
//! // otherwise you can use `Force` but you have to set it before each sensor data reading.
//!
//! sensor.set_power_control(PowerControl::normal()).unwrap();
//!
//! let sensor_data: SensorData = sensor.sensor_values().unwrap();
//! ```

#![deny(missing_docs)]
#![deny(warnings)]
#![no_std]

use embedded_hal as ehal;

/// The expected value of the ChipId register
pub const CHIP_ID: u8 = 0x50;

/// The I2C address of a [`BMP388`] sensor.
///
/// > The 7-bit device address is 111011x. The 6 MSB bits are fixed. The last bit is changeable by SDO value and can be changed
/// > during operation. Connecting SDO to GND results in slave address 1110110 (0x76); connection it to VDDIO results in slave
/// > address 1110111 (0x77), which is the same as BMP180’s I²C address. The SDO pin cannot be left floating; if left floating, the
/// > I²C address will be undefined.
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
pub enum Addr {
    /// The primary I2C address.
    ///
    /// When SDO is connected to GND
    Primary = 0x76,
    /// The secondary I2C address.
    ///
    /// When SDO is connected to VDDIO
    Secondary = 0x77,
}

/// BMP388 driver
///
/// # Examples
///
/// ```ignore
/// use bmp388::{BMP388, Addr, SensorData};
/// // depends on your board/chip
/// let i2c = todo!("Create the I2C interface");
/// // depends on your board/chip and/or crates you use.
/// let delay = todo!("Create a delay that implements `embedded_hal::delay::DelayMs`");
/// let mut sensor = BMP388::new(i2c, Addr::Primary as u8, delay).unwrap();
///
/// // default `PowerMode` is `Sleep` so the first thing we need to change is the `PowerControl`.
/// // For continues reading using `Normal` mode,
/// // otherwise you can use `Force` but you have to set it before each sensor data reading.
///
/// sensor.set_power_control(PowerControl::normal()).unwrap();
///
/// let sensor_data: SensorData = sensor.sensor_values().unwrap();
/// ```
pub struct BMP388<I2C: ehal::blocking::i2c::WriteRead> {
    com: I2C,
    addr: u8,
    // Temperature compensation
    dig_t1: u16,
    dig_t2: u16,
    dig_t3: i8,
    // Pressure calibration
    dig_p1: i16,
    dig_p2: i16,
    dig_p3: i8,
    dig_p4: i8,
    dig_p5: u16,
    dig_p6: u16,
    dig_p7: i8,
    dig_p8: i8,
    dig_p9: i16,
    dig_p10: i8,
    dig_p11: i8,
}

impl<I2C: ehal::blocking::i2c::WriteRead> BMP388<I2C> {
    /// Creates new BMP388 driver
    ///
    /// The Delay is used to correctly wait for the calibration data after resetting the chip.
    pub fn new<E>(
        i2c: I2C,
        addr: u8,
        delay: &mut impl ehal::blocking::delay::DelayMs<u8>,
    ) -> Result<BMP388<I2C>, E>
    where
        I2C: ehal::blocking::i2c::WriteRead<Error = E>,
    {
        let mut chip = BMP388 {
            com: i2c,
            addr,
            dig_t1: 0,
            dig_t2: 0,
            dig_t3: 0,
            dig_p1: 0,
            dig_p2: 0,
            dig_p3: 0,
            dig_p4: 0,
            dig_p5: 0,
            dig_p6: 0,
            dig_p7: 0,
            dig_p8: 0,
            dig_p9: 0,
            dig_p10: 0,
            dig_p11: 0,
        };

        if chip.id()? == CHIP_ID {
            chip.reset()?;
            // without this the first few bytes of calib data could be incorrectly zero
            delay.delay_ms(10);
            chip.read_calibration()?;
        }

        Ok(chip)
    }
}

impl<I2C: ehal::blocking::i2c::WriteRead> BMP388<I2C> {
    fn read_calibration(&mut self) -> Result<(), I2C::Error> {
        let mut data: [u8; 21] = [0; 21];
        self.com
            .write_read(self.addr, &[Register::calib00 as u8], &mut data)?;

        self.dig_t1 = (data[0] as u16) | ((data[1] as u16) << 8);
        self.dig_t2 = (data[2] as u16) | ((data[3] as u16) << 8);
        self.dig_t3 = data[4] as i8;

        self.dig_p1 = (data[5] as i16) | ((data[6] as i16) << 8);
        self.dig_p2 = (data[7] as i16) | ((data[8] as i16) << 8);
        self.dig_p3 = data[9] as i8;
        self.dig_p4 = data[10] as i8;
        self.dig_p5 = (data[11] as u16) | ((data[12] as u16) << 8);
        self.dig_p6 = (data[13] as u16) | ((data[14] as u16) << 8);
        self.dig_p7 = data[15] as i8;
        self.dig_p8 = data[16] as i8;
        self.dig_p9 = (data[17] as i16) | ((data[18] as i16) << 8);
        self.dig_p10 = data[19] as i8;
        self.dig_p11 = data[20] as i8;
        Ok(())
    }

    /// Reads and returns sensor values
    pub fn sensor_values(&mut self) -> Result<SensorData, I2C::Error> {
        let mut data: [u8; 6] = [0, 0, 0, 0, 0, 0];
        self.com
            .write_read(self.addr, &[Register::sensor_data as u8], &mut data)?;
        let uncompensated_press = (data[0] as u32) | (data[1] as u32) << 8 | (data[2] as u32) << 16;
        let uncompensated_temp = (data[3] as u32) | (data[4] as u32) << 8 | (data[5] as u32) << 16;

        let temperature = self.compensate_temp(uncompensated_temp);
        let pressure = self.compensate_pressure(uncompensated_press, temperature);

        Ok(SensorData {
            pressure,
            temperature,
        })
    }

    /// Compensates a pressure value
    fn compensate_pressure(&self, uncompensated: u32, compensated_temp: f64) -> f64 {
        let uncompensated = uncompensated as f64;
        let p1 = ((self.dig_p1 as f64) - 16_384.0) / 1_048_576.0; //2^14 / 2^20
        let p2 = ((self.dig_p2 as f64) - 16_384.0) / 536_870_912.0; //2^14 / 2^29
        let p3 = (self.dig_p3 as f64) / 4_294_967_296.0; //2^32
        let p4 = (self.dig_p4 as f64) / 137_438_953_472.0; //2^37
        let p5 = (self.dig_p5 as f64) / 0.125; //2^-3
        let p6 = (self.dig_p6 as f64) / 64.0; //2^6
        let p7 = (self.dig_p7 as f64) / 256.0; //2^8
        let p8 = (self.dig_p8 as f64) / 32_768.0; //2^15
        let p9 = (self.dig_p9 as f64) / 281_474_976_710_656.0; //2^48
        let p10 = (self.dig_p10 as f64) / 281_474_976_710_656.0; //2^48
        let p11 = (self.dig_p11 as f64) / 36_893_488_147_419_103_232.0; //2^65

        let mut partial_data1 = p6 * compensated_temp;
        let mut partial_data2 = p7 * (compensated_temp * compensated_temp);
        let mut partial_data3 = p8 * (compensated_temp * compensated_temp * compensated_temp);
        let partial_out1 = p5 + partial_data1 + partial_data2 + partial_data3;

        partial_data1 = p2 * compensated_temp;
        partial_data2 = p3 * (compensated_temp * compensated_temp);
        partial_data3 = p4 * (compensated_temp * compensated_temp * compensated_temp);
        let partial_out2 = uncompensated * (p1 + partial_data1 + partial_data2 + partial_data3);

        partial_data1 = uncompensated * uncompensated;
        partial_data2 = p9 + p10 * compensated_temp;
        partial_data3 = partial_data1 * partial_data2;
        let partial_data4 = partial_data3 + (uncompensated * uncompensated * uncompensated) * p11;

        partial_out1 + partial_out2 + partial_data4
    }

    /// Compensates a temperature value
    fn compensate_temp(&self, uncompensated: u32) -> f64 {
        let t1 = (self.dig_t1 as f64) / 0.00390625; //2^-8
        let t2 = (self.dig_t2 as f64) / 1_073_741_824.0; //2^30
        let t3 = (self.dig_t3 as f64) / 281_474_976_710_656.0; //2^48

        let partial_data1 = (uncompensated as f64) - t1;
        let partial_data2 = partial_data1 * t2;

        partial_data2 + (partial_data1 * partial_data1) * t3
    }

    /// Sets power settings
    pub fn set_power_control(&mut self, new: PowerControl) -> Result<(), I2C::Error> {
        self.write_byte(Register::pwr_ctrl, new.to_reg())
    }

    /// Gets power settings
    pub fn power_control(&mut self) -> Result<PowerControl, I2C::Error> {
        let value = self.read_byte(Register::pwr_ctrl)?;

        Ok(PowerControl::from_reg(value))
    }

    ///Sets sampling rate
    pub fn set_sampling_rate(&mut self, new: SamplingRate) -> Result<(), I2C::Error> {
        let sampling_rate = new as u8;
        self.write_byte(Register::odr, sampling_rate)
    }

    /// Returns current sampling rate
    pub fn sampling_rate(&mut self) -> Result<SamplingRate, I2C::Error> {
        let value = self.read_byte(Register::odr)?;
        let value = match value {
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
            _ => SamplingRate::ms65_536,
        };
        Ok(value)
    }

    /// Returns current filter
    pub fn filter(&mut self) -> Result<Filter, I2C::Error> {
        let config = self.read_byte(Register::config)?;
        let filter = match config << 1 {
            x if x == Filter::c0 as u8 => Filter::c0,
            x if x == Filter::c1 as u8 => Filter::c1,
            x if x == Filter::c3 as u8 => Filter::c3,
            x if x == Filter::c7 as u8 => Filter::c7,
            x if x == Filter::c15 as u8 => Filter::c15,
            x if x == Filter::c31 as u8 => Filter::c31,
            x if x == Filter::c63 as u8 => Filter::c63,
            _ => Filter::c127,
        };
        Ok(filter)
    }

    /// Sets filter
    pub fn set_filter(&mut self, new: Filter) -> Result<(), I2C::Error> {
        let filter = (new as u8) << 1;
        self.write_byte(Register::config, filter)
    }

    /// Sets oversampling configuration
    pub fn set_oversampling(&mut self, new: OversamplingConfig) -> Result<(), I2C::Error> {
        self.write_byte(Register::osr, new.to_reg())
    }

    /// Get oversampling configuration
    pub fn oversampling(&mut self) -> Result<OversamplingConfig, I2C::Error> {
        let value = self.read_byte(Register::osr)?;
        let osr4_t = match (value & (0b111 << 3)) >> 3 {
            x if x == Oversampling::x1 as u8 => Oversampling::x1,
            x if x == Oversampling::x2 as u8 => Oversampling::x2,
            x if x == Oversampling::x4 as u8 => Oversampling::x4,
            x if x == Oversampling::x8 as u8 => Oversampling::x8,
            x if x == Oversampling::x16 as u8 => Oversampling::x16,
            _ => Oversampling::x32,
        };
        let osr_p = match value & 0b111 {
            x if x == Oversampling::x1 as u8 => Oversampling::x1,
            x if x == Oversampling::x2 as u8 => Oversampling::x2,
            x if x == Oversampling::x4 as u8 => Oversampling::x4,
            x if x == Oversampling::x8 as u8 => Oversampling::x8,
            x if x == Oversampling::x16 as u8 => Oversampling::x16,
            _ => Oversampling::x32,
        };
        Ok(OversamplingConfig { osr_p, osr4_t })
    }

    /// Sets interrupt configuration
    pub fn set_interrupt_config(&mut self, new: InterruptConfig) -> Result<(), I2C::Error> {
        let mode = new.output as u8;
        let level = (new.active_high as u8) << 1;
        let latch = (new.latch as u8) << 2;
        let data_ready = (new.data_ready_interrupt_enable as u8) << 6;
        self.write_byte(Register::int_ctrl, mode | level | latch | data_ready)
    }

    /// Returns current interrupt configuration
    pub fn interrupt_config(&mut self) -> Result<InterruptConfig, I2C::Error> {
        let value = self.read_byte(Register::int_ctrl)?;
        let mode = match value & 0b1 {
            0 => OutputMode::PushPull,
            _ => OutputMode::OpenDrain,
        };
        let level = value & (1 << 1) != 0;
        let latch = value & (1 << 2) != 0;
        let data_ready = value & (1 << 6) != 0;

        Ok(InterruptConfig {
            output: mode,
            active_high: level,
            latch,
            data_ready_interrupt_enable: data_ready,
        })
    }

    ///Get the status register
    pub fn status(&mut self) -> Result<Status, I2C::Error> {
        let status = self.read_byte(Register::status)?;

        Ok(Status::from_reg(status))
    }

    ///Get the error register
    pub fn error(&mut self) -> Result<Error, I2C::Error> {
        let error = self.read_byte(Register::err)?;
        Ok(Error {
            fatal: error & (1 << 0) != 0,
            cmd: error & (1 << 1) != 0,
            config: error & (1 << 2) != 0,
        })
    }

    /// Returns device id
    pub fn id(&mut self) -> Result<u8, I2C::Error> {
        self.read_byte(Register::id)
    }

    /// Software reset, emulates POR
    pub fn reset(&mut self) -> Result<(), I2C::Error> {
        self.write_byte(Register::cmd, 0xB6) // Magic from documentation
    }

    fn write_byte(&mut self, reg: Register, byte: u8) -> Result<(), I2C::Error> {
        let mut buffer = [0];
        self.com
            .write_read(self.addr, &[reg as u8, byte], &mut buffer)
    }

    fn read_byte(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut data: [u8; 1] = [0];
        match self.com.write_read(self.addr, &[reg as u8], &mut data) {
            Ok(_) => Ok(data[0]),
            Err(err) => Err(err),
        }
    }
}

///Error
pub struct Error {
    ///Fatal error
    pub fatal: bool,
    ///Command error
    pub cmd: bool,
    ///Configuration error
    pub config: bool,
}

///Status
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Status {
    ///Indicates whether chip is ready for a command
    pub command_ready: bool,
    ///Indicates whether pressure data is ready
    pub pressure_data_ready: bool,
    ///Indicates whether temperature data is ready
    pub temperature_data_ready: bool,
}

impl Status {
    /// Creates the Status from a register value
    pub(crate) fn from_reg(value: u8) -> Self {
        Self {
            command_ready: value & (1 << 4) != 0,
            pressure_data_ready: value & (1 << 5) != 0,
            temperature_data_ready: value & (1 << 6) != 0,
        }
    }
}

/// Interrupt configuration
#[derive(Debug, Copy, Clone)]
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

///Output mode for interrupt pin
#[derive(Debug, Copy, Clone)]
pub enum OutputMode {
    ///Push-pull output mode
    PushPull = 0,
    ///Open-drain output mode
    OpenDrain = 1,
}

#[derive(Debug, Copy, Clone)]
///Sensor data
pub struct SensorData {
    ///The measured pressure
    pub pressure: f64,
    /// The measured temperature
    pub temperature: f64,
}

/// Power Control
///
/// Register: `PWR_CTRL`1
/// ```
/// // Default accordingly to the datasheet
/// ```
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

#[derive(Debug, Copy, Clone)]
///Oversampling Config (OSR)
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

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
/// Standby time in ms
///
/// Register 0x1D “ODR”
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

#[derive(Debug, Copy, Clone, Default)]
#[allow(non_camel_case_types)]
/// The time constant of IIR filter
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

#[derive(Debug, Copy, Clone)]
#[allow(non_camel_case_types)]
/// Oversampling
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

#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
/// PowerMode
///
/// ```
/// use bmp388::PowerMode;
///
/// let default = PowerMode::default();
/// assert_eq!(PowerMode::Sleep, default);
/// ```
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
enum Register {
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

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn test_status_reg_value() {
        let all_true_status = Status::from_reg(0b01110000);
        assert_eq!(
            all_true_status,
            Status {
                command_ready: true,
                pressure_data_ready: true,
                temperature_data_ready: true,
            }
        )
    }

    #[test]
    fn test_oversampling_config_to_reg_value() {
        let config = OversamplingConfig {
            // bits 0 to 2 - 101
            osr_p: Oversampling::x32,
            // bits 3 to 5 - 001
            osr4_t: Oversampling::x2,
        };

        assert_eq!(0b001101, config.to_reg());
    }
}
