use super::*;

use embedded_hal_async as ehal_async;

impl<I2C: ehal_async::i2c::I2c> BMP388<I2C, Async> {
    /// Creates new BMP388 driver
    ///
    /// The Delay is used to correctly wait for the calibration data after resetting the chip.
    #[cfg(feature = "nightly")]
    pub async fn new<E>(
        i2c: I2C,
        addr: u8,
        delay: &mut impl embedded_hal_async::delay::DelayUs,
    ) -> Result<BMP388<I2C, Async>, E>
    where
        I2C: embedded_hal_async::i2c::I2c<embedded_hal_async::i2c::SevenBitAddress, Error = E>,
    {
        let mut chip = BMP388 {
            com: i2c,
            addr,
            sea_level_pressure: STANDARD_SEA_LEVEL_PRESSURE,
            temperature_calibration: TemperatureCalibration::default(),
            pressure_calibration: PressureCalibration::default(),
            phantom: PhantomData,
        };

        if chip.id().await? == CHIP_ID {
            chip.reset().await?;
            // without this the first few bytes of calib data could be incorrectly zero
            delay.delay_ms(10).await;
            chip.read_calibration().await?;
        }

        Ok(chip)
    }

    pub(crate) async fn read_calibration(&mut self) -> Result<(), I2C::Error> {
        let mut data: [u8; 21] = [0; 21];
        self.com
            .write_read(self.addr, &[Register::calib00 as u8], &mut data)
            .await?;

        self.temperature_calibration.update_calibration(data);
        self.pressure_calibration.update_calibration(data);

        Ok(())
    }

    /// Reads and returns sensor values
    pub async fn sensor_values(&mut self) -> Result<SensorData, I2C::Error> {
        let mut data: [u8; 6] = [0, 0, 0, 0, 0, 0];
        self.com
            .write_read(self.addr, &[Register::sensor_data as u8], &mut data)
            .await?;
        let uncompensated_press = (data[0] as u32) | (data[1] as u32) << 8 | (data[2] as u32) << 16;
        let uncompensated_temp = (data[3] as u32) | (data[4] as u32) << 8 | (data[5] as u32) << 16;

        let temperature = self.compensate_temp(uncompensated_temp);
        let pressure = self.compensate_pressure(uncompensated_press, temperature);

        Ok(SensorData {
            pressure,
            temperature,
        })
    }

    /// Calculate altitude, unit: meters
    ///
    /// Reads the sensor's values and calculates the altitude based on
    /// the atmospheric pressure measured by the sensor.
    ///
    /// If the reference value is provided before, the absolute value of the current
    /// position pressure is calculated according to the calibrated sea level atmospheric pressure
    ///
    /// See <https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf>
    pub async fn altitude(&mut self) -> Result<f64, I2C::Error> {
        let pressure = self.sensor_values().await?.pressure;

        let altitude = 44307.69396 * (1.0 - (pressure / self.sea_level_pressure).pow(0.190284));

        Ok(altitude)
    }
    /// Calibrate the the altitude based on the current pressure.
    ///
    /// Takes the given current location altitude as the reference value to
    /// eliminate the absolute difference for subsequent pressure and altitude data.
    /// Make sure you call this method only when needed or it can throw off the
    /// value of newer readings.
    ///
    /// # Returns
    /// The newly calculated sea level pressure.
    ///
    /// Taken from the Arduino library for BMP388: <https://github.com/DFRobot/DFRobot_BMP3XX/blob/master/python/raspberrypi/DFRobot_BMP3XX.py>
    pub async fn calibrated_absolute_difference(
        &mut self,
        altitude: f64,
    ) -> Result<f64, I2C::Error> {
        let pressure = self.sensor_values().await?.pressure;

        self.sea_level_pressure = pressure / (1.0 - (altitude / 44307.69396)).pow(5.255302);

        Ok(self.sea_level_pressure)
    }

    /// Sets power settings
    pub async fn set_power_control(&mut self, new: PowerControl) -> Result<(), I2C::Error> {
        self.write_byte(Register::pwr_ctrl, new.to_reg()).await
    }

    /// Gets power settings
    pub async fn power_control(&mut self) -> Result<PowerControl, I2C::Error> {
        let value = self.read_byte(Register::pwr_ctrl).await?;

        Ok(PowerControl::from_reg(value))
    }

    /// Sets sampling rate
    pub async fn set_sampling_rate(&mut self, new: SamplingRate) -> Result<(), I2C::Error> {
        let sampling_rate = new as u8;
        self.write_byte(Register::odr, sampling_rate).await
    }

    /// Returns current sampling rate
    pub async fn sampling_rate(&mut self) -> Result<SamplingRate, I2C::Error> {
        let value = self.read_byte(Register::odr).await?;
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
    pub async fn filter(&mut self) -> Result<Filter, I2C::Error> {
        let config = self.read_byte(Register::config).await?;
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
    pub async fn set_filter(&mut self, new: Filter) -> Result<(), I2C::Error> {
        let filter = (new as u8) << 1;
        self.write_byte(Register::config, filter).await
    }

    /// Sets oversampling configuration
    pub async fn set_oversampling(&mut self, new: OversamplingConfig) -> Result<(), I2C::Error> {
        self.write_byte(Register::osr, new.to_reg()).await
    }

    /// Get oversampling configuration
    pub async fn oversampling(&mut self) -> Result<OversamplingConfig, I2C::Error> {
        let value = self.read_byte(Register::osr).await?;
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
    pub async fn set_interrupt_config(&mut self, new: InterruptConfig) -> Result<(), I2C::Error> {
        let mode = new.output as u8;
        let level = (new.active_high as u8) << 1;
        let latch = (new.latch as u8) << 2;
        let data_ready = (new.data_ready_interrupt_enable as u8) << 6;
        self.write_byte(Register::int_ctrl, mode | level | latch | data_ready)
            .await
    }

    /// Returns current interrupt configuration
    pub async fn interrupt_config(&mut self) -> Result<InterruptConfig, I2C::Error> {
        let value = self.read_byte(Register::int_ctrl).await?;
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

    /// Get the status register
    pub async fn status(&mut self) -> Result<Status, I2C::Error> {
        let status = self.read_byte(Register::status).await?;

        Ok(Status::from_reg(status))
    }

    /// Get the error register
    pub async fn error(&mut self) -> Result<Error, I2C::Error> {
        let error = self.read_byte(Register::err).await?;
        Ok(Error {
            fatal: error & (1 << 0) != 0,
            cmd: error & (1 << 1) != 0,
            config: error & (1 << 2) != 0,
        })
    }

    /// Returns device id
    pub async fn id(&mut self) -> Result<u8, I2C::Error> {
        self.read_byte(Register::id).await
    }

    /// Software reset, emulates POR
    pub async fn reset(&mut self) -> Result<(), I2C::Error> {
        self.write_byte(Register::cmd, 0xB6).await // Magic from documentation
    }

    async fn write_byte(&mut self, reg: Register, byte: u8) -> Result<(), I2C::Error> {
        let mut buffer = [0];
        self.com
            .write_read(self.addr, &[reg as u8, byte], &mut buffer)
            .await
    }

    async fn read_byte(&mut self, reg: Register) -> Result<u8, I2C::Error> {
        let mut data: [u8; 1] = [0];
        match self
            .com
            .write_read(self.addr, &[reg as u8], &mut data)
            .await
        {
            Ok(_) => Ok(data[0]),
            Err(err) => Err(err),
        }
    }
}
