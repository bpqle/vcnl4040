use embedded_hal_async::i2c::I2c;

/// Default I2C address for VCNL4040
pub const VCNL4040_I2C_ADDR_DEFAULT: u8 = 0x60;

/// Register addresses (16-bit registers)
pub mod registers {
    pub const ALS_CONFIG: u8 = 0x00;     // Ambient light sensor configuration
pub const ALS_THDH: u8 = 0x01;       // Ambient light high threshold
pub const ALS_THDL: u8 = 0x02;       // Ambient light low threshold
pub const PS_CONF1_L: u8 = 0x03;     // Proximity sensor configuration 1/2
pub const PS_MS_H: u8 = 0x04;        // Proximity sensor configuration 1/2
pub const PS_THDL: u8 = 0x06;        // Proximity sensor low threshold
pub const PS_THDH: u8 = 0x07;        // Proximity sensor high threshold
pub const PS_DATA: u8 = 0x08;        // Proximity sensor data
pub const ALS_DATA: u8 = 0x09;       // Ambient light sensor data
pub const WHITE_DATA: u8 = 0x0A;     // White light sensor data
pub const INT_FLAG: u8 = 0x0B;       // Interrupt status
pub const DEVICE_ID: u8 = 0x0C;      // Device ID
}

/// Proximity LED current values
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum LedCurrent {
    Current50mA = 0,
    Current75mA = 1,
    Current100mA = 2,
    Current120mA = 3,
    Current140mA = 4,
    Current160mA = 5,
    Current180mA = 6,
    Current200mA = 7,
}

/// Proximity LED duty cycle values
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum LedDutyCycle {
    Duty1_40 = 0,
    Duty1_80 = 1,
    Duty1_160 = 2,
    Duty1_320 = 3,
}

/// Ambient light integration time values
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum AmbientIntegrationTime {
    Time80ms = 0,
    Time160ms = 1,
    Time320ms = 2,
    Time640ms = 3,
}

/// Proximity measurement integration time values
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum ProximityIntegrationTime {
    Time1T = 0,
    Time1_5T = 1,
    Time2T = 2,
    Time2_5T = 3,
    Time3T = 4,
    Time3_5T = 5,
    Time4T = 6,
    Time8T = 7,
}

/// Proximity interrupt types
#[derive(Debug, Clone, Copy, PartialEq)]
#[repr(u8)]
pub enum ProximityInterruptType {
    Disable = 0,
    Close = 1,
    Away = 2,
    CloseAway = 3,
}

/// Interrupt status flags
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InterruptStatus {
    pub proximity_away: bool,
    pub proximity_close: bool,
    pub ambient_high: bool,
    pub ambient_low: bool,
    pub proximity_protect_mode: bool,
}

impl From<u8> for InterruptStatus {
    fn from(value: u8) -> Self {
        Self {
            proximity_away: (value & 0x01) != 0,
            proximity_close: (value & 0x02) != 0,
            ambient_high: (value & 0x10) != 0,
            ambient_low: (value & 0x20) != 0,
            proximity_protect_mode: (value & 0x40) != 0,
        }
    }
}

/// Error types for VCNL4040 operations
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Error<E> {
    /// I2C communication error
    I2c(E),
    /// Invalid device ID
    InvalidDevice,
    /// Invalid parameter
    InvalidParameter,
}

impl<E> From<E> for Error<E> {
    fn from(error: E) -> Self {
        Error::I2c(error)
    }
}

/// VCNL4040 sensor driver (async)
pub struct Vcnl4040<I2C> {
    i2c: I2C,
    address: u8,
}

impl<I2C> Vcnl4040<I2C>
    where
        I2C: I2c,
{
    /// Create a new VCNL4040 driver instance
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            address: VCNL4040_I2C_ADDR_DEFAULT,
        }
    }

    /// Create a new VCNL4040 driver instance with custom I2C address
    pub fn new_with_address(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Initialize the sensor and verify device ID
    pub async fn init(&mut self, enable_all: bool) -> Result<(), Error<I2C::Error>> {
        // Verify device ID - VCNL4040 should return 0x0186
        let device_id = self.read_register(registers::DEVICE_ID).await?;
        if device_id != 0x0186 {
            return Err(Error::InvalidDevice);
        }

        // Initialize with default settings
        if enable_all {
            self.enable_proximity(true).await?;
            self.enable_white_light(true).await?;
            self.enable_ambient_light(true).await?;
            self.set_proximity_high_resolution(true).await?;
        }
        Ok(())
    }

    /// Read proximity sensor data
    pub async fn get_proximity(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_register(registers::PS_DATA).await
    }

    /// Read ambient light sensor data
    pub async fn get_ambient_light(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_register(registers::ALS_DATA).await
    }

    /// Read white light sensor data with proper scaling
    pub async fn get_white_light(&mut self) -> Result<u16, Error<I2C::Error>> {
        let white_data = self.read_register(registers::WHITE_DATA).await?;

        // Scale the light depending on the integration time
        // See page 8 of the VCNL4040 application note
        let integration_time = self.get_ambient_integration_time().await?;
        let scale_factor = 0.1 / (1 << (integration_time as u8)) as f32;

        Ok((white_data as f32 * scale_factor) as u16)
    }

    /// Calculate lux value from ambient light reading with proper scaling
    pub async fn get_lux(&mut self) -> Result<u16, Error<I2C::Error>> {
        let als_data = self.get_ambient_light().await?;

        // Scale the lux depending on the integration time
        // See page 8 of the VCNL4040 application note
        let integration_time = self.get_ambient_integration_time().await?;
        let scale_factor = 0.1 / (1 << (integration_time as u8)) as f32;

        Ok((als_data as f32 * scale_factor) as u16)
    }

    /// Enable/disable proximity sensor
    pub async fn enable_proximity(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        let mut config = self.read_register(registers::PS_CONF1_L).await?;
        if enable {
            config &= !0x01; // Clear SD bit to enable
        } else {
            config |= 0x01; // Set SD bit to disable
        }
        self.write_register(registers::PS_CONF1_L, config).await
    }

    /// Enable/disable white light sensor
    pub async fn enable_white_light(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        let mut config = self.read_register(registers::PS_MS_H).await?;
        if enable {
            config &= !(1 << 15); // Clear bit 15 to enable
        } else {
            config |= 1 << 15; // Set bit 15 to disable
        }
        self.write_register(registers::PS_MS_H, config).await
    }

    /// Enable/disable ambient light sensor
    pub async fn enable_ambient_light(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        let mut config = self.read_register(registers::ALS_CONFIG).await?;
        if enable {
            config &= !0x01; // Clear SD bit to enable
        } else {
            config |= 0x01; // Set SD bit to disable
        }
        self.write_register(registers::ALS_CONFIG, config).await
    }

    /// Get interrupt status (clears interrupt flags)
    pub async fn get_interrupt_status(&mut self) -> Result<InterruptStatus, Error<I2C::Error>> {
        let status = self.read_register(registers::INT_FLAG).await?;
        // The interrupt status is in the upper byte (bits 15:8)
        let status_byte = (status >> 8) as u8;
        Ok(InterruptStatus::from(status_byte))
    }

    /// Set ambient light high threshold
    pub async fn set_ambient_light_high_threshold(&mut self, threshold: u16) -> Result<(), Error<I2C::Error>> {
        self.write_register(registers::ALS_THDH, threshold).await
    }

    /// Get ambient light high threshold
    pub async fn get_ambient_light_high_threshold(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_register(registers::ALS_THDH).await
    }

    /// Set ambient light low threshold
    pub async fn set_ambient_light_low_threshold(&mut self, threshold: u16) -> Result<(), Error<I2C::Error>> {
        self.write_register(registers::ALS_THDL, threshold).await
    }

    /// Get ambient light low threshold
    pub async fn get_ambient_light_low_threshold(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_register(registers::ALS_THDL).await
    }

    /// Set proximity high threshold
    pub async fn set_proximity_high_threshold(&mut self, threshold: u16) -> Result<(), Error<I2C::Error>> {
        self.write_register(registers::PS_THDH, threshold).await
    }

    /// Get proximity high threshold
    pub async fn get_proximity_high_threshold(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_register(registers::PS_THDH).await
    }

    /// Set proximity low threshold
    pub async fn set_proximity_low_threshold(&mut self, threshold: u16) -> Result<(), Error<I2C::Error>> {
        self.write_register(registers::PS_THDL, threshold).await
    }

    /// Get proximity low threshold
    pub async fn get_proximity_low_threshold(&mut self) -> Result<u16, Error<I2C::Error>> {
        self.read_register(registers::PS_THDL).await
    }

    /// Set proximity LED current
    pub async fn set_proximity_led_current(&mut self, current: LedCurrent) -> Result<(), Error<I2C::Error>> {
        let mut config = self.read_register(registers::PS_MS_H).await?;
        config = (config & !0x0700) | ((current as u16) << 8); // Clear bits 10:8 and set new value
        self.write_register(registers::PS_MS_H, config).await
    }

    /// Get proximity LED current
    pub async fn get_proximity_led_current(&mut self) -> Result<LedCurrent, Error<I2C::Error>> {
        let config = self.read_register(registers::PS_MS_H).await?;
        let current_value = (config >> 8) & 0x07; // Extract bits 10:8
        match current_value {
            0 => Ok(LedCurrent::Current50mA),
            1 => Ok(LedCurrent::Current75mA),
            2 => Ok(LedCurrent::Current100mA),
            3 => Ok(LedCurrent::Current120mA),
            4 => Ok(LedCurrent::Current140mA),
            5 => Ok(LedCurrent::Current160mA),
            6 => Ok(LedCurrent::Current180mA),
            7 => Ok(LedCurrent::Current200mA),
            _ => Err(Error::InvalidParameter),
        }
    }

    /// Set proximity LED duty cycle
    pub async fn set_proximity_led_duty_cycle(&mut self, duty_cycle: LedDutyCycle) -> Result<(), Error<I2C::Error>> {
        let mut config = self.read_register(registers::PS_CONF1_L).await?;
        config = (config & !0xC0) | ((duty_cycle as u16) << 6); // Clear bits 7:6 and set new value
        self.write_register(registers::PS_CONF1_L, config).await
    }

    /// Get proximity LED duty cycle
    pub async fn get_proximity_led_duty_cycle(&mut self) -> Result<LedDutyCycle, Error<I2C::Error>> {
        let config = self.read_register(registers::PS_CONF1_L).await?;
        let duty_value = (config >> 6) & 0x03; // Extract bits 7:6
        match duty_value {
            0 => Ok(LedDutyCycle::Duty1_40),
            1 => Ok(LedDutyCycle::Duty1_80),
            2 => Ok(LedDutyCycle::Duty1_160),
            3 => Ok(LedDutyCycle::Duty1_320),
            _ => Err(Error::InvalidParameter),
        }
    }

    /// Set ambient light integration time
    pub async fn set_ambient_integration_time(&mut self, time: AmbientIntegrationTime) -> Result<(), Error<I2C::Error>> {
        let mut config = self.read_register(registers::ALS_CONFIG).await?;

        // Calculate delay based on old and new integration times
        let old_time = self.get_ambient_integration_time().await?;
        let _old_it_ms = (8 << (old_time as u8)) * 10;
        let _new_it_ms = (8 << (time as u8)) * 10;

        config = (config & !0xC0) | ((time as u16) << 6); // Clear bits 7:6 and set new value
        self.write_register(registers::ALS_CONFIG, config).await?;

        // Note: In async embedded systems, you'd typically use an async delay
        // This is a placeholder - implement proper async delay based on your platform
        // delay.delay_ms(old_it_ms + new_it_ms + 1).await;

        Ok(())
    }

    /// Get ambient light integration time
    pub async fn get_ambient_integration_time(&mut self) -> Result<AmbientIntegrationTime, Error<I2C::Error>> {
        let config = self.read_register(registers::ALS_CONFIG).await?;
        let time_value = (config >> 6) & 0x03; // Extract bits 7:6
        match time_value {
            0 => Ok(AmbientIntegrationTime::Time80ms),
            1 => Ok(AmbientIntegrationTime::Time160ms),
            2 => Ok(AmbientIntegrationTime::Time320ms),
            3 => Ok(AmbientIntegrationTime::Time640ms),
            _ => Err(Error::InvalidParameter),
        }
    }

    /// Set proximity integration time
    pub async fn set_proximity_integration_time(&mut self, time: ProximityIntegrationTime) -> Result<(), Error<I2C::Error>> {
        let mut config = self.read_register(registers::PS_CONF1_L).await?;
        config = (config & !0x0E) | ((time as u16) << 1); // Clear bits 3:1 and set new value
        self.write_register(registers::PS_CONF1_L, config).await
    }

    /// Get proximity integration time
    pub async fn get_proximity_integration_time(&mut self) -> Result<ProximityIntegrationTime, Error<I2C::Error>> {
        let config = self.read_register(registers::PS_CONF1_L).await?;
        let time_value = (config >> 1) & 0x07; // Extract bits 3:1
        match time_value {
            0 => Ok(ProximityIntegrationTime::Time1T),
            1 => Ok(ProximityIntegrationTime::Time1_5T),
            2 => Ok(ProximityIntegrationTime::Time2T),
            3 => Ok(ProximityIntegrationTime::Time2_5T),
            4 => Ok(ProximityIntegrationTime::Time3T),
            5 => Ok(ProximityIntegrationTime::Time3_5T),
            6 => Ok(ProximityIntegrationTime::Time4T),
            7 => Ok(ProximityIntegrationTime::Time8T),
            _ => Err(Error::InvalidParameter),
        }
    }

    /// Enable proximity interrupts
    pub async fn enable_proximity_interrupts(&mut self, interrupt_type: ProximityInterruptType) -> Result<(), Error<I2C::Error>> {
        let mut config = self.read_register(registers::PS_CONF1_L).await?;
        config = (config & !0x0300) | ((interrupt_type as u16) << 8); // Clear bits 9:8 and set new value
        self.write_register(registers::PS_CONF1_L, config).await
    }

    /// Set proximity high resolution mode
    pub async fn set_proximity_high_resolution(&mut self, high_resolution: bool) -> Result<(), Error<I2C::Error>> {
        let mut config = self.read_register(registers::PS_CONF1_L).await?;
        if high_resolution {
            config |= 1 << 11; // Set bit 11 for high resolution
        } else {
            config &= !(1 << 11); // Clear bit 11 for normal resolution
        }
        self.write_register(registers::PS_CONF1_L, config).await
    }

    /// Get proximity high resolution mode
    pub async fn get_proximity_high_resolution(&mut self) -> Result<bool, Error<I2C::Error>> {
        let config = self.read_register(registers::PS_CONF1_L).await?;
        Ok((config & (1 << 11)) != 0)
    }

    /// Enable/disable ambient light interrupts
    pub async fn enable_ambient_light_interrupts(&mut self, enable: bool) -> Result<(), Error<I2C::Error>> {
        let mut config = self.read_register(registers::ALS_CONFIG).await?;
        if enable {
            config |= 0x02; // Set INT_EN bit
        } else {
            config &= !0x02; // Clear INT_EN bit
        }
        self.write_register(registers::ALS_CONFIG, config).await
    }

    /// Read a 16-bit register
    async fn read_register(&mut self, register: u8) -> Result<u16, Error<I2C::Error>> {
        let mut buffer = [0u8; 2];
        self.i2c.write_read(self.address, &[register], &mut buffer).await?;
        // VCNL4040 uses little-endian format
        Ok(u16::from_le_bytes(buffer))
    }

    /// Write a 16-bit register
    async fn write_register(&mut self, register: u8, value: u16) -> Result<(), Error<I2C::Error>> {
        let bytes = value.to_le_bytes();
        let buffer = [register, bytes[0], bytes[1]];
        self.i2c.write(self.address, &buffer).await?;
        Ok(())
    }

    /// Release the I2C bus
    pub fn release(self) -> I2C {
        self.i2c
    }
}