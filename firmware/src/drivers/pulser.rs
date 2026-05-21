// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! EDM pulser board I2C device: thin register-level I/O over an injected bus.
//!
//! Generic over any async I2C bus (see [`PulserBus`]); register semantics and
//! state live one layer up in `crate::pulser`.
#![allow(dead_code)]

// --- Bus ---------------------------------------------------------------------

/// Minimal async I2C transport. `board` injects a concrete bus (e.g. embassy's
/// `I2c`); method shapes mirror `embassy_stm32::i2c::I2c::write`/`write_read`.
pub trait Bus {
    type Error;
    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Self::Error>;
    async fn write_read(&mut self, addr: u8, tx: &[u8], rx: &mut [u8]) -> Result<(), Self::Error>;
}

// --- Device ------------------------------------------------------------------

/// 7-bit I2C address of the pulser board.
pub const I2C_ADDR: u8 = 0x3b;

// Registers (from https://github.com/xy-kasumi/Spark-pulser/blob/main/docs/user-manual.md).
pub const REG_POLARITY: u8 = 0x01; // RW: 0=OFF, 1-4=energize with polarity
pub const REG_PULSE_CURRENT: u8 = 0x02; // RW: pulse current in 100mA units (1-200)
pub const REG_TEMPERATURE: u8 = 0x03; // R:  heatsink temperature in °C
pub const REG_PULSE_DUR: u8 = 0x04; // RW: pulse duration in 10us units (5-100)
pub const REG_MAX_DUTY: u8 = 0x05; // RW: max duty factor in percent (1-95)
pub const REG_CKP_PS: u8 = 0x10; // R (special): rate of pulse & short

pub struct Device<B: Bus> {
    bus: B,
}

impl<B: Bus> Device<B> {
    pub fn new(bus: B) -> Self {
        Self { bus }
    }

    pub async fn read_register(&mut self, reg: u8) -> Result<u8, B::Error> {
        let mut buf = [0u8; 1];
        self.bus.write_read(I2C_ADDR, &[reg], &mut buf).await?;
        Ok(buf[0])
    }

    pub async fn write_register(&mut self, reg: u8, val: u8) -> Result<(), B::Error> {
        self.bus.write(I2C_ADDR, &[reg, val]).await
    }
}
