// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Driver for https://github.com/xy-kasumi/Spark-pulser/

#![allow(dead_code)]

// --- Bus ---------------------------------------------------------------------

/// Chip-independent I2C bus representation.
pub trait Bus {
    type Error;
    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Self::Error>;
    async fn write_read(&mut self, addr: u8, tx: &[u8], rx: &mut [u8]) -> Result<(), Self::Error>;
}

// --- Device ------------------------------------------------------------------

/// 7-bit I2C address of the pulser board.
pub const I2C_ADDR: u8 = 0x3c;

pub const REG_CTRL: u8 = 0x01; // RW: bit0 `run` (1 runs, 0 stops)
pub const REG_MODE: u8 = 0x02; // RW: bit0 `mode` (0=probe, 1=cut). Write fails while running.
pub const REG_CURR: u8 = 0x03; // RW: pulse current in A; auto-clamped to a supported value. Write fails while running.
pub const REG_DUR: u8 = 0x04; // RW: `reserved:1 | exp:2 | frac:5` pulse duration. Auto-clamped per CURR. Write fails while running.
pub const REG_DUTY: u8 = 0x05; // RW: max duty = `(byte+1)/256`. Auto-clamped per CURR & DUR. Write fails while running.
pub const REG_RES0: u8 = 0x08; // R: result byte 0; reading clears the WDT & updates the result.
pub const REG_RES1: u8 = 0x09; // R: result byte 1 (`num_good` in cut mode).
pub const REG_FAULT: u8 = 0x10; // R + clear: bit0 `fault`, bit1 `wdt` (write 1 to clear `wdt`).

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

    /// Cut-mode result `(fault, open, short, num_good)`: `open`/`short` window
    /// ratios in `[0, 1]`, `num_good` good pulses since the last read. Atomic
    /// `RES0`+`RES1` read; resets the WDT. `None` on bus error or `r_open+r_short>7`.
    pub async fn read_res_cut(&mut self) -> Option<(bool, f32, f32, u8)> {
        let mut buf = [0u8; 2];
        self.bus
            .write_read(I2C_ADDR, &[REG_RES0], &mut buf)
            .await
            .ok()?;
        let (res0, num_good) = (buf[0], buf[1]);
        let fault = res0 & 0x80 != 0;
        let r_open = (res0 >> 3) & 0x7;
        let r_short = res0 & 0x7;
        if r_open + r_short > 7 {
            return None;
        }
        Some((fault, r_open as f32 / 7.0, r_short as f32 / 7.0, num_good))
    }

    /// Probe-mode result `(fault, detected)`. Resets the WDT; `None` on bus error.
    pub async fn read_res_probe(&mut self) -> Option<(bool, bool)> {
        let res0 = self.read_register(REG_RES0).await.ok()?;
        Some((res0 & 0x80 != 0, res0 & 1 != 0))
    }
}

/// Pack `DUR` = `reserved:1 | exp:2 | frac:5`, where duration = `mul(exp) * frac/20`
/// with `mul` 10/100/1000us and `frac` in `1..=19`. Picks the finest exponent that
/// covers `pulse_us`; the device re-clamps to the active current's range.
/// e.g. 0.5us->0x01, 100us->0x42, 950us->0x53.
pub fn pack_dur(pulse_us: f32) -> u8 {
    const MUL: [f32; 3] = [10.0, 100.0, 1000.0];
    for exp in 0u8..=2 {
        let frac = ((pulse_us * 20.0 / MUL[exp as usize]) + 0.5) as i32;
        if frac <= 19 {
            return (exp << 5) | frac.clamp(1, 19) as u8;
        }
    }
    (2 << 5) | 19 // 950us, the maximum.
}

/// Pack `DUTY`: max duty = `(byte+1)/256`. Floors so the cap never exceeds `duty_pct`;
/// the device re-clamps to the active current & duration band.
/// e.g. 9%->0x16, 25%->0x3f, 49%->0x7c.
pub fn pack_duty(duty_pct: f32) -> u8 {
    ((duty_pct / 100.0 * 256.0) as i32 - 1).clamp(0, 255) as u8
}
