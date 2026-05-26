// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! stateful API for pulser over the I2C device driver.

#![allow(dead_code)]

use crate::drivers::pulser::{self, Bus};

/// EWMA coefficient for the smoothed pulse ratio: ~1 s time constant at 1 ms polling.
const RATIO_ALPHA: f32 = 0.001;

#[derive(Clone, Copy)]
pub struct Config {
    pub tool_negative: bool,
    pub pulse_us: f32,
    pub current_a: f32,
    pub duty_pct: f32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            tool_negative: true,
            pulse_us: 500.0,
            current_a: 1.0,
            duty_pct: 25.0,
        }
    }
}

/// Diagnostic stats.
pub struct Stat {
    pub fault: bool,
    pub energized: bool,
    pub i2c_write: u32,
    pub i2c_write_fail: u32,
    pub i2c_read: u32,
    pub i2c_read_fail: u32,
}

/// Pulse ratio. Each field is in [0, 1] and they add up to 1.
#[derive(Clone, Copy)]
pub struct PulseRatio {
    pub good: f32,
    pub short: f32,
    pub open: f32,
}

impl PulseRatio {
    /// All-open ratio — the resting state when not energized.
    pub const ALL_OPEN: Self = Self {
        good: 0.0,
        short: 0.0,
        open: 1.0,
    };
}

pub struct Device<B: Bus> {
    dev: pulser::Device<B>,
    fault: bool,
    energized: bool,
    /// Raw last-tick ratio. Consumed by the 1 ms control loop.
    last_ratio: PulseRatio,
    /// EWMA-smoothed ratio. Consumed by `?edm` reporting.
    smoothed_ratio: PulseRatio,
    num_i2c_write: u32,
    num_i2c_write_fail: u32,
    num_i2c_read: u32,
    num_i2c_read_fail: u32,
}

impl<B: Bus> Device<B> {
    pub fn new(dev: pulser::Device<B>) -> Self {
        Self {
            dev,
            fault: true,
            energized: false,
            last_ratio: PulseRatio::ALL_OPEN,
            smoothed_ratio: PulseRatio::ALL_OPEN,
            num_i2c_write: 0,
            num_i2c_write_fail: 0,
            num_i2c_read: 0,
            num_i2c_read_fail: 0,
        }
    }

    /// Probe comm; on success clears `fault`. Inspect [`Self::fault`] afterward.
    pub async fn init(&mut self) {
        // Check comm. Wait up to 500ms (pulser power bring up might take time).
        for _ in 0..5 {
            // Verify comm with safe register read.
            if self.read_reg_counted(pulser::REG_POLARITY).await.is_ok() {
                self.fault = false;
                return;
            }
            embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        }
    }

    pub fn fault(&self) -> bool {
        self.fault
    }

    /// Energize with the given config.
    ///
    /// `pulse_us` 100-1000, `current_a` 0-20 (0 → minimum), `duty_pct` 1-95.
    /// `tool_negative` selects tool-negative (polarity 2) vs tool-positive (polarity 1).
    ///
    /// Returns `Err(())` on any I²C write failure; caller should escalate to
    /// fault. `energized` is only set on the success path.
    pub async fn energize(&mut self, cfg: &Config) -> Result<(), ()> {
        let pulse_dur_10us = (cfg.pulse_us * 0.1) as u8;
        let mut pulse_current_100ma = (cfg.current_a * 10.0) as u8;
        if pulse_current_100ma == 0 {
            pulse_current_100ma = 1; // 100mA minimum
        }
        let duty = cfg.duty_pct as u8;
        let polarity = if cfg.tool_negative { 2 } else { 1 };

        // Polarity last, so all parameters are set before the hardware activates.
        self.write_reg_counted(pulser::REG_PULSE_CURRENT, pulse_current_100ma)
            .await?;
        self.write_reg_counted(pulser::REG_PULSE_DUR, pulse_dur_10us)
            .await?;
        self.write_reg_counted(pulser::REG_MAX_DUTY, duty).await?;
        self.write_reg_counted(pulser::REG_POLARITY, polarity)
            .await?;

        self.energized = true;
        self.smoothed_ratio = PulseRatio::ALL_OPEN;
        Ok(())
    }

    /// De-energize. Clears local state unconditionally; the returned `Err(())`
    /// surfaces an I²C failure so the caller can escalate to fault. From the
    /// cancel teardown path, callers ignore the error — fault entry has its
    /// own idempotent latch and a failed deenergize there shouldn't re-fire it.
    pub async fn deenergize(&mut self) -> Result<(), ()> {
        self.energized = false;
        self.last_ratio = PulseRatio::ALL_OPEN;
        self.smoothed_ratio = PulseRatio::ALL_OPEN;
        self.write_reg_counted(pulser::REG_POLARITY, 0).await
    }

    /// One polling step: when energized, refresh the pulse/short/open rates and
    /// smoothed effective duty. Driven by the orchestrator at ~1 ms.
    pub async fn tick(&mut self) {
        if !self.energized {
            return;
        }

        self.num_i2c_read += 1;
        let (good, short) = match self.dev.read_ckp_ps().await {
            Some((val_p, val_s)) => (val_p as f32 / 15.0, val_s as f32 / 15.0),
            None => {
                self.num_i2c_read_fail += 1;
                return;
            }
        };

        let raw = PulseRatio {
            good,
            short,
            open: 1.0 - (good + short),
        };
        self.last_ratio = raw;
        // EWMA is linear, so the smoothed components keep summing to 1.
        self.smoothed_ratio = PulseRatio {
            good: ema(self.smoothed_ratio.good, raw.good, RATIO_ALPHA),
            short: ema(self.smoothed_ratio.short, raw.short, RATIO_ALPHA),
            open: ema(self.smoothed_ratio.open, raw.open, RATIO_ALPHA),
        };
    }

    /// Raw last-tick ratio. For the 1 ms control loop. open=1 when non-energized.
    pub fn last_ratio(&self) -> PulseRatio {
        self.last_ratio
    }

    /// EWMA-smoothed ratio. For `?edm` reporting. open=1 when non-energized.
    pub fn smoothed_ratio(&self) -> PulseRatio {
        self.smoothed_ratio
    }

    pub fn has_discharge(&self) -> bool {
        self.last_ratio.good > 0.0 || self.last_ratio.short > 0.0
    }

    pub fn energized(&self) -> bool {
        self.energized
    }

    /// Gather a [`Stat`] snapshot for the `stat` command.
    pub fn read_stat(&self) -> Stat {
        Stat {
            fault: self.fault,
            energized: self.energized,
            i2c_write: self.num_i2c_write,
            i2c_write_fail: self.num_i2c_write_fail,
            i2c_read: self.num_i2c_read,
            i2c_read_fail: self.num_i2c_read_fail,
        }
    }

    async fn read_reg_counted(&mut self, reg: u8) -> Result<u8, ()> {
        self.num_i2c_read += 1;
        self.dev.read_register(reg).await.map_err(|_| {
            self.num_i2c_read_fail += 1;
        })
    }

    async fn write_reg_counted(&mut self, reg: u8, val: u8) -> Result<(), ()> {
        self.num_i2c_write += 1;
        self.dev.write_register(reg, val).await.map_err(|_| {
            self.num_i2c_write_fail += 1;
            self.fault = true;
        })
    }
}

/// Exponential moving average.
fn ema(cum: f32, new: f32, alpha: f32) -> f32 {
    cum + alpha * (new - cum)
}
