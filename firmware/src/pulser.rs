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
    /// `Some(cfg)` requests Energized; `None` requests Deenergized. Set sync;
    /// the async [`Self::tick`] reconciles the hardware toward this.
    desired: Option<Config>,
    /// Whether the hardware is currently energized (last successful transition).
    current_energized: bool,
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
            desired: None,
            current_energized: false,
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

    /// Request that the hardware be energized with `cfg`.
    /// Next [`Self::tick`] performs the I²C writes.
    pub fn request_energize(&mut self, cfg: &Config) {
        self.desired = Some(*cfg);
    }

    /// Request that the hardware be de-energized.
    /// Next [`Self::tick`] performs the I²C writes.
    pub fn request_deenergize(&mut self) {
        self.desired = None;
    }

    /// One polling step.
    /// Execute pending energize state reconciliation & do stats update (if energized).
    pub async fn tick(&mut self) {
        match self.desired {
            Some(cfg) if !self.current_energized => self.energize(cfg).await,
            None if self.current_energized => self.deenergize().await,
            Some(_) => self.poll().await,
            None => {} // Idle.
        }
    }

    /// Transition Deenergized → Energized. Polarity is written last so all
    /// parameters are set before the hardware activates.
    async fn energize(&mut self, cfg: Config) {
        let pulse_dur_10us = (cfg.pulse_us * 0.1) as u8;
        let mut pulse_current_100ma = (cfg.current_a * 10.0) as u8;
        if pulse_current_100ma == 0 {
            pulse_current_100ma = 1; // 100mA minimum
        }
        let duty = cfg.duty_pct as u8;
        let polarity = if cfg.tool_negative { 2 } else { 1 };
        if self
            .write_reg_counted(pulser::REG_PULSE_CURRENT, pulse_current_100ma)
            .await
            .is_err()
            || self
                .write_reg_counted(pulser::REG_PULSE_DUR, pulse_dur_10us)
                .await
                .is_err()
            || self
                .write_reg_counted(pulser::REG_MAX_DUTY, duty)
                .await
                .is_err()
            || self
                .write_reg_counted(pulser::REG_POLARITY, polarity)
                .await
                .is_err()
        {
            return;
        }
        self.current_energized = true;
        self.smoothed_ratio = PulseRatio::ALL_OPEN;
    }

    /// Transition Energized → Deenergized.
    async fn deenergize(&mut self) {
        self.current_energized = false;
        self.last_ratio = PulseRatio::ALL_OPEN;
        self.smoothed_ratio = PulseRatio::ALL_OPEN;
        let _ = self.write_reg_counted(pulser::REG_POLARITY, 0).await;
    }

    /// Steady-state poll: refresh raw and smoothed pulse/short/open ratios.
    async fn poll(&mut self) {
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
        self.current_energized
    }

    /// True when the hardware state matches the most recent request.
    pub fn settled(&self) -> bool {
        self.desired.is_some() == self.current_energized
    }

    /// Gather a [`Stat`] snapshot for the `stat` command.
    pub fn read_stat(&self) -> Stat {
        Stat {
            fault: self.fault,
            energized: self.current_energized,
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
