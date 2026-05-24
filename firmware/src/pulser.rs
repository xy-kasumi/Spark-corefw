// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! EDM pulser: stateful API over the I2C device driver. Owns no poll loop —
//! the orchestrator calls [`Pulser::tick`] on its 1 ms cadence.
#![allow(dead_code)]

use model::pstate;

use crate::drivers::pulser::{self, Bus};
use crate::line_tx;

/// EWMA coefficient for eff_duty: ~1 s time constant at 1 ms polling.
const EFF_DUTY_ALPHA: f32 = 0.001;

/// Retries for a critical register write before declaring the device lost.
const WRITE_RETRIES: u32 = 5;

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

/// Snapshot for the `stat` command: cached counters/rates plus a fresh read-back
/// of the config registers. `None` config fields mean that register read failed.
/// Built under the pulser lock so the caller can format and emit lines after
/// releasing it.
pub struct Stat {
    pub init_ok: bool,
    pub energized: bool,
    pub poll_count: u32,
    pub i2c_fail: u32,
    pub ratio: PulseRatio,
    pub pulse_current_a: Option<f32>,
    pub pulse_dur_us: Option<f32>,
    pub max_duty_pct: Option<f32>,
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
    init_ok: bool,
    energized: bool,
    /// Discard the first checkpoint after energize — it holds stale pre-energize data.
    first_after_energize: bool,
    last_ratio: PulseRatio,
    eff_duty: f32,
    poll_count: u32,
    num_i2c_fail: u32,
}

impl<B: Bus> Device<B> {
    pub fn new(dev: pulser::Device<B>) -> Self {
        Self {
            dev,
            init_ok: false,
            energized: false,
            first_after_energize: true,
            last_ratio: PulseRatio::ALL_OPEN,
            eff_duty: 0.0,
            poll_count: 0,
            num_i2c_fail: 0,
        }
    }

    /// Verify communication by reading a control register, emitting the
    /// `pulser.ok` line (and `pulser.msg` on failure) into the caller's open
    /// `init` p-state group. The caller owns the group's `begin`/`end`.
    pub async fn init(&mut self, line_tx: &line_tx::LineTx) -> bool {
        self.init_ok = self.dev.read_register(pulser::REG_POLARITY).await.is_ok();
        if self.init_ok {
            let _ =
                line_tx.try_send(pstate::Line::new(pstate::PsType::Init).bool("pulser.ok", true));
        } else {
            let _ =
                line_tx.try_send(pstate::Line::new(pstate::PsType::Init).bool("pulser.ok", false));
            let _ = line_tx.try_send(
                pstate::Line::new(pstate::PsType::Init).str_val("pulser.msg", "I2C read failed"),
            );
        }
        self.init_ok
    }

    /// Energize with the given config.
    ///
    /// `pulse_us` 100-1000, `current_a` 0-20 (0 → minimum), `duty_pct` 1-95.
    /// `tool_negative` selects tool-negative (polarity 2) vs tool-positive (polarity 1).
    pub async fn energize(&mut self, cfg: &Config) {
        let pulse_dur_10us = (cfg.pulse_us * 0.1) as u8;
        let mut pulse_current_100ma = (cfg.current_a * 10.0) as u8;
        if pulse_current_100ma == 0 {
            pulse_current_100ma = 1; // 100mA minimum
        }
        let duty = cfg.duty_pct as u8;
        let polarity = if cfg.tool_negative { 2 } else { 1 };

        // Polarity last, so all parameters are set before the hardware activates.
        self.write_with_retry(pulser::REG_PULSE_CURRENT, pulse_current_100ma)
            .await;
        self.write_with_retry(pulser::REG_PULSE_DUR, pulse_dur_10us)
            .await;
        self.write_with_retry(pulser::REG_MAX_DUTY, duty).await;
        self.write_with_retry(pulser::REG_POLARITY, polarity).await;

        self.energized = true;
        self.first_after_energize = true;
        self.eff_duty = 0.0;
    }

    pub async fn deenergize(&mut self) {
        self.energized = false;
        self.last_ratio = PulseRatio::ALL_OPEN;
        self.eff_duty = 0.0;
        self.write_with_retry(pulser::REG_POLARITY, 0).await;
    }

    /// One polling step: when energized, refresh the pulse/short/open rates and
    /// smoothed effective duty. Driven by the orchestrator at ~1 ms.
    pub async fn tick(&mut self) {
        if !self.energized {
            self.first_after_energize = true;
            self.poll_count += 1;
            return;
        }

        let (good, short) = match self.dev.read_ckp_ps().await {
            Some((val_p, val_s)) => (val_p as f32 / 15.0, val_s as f32 / 15.0),
            None => {
                self.num_i2c_fail += 1;
                return;
            }
        };

        if self.first_after_energize {
            self.first_after_energize = false;
        } else {
            self.last_ratio = PulseRatio {
                good,
                short,
                open: 1.0 - (good + short),
            };
            self.eff_duty += EFF_DUTY_ALPHA * (good - self.eff_duty);
        }
        self.poll_count += 1;
    }

    /// Latest pulse ratio. open=1 when non-energized.
    pub fn pulse_ratio(&self) -> PulseRatio {
        self.last_ratio
    }

    /// Smoothed effective duty [0, 1]; 0 when not energized.
    pub fn eff_duty(&self) -> f32 {
        if self.energized {
            self.eff_duty
        } else {
            0.0
        }
    }

    pub fn has_discharge(&self) -> bool {
        self.last_ratio.good > 0.0 || self.last_ratio.short > 0.0
    }

    pub fn energized(&self) -> bool {
        self.energized
    }

    /// Gather a [`PulserStat`] snapshot for the `stat` command. Reads the config
    /// registers fresh (the board, not this struct, holds them). Must finish
    /// before the caller emits lines, so no `line_tx` here — see [`PulserStat`].
    pub async fn read_stat(&mut self) -> Stat {
        if !self.init_ok {
            return Stat {
                init_ok: false,
                energized: false,
                poll_count: self.poll_count,
                i2c_fail: self.num_i2c_fail,
                ratio: PulseRatio::ALL_OPEN,
                pulse_current_a: None,
                pulse_dur_us: None,
                max_duty_pct: None,
            };
        }
        // Config registers are held by the board, not cached — read them back.
        let pulse_current_a = self
            .dev
            .read_register(pulser::REG_PULSE_CURRENT)
            .await
            .ok()
            .map(|v| v as f32 * 0.1);
        let pulse_dur_us = self
            .dev
            .read_register(pulser::REG_PULSE_DUR)
            .await
            .ok()
            .map(|v| v as f32 * 10.0);
        let max_duty_pct = self
            .dev
            .read_register(pulser::REG_MAX_DUTY)
            .await
            .ok()
            .map(|v| v as f32);
        Stat {
            init_ok: true,
            energized: self.energized,
            poll_count: self.poll_count,
            i2c_fail: self.num_i2c_fail,
            ratio: self.last_ratio,
            pulse_current_a,
            pulse_dur_us,
            max_duty_pct,
        }
    }

    /// Write a critical register, retrying briefly; a total failure marks the
    /// device state unknown.
    async fn write_with_retry(&mut self, reg: u8, val: u8) {
        for _ in 0..WRITE_RETRIES {
            if self.dev.write_register(reg, val).await.is_ok() {
                return;
            }
            self.num_i2c_fail += 1;
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
        }
        self.init_ok = false;
    }
}
