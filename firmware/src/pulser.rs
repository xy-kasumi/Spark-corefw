// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! EDM pulser: stateful API over the I2C device driver, mirroring the C
//! `pulser.h` surface. Unlike the C version it owns no poll loop — the
//! orchestrator calls [`Pulser::tick`] on its 1 ms cadence.
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
/// EDM rates are normalized to [0, 1]. Built under the pulser lock so the caller
/// can format and emit lines after releasing it.
pub struct Stat {
    pub init_ok: bool,
    pub energized: bool,
    pub poll_count: u32,
    pub i2c_fail: u32,
    pub r_pulse: f32,
    pub r_short: f32,
    pub r_open: f32,
    pub temp_c: Option<u8>,
    pub pulse_current_a: Option<f32>,
    pub pulse_dur_us: Option<f32>,
    pub max_duty_pct: Option<f32>,
}

pub struct Device<B: Bus> {
    dev: pulser::Device<B>,
    init_ok: bool,
    energized: bool,
    /// Discard the first checkpoint after energize — it holds stale pre-energize data.
    first_after_energize: bool,
    last_r_pulse: u8,
    last_r_short: u8,
    last_r_open: u8,
    last_temp: u8,
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
            last_r_pulse: 0,
            last_r_short: 0,
            last_r_open: 255,
            last_temp: 0,
            eff_duty: 0.0,
            poll_count: 0,
            num_i2c_fail: 0,
        }
    }

    /// Verify communication by reading the temperature register, emitting the
    /// `pulser.ok` line (and `pulser.msg` on failure) into the caller's open
    /// `init` p-state group. The caller owns the group's `begin`/`end`.
    pub async fn init(&mut self, line_tx: &line_tx::LineTx) -> bool {
        match self.dev.read_register(pulser::REG_TEMPERATURE).await {
            Ok(temp) => {
                self.last_temp = temp;
                self.init_ok = true;
            }
            Err(_) => {
                self.init_ok = false;
            }
        }
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

    /// Energize with the given parameters.
    ///
    /// `pulse_us` 100-1000, `current_a` 0-20 (0 → minimum), `duty_pct` 1-95.
    /// `negative` selects tool-negative (polarity 2) vs tool-positive (polarity 1).
    pub async fn energize(&mut self, negative: bool, pulse_us: f32, current_a: f32, duty_pct: f32) {
        let pulse_dur_10us = (pulse_us * 0.1) as u8;
        let mut pulse_current_100ma = (current_a * 10.0) as u8;
        if pulse_current_100ma == 0 {
            pulse_current_100ma = 1; // 100mA minimum
        }
        let duty = duty_pct as u8;
        let polarity = if negative { 2 } else { 1 };

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
        self.last_r_pulse = 0;
        self.last_r_short = 0;
        self.last_r_open = 255; // all-open when not energized
        self.eff_duty = 0.0;
        self.write_with_retry(pulser::REG_POLARITY, 0).await;
    }

    /// One polling step: refresh temperature, and when energized refresh the
    /// pulse/short/open rates and smoothed effective duty. Driven by the
    /// orchestrator at ~1 ms.
    pub async fn tick(&mut self) {
        let temp = match self.dev.read_register(pulser::REG_TEMPERATURE).await {
            Ok(v) => v,
            Err(_) => {
                self.num_i2c_fail += 1;
                return;
            }
        };
        self.last_temp = temp;

        if !self.energized {
            self.first_after_energize = true;
            self.poll_count += 1;
            return;
        }

        let val_ps = match self.dev.read_register(pulser::REG_CKP_PS).await {
            Ok(v) => v,
            Err(_) => {
                self.num_i2c_fail += 1;
                return;
            }
        };
        let val_p = (val_ps >> 4) & 0xf;
        let val_s = val_ps & 0xf;
        if val_p + val_s > 15 {
            // Out of protocol range — treat noise like a comm failure.
            self.num_i2c_fail += 1;
            return;
        }

        if self.first_after_energize {
            self.first_after_energize = false;
        } else {
            self.last_r_pulse = (val_p as u16 * 255 / 15) as u8;
            self.last_r_short = (val_s as u16 * 255 / 15) as u8;
            self.last_r_open = ((15 - (val_p + val_s)) as u16 * 255 / 15) as u8;
            self.eff_duty += EFF_DUTY_ALPHA * (val_p as f32 / 15.0 - self.eff_duty);
        }
        self.poll_count += 1;
    }

    /// Latest short rate (0-255); >127 typically indicates retraction needed.
    pub fn short_rate(&self) -> u8 {
        self.last_r_short
    }

    /// Latest pulse rate (0-255).
    pub fn pulse_rate(&self) -> u8 {
        self.last_r_pulse
    }

    /// Latest open rate (0-255).
    pub fn open_rate(&self) -> u8 {
        self.last_r_open
    }

    /// Latest heatsink temperature (°C).
    pub fn temp(&self) -> u8 {
        self.last_temp
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
        self.last_r_pulse > 0 || self.last_r_short > 0
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
                r_pulse: 0.0,
                r_short: 0.0,
                r_open: 0.0,
                temp_c: None,
                pulse_current_a: None,
                pulse_dur_us: None,
                max_duty_pct: None,
            };
        }
        // Config registers are held by the board, not cached — read them back.
        let temp_c = self.dev.read_register(pulser::REG_TEMPERATURE).await.ok();
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
            r_pulse: self.last_r_pulse as f32 / 255.0,
            r_short: self.last_r_short as f32 / 255.0,
            r_open: self.last_r_open as f32 / 255.0,
            temp_c,
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
