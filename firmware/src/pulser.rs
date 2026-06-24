// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! stateful API for pulser over the I2C device driver.

#![allow(dead_code)]

use crate::drivers::pulser::{self, Bus};

/// EWMA coefficient for the control ratio: ~200 ms time constant at 1 ms polling.
const CONTROL_RATIO_ALPHA: f32 = 0.08;
/// EWMA coefficient for the reporting ratio: ~1 s time constant at 1 ms polling.
const REPORT_RATIO_ALPHA: f32 = 0.001;

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

/// What the hardware should be doing while energized. (Whether to energize at all
/// is the separate `Option` in [`Device::desired`].)
#[derive(Clone, Copy)]
enum Request {
    /// Iso-pulse cutting with the given parameters.
    Cut(Config),
    /// Probe: minimum-energy pulses that auto-halt on contact.
    Probe,
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

/// Pulser measurement. `open`/`short` are window-count ratios driving the control
/// loop; `eff_duty` is the discharge time-fraction for `?edm` (so they don't sum to 1).
#[derive(Clone, Copy)]
pub struct PulseRatio {
    pub eff_duty: f32,
    pub short: f32,
    pub open: f32,
}

impl PulseRatio {
    /// Resting state when not discharging: no duty, all windows open.
    pub const ALL_OPEN: Self = Self {
        eff_duty: 0.0,
        short: 0.0,
        open: 1.0,
    };
}

pub struct Device<B: Bus> {
    dev: pulser::Device<B>,
    fault: bool,
    /// `Some` requests Energized; `None` requests Deenergized. Set sync;
    /// the async [`Self::tick`] reconciles the hardware toward this.
    desired: Option<Request>,
    /// Whether the hardware is currently energized (last successful transition).
    current_energized: bool,
    /// True once a probe contact has been detected (latched until deenergize).
    probe_detected: bool,
    /// Raw last-tick ratio. Drives discharge detection.
    last_ratio: PulseRatio,
    /// ~50 ms EWMA ratio. Consumed by the motion control loop.
    control_ratio: PulseRatio,
    /// ~1 s EWMA ratio. Consumed by `?edm` reporting.
    report_ratio: PulseRatio,
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
            probe_detected: false,
            last_ratio: PulseRatio::ALL_OPEN,
            control_ratio: PulseRatio::ALL_OPEN,
            report_ratio: PulseRatio::ALL_OPEN,
            num_i2c_write: 0,
            num_i2c_write_fail: 0,
            num_i2c_read: 0,
            num_i2c_read_fail: 0,
        }
    }

    /// Probe comm; on success clears `fault` unless the device latched a fault.
    /// Inspect [`Self::fault`] afterward.
    pub async fn init(&mut self) {
        // Check comm. Wait up to 500ms (pulser power bring up might take time).
        for _ in 0..5 {
            // Liveness via a side-effect-free CTRL read.
            if self.read_reg_counted(pulser::REG_CTRL).await.is_ok() {
                // Clear a stale watchdog latch, then check the latched fault bit.
                let _ = self.dev.write_register(pulser::REG_FAULT, 0b10).await;
                if let Ok(f) = self.dev.read_register(pulser::REG_FAULT).await {
                    self.fault = f & 1 != 0;
                    return;
                }
            }
            embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        }
    }

    pub fn fault(&self) -> bool {
        self.fault
    }

    /// Request a cut (iso-pulse) run with `cfg`. Next [`Self::tick`] energizes.
    pub fn request_cut(&mut self, cfg: &Config) {
        self.desired = Some(Request::Cut(*cfg));
    }

    /// Request a probe run (minimum-energy pulses, hardware auto-halt on contact).
    /// Next [`Self::tick`] energizes.
    pub fn request_probe(&mut self) {
        self.desired = Some(Request::Probe);
    }

    /// Request that the hardware be de-energized.
    /// Next [`Self::tick`] performs the I²C writes.
    pub fn request_deenergize(&mut self) {
        self.desired = None;
    }

    /// One polling step, `dt_s` seconds since the previous one (the tick period).
    /// Execute pending energize state reconciliation & do stats update (if energized).
    pub async fn tick(&mut self, dt_s: f32) {
        match self.desired {
            Some(req) if !self.current_energized => self.energize(req).await,
            None if self.current_energized => self.deenergize().await,
            Some(req) => self.poll(req, dt_s).await,
            None => {} // Idle.
        }
    }

    /// Transition Deenergized → Energized. Mode/current/timing are written first
    /// (the hardware rejects them while running), then `run` activates last.
    async fn energize(&mut self, req: Request) {
        let ok = match req {
            Request::Cut(cfg) => {
                let dur = pulser::pack_dur(cfg.pulse_us);
                let duty = pulser::pack_duty(cfg.duty_pct);
                // No polarity register: `cfg.tool_negative` ignored. Device clamps current.
                // Order CURR->DUR->DUTY matches the device's clamp dependency chain.
                self.write_reg_counted(pulser::REG_MODE, 1).await.is_ok() // cut
                    && self
                        .write_reg_counted(pulser::REG_CURR, cfg.current_a as u8)
                        .await
                        .is_ok()
                    && self.write_reg_counted(pulser::REG_DUR, dur).await.is_ok()
                    && self.write_reg_counted(pulser::REG_DUTY, duty).await.is_ok()
            }
            Request::Probe => self.write_reg_counted(pulser::REG_MODE, 0).await.is_ok(), // probe
        };
        if !ok || self.write_reg_counted(pulser::REG_CTRL, 1).await.is_err() {
            return;
        }
        self.current_energized = true;
        self.probe_detected = false;
        self.last_ratio = PulseRatio::ALL_OPEN;
        self.control_ratio = PulseRatio::ALL_OPEN;
        self.report_ratio = PulseRatio::ALL_OPEN;
    }

    /// Transition Energized → Deenergized. Commits state only after the halt write
    /// succeeds; on failure, stays energized and retries (the watchdog halts HW anyway).
    async fn deenergize(&mut self) {
        if self.write_reg_counted(pulser::REG_CTRL, 0).await.is_err() {
            return;
        }
        self.current_energized = false;
        self.probe_detected = false;
        self.last_ratio = PulseRatio::ALL_OPEN;
        self.control_ratio = PulseRatio::ALL_OPEN;
        self.report_ratio = PulseRatio::ALL_OPEN;
    }

    /// Steady-state poll: refresh measurements and reset the watchdog.
    async fn poll(&mut self, req: Request, dt_s: f32) {
        match req {
            Request::Cut(cfg) => self.poll_cut(cfg.pulse_us, dt_s).await,
            Request::Probe => self.poll_probe().await,
        }
    }

    /// Cut poll: refresh raw, control, and reporting open/short window ratios and `eff_duty`.
    async fn poll_cut(&mut self, pulse_us: f32, dt_s: f32) {
        self.num_i2c_read += 1;
        let (fault, open, short, num_good) = match self.dev.read_res_cut().await {
            Some(v) => v,
            None => {
                self.num_i2c_read_fail += 1;
                return;
            }
        };
        if fault {
            self.fault = true;
        }
        // eff_duty = pulse_dur * num_good / elapsed; `num_good` is read-and-clear per tick.
        let dt_us = dt_s * 1.0e6;
        let eff_duty = if dt_us > 0.0 {
            (pulse_us * num_good as f32 / dt_us).min(1.0)
        } else {
            0.0
        };
        let raw = PulseRatio {
            eff_duty,
            short,
            open,
        };
        self.last_ratio = raw;
        self.control_ratio = ema_ratio(self.control_ratio, raw, CONTROL_RATIO_ALPHA);
        self.report_ratio = ema_ratio(self.report_ratio, raw, REPORT_RATIO_ALPHA);
    }

    /// Probe poll: latch contact detection and reset the watchdog.
    async fn poll_probe(&mut self) {
        self.num_i2c_read += 1;
        match self.dev.read_res_probe().await {
            Some((fault, detected)) => {
                if fault {
                    self.fault = true;
                }
                self.probe_detected |= detected;
            }
            None => self.num_i2c_read_fail += 1,
        }
    }

    /// ~50 ms EWMA ratio. For the motion control loop. open=1 when non-energized.
    pub fn control_ratio(&self) -> PulseRatio {
        self.control_ratio
    }

    /// ~1 s EWMA ratio. For `?edm` reporting. open=1 when non-energized.
    pub fn report_ratio(&self) -> PulseRatio {
        self.report_ratio
    }

    /// Whether the gap is conducting (probe contact, or any cut pulse).
    pub fn has_discharge(&self) -> bool {
        match self.desired {
            Some(Request::Probe) => self.probe_detected,
            _ => self.last_ratio.eff_duty > 0.0 || self.last_ratio.short > 0.0,
        }
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

/// Field-wise [`ema`] of a [`PulseRatio`].
fn ema_ratio(cum: PulseRatio, new: PulseRatio, alpha: f32) -> PulseRatio {
    PulseRatio {
        eff_duty: ema(cum.eff_duty, new.eff_duty, alpha),
        short: ema(cum.short, new.short, alpha),
        open: ema(cum.open, new.open, alpha),
    }
}
