// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

use crate::drivers::digital_out::Pin;

/// Time before water actually start flowing (tube latency).
const SETTLE_ON_TICKS: u16 = 1000;
/// Time until water flow actually stops.
const SETTLE_OFF_TICKS: u16 = 100;

pub struct Pump<D: Pin> {
    pin_en: D,
    /// Last pump-on/off commanded state.
    commanded: bool,
    /// `fset ov.pump_en` override: while set, forces the pump on regardless of
    /// `commanded`. The GPIO follows `commanded || override_on`.
    override_on: bool,
    /// Ticks remaining until the most recent `set_enable` is considered settled.
    settle_ticks: u16,
}

impl<D: Pin> Pump<D> {
    pub fn new(pin_en: D) -> Self {
        Self {
            pin_en,
            commanded: false,
            override_on: false,
            settle_ticks: 0,
        }
    }

    /// Set the commanded state and arm the settle countdown. Apply the GPIO
    /// immediately; the caller polls [`settled`](Self::settled) to wait it out.
    pub fn set_enable(&mut self, enable: bool) {
        self.commanded = enable;
        self.settle_ticks = if enable {
            SETTLE_ON_TICKS
        } else {
            SETTLE_OFF_TICKS
        };
        self.apply();
    }

    /// True when the most recent `set_enable` has finished settling.
    pub fn settled(&self) -> bool {
        self.settle_ticks == 0
    }

    pub fn tick(&mut self) {
        self.settle_ticks = self.settle_ticks.saturating_sub(1);
    }

    /// `fset ov.pump_en`: force the pump on (true) or hand control back to
    /// the commanded state (false). Applied immediately with no settle delay, so it
    /// is safe to call from the tick loop.
    pub fn set_override(&mut self, on: bool) {
        self.override_on = on;
        self.apply();
    }

    /// Cancel: stop the pump and drop the `fset ov.pump_en` override, returning
    /// to the powered-off default. Applied immediately with no settle delay, so
    /// it is safe to call from the tick loop.
    pub fn cancel(&mut self) {
        self.commanded = false;
        self.override_on = false;
        self.settle_ticks = 0;
        self.apply();
    }

    fn apply(&mut self) {
        self.pin_en.set(self.commanded || self.override_on);
    }
}
