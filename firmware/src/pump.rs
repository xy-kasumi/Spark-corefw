// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Coolant/dielectric pump: a single active-high GPIO with settle delays.

use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

pub struct Pump {
    gpio: Output<'static>,
    /// Last M8/M9 commanded state.
    commanded: bool,
    /// `fset ov.pump_en` override: while set, forces the pump on regardless of
    /// `commanded`. The GPIO follows `commanded || override_on`.
    override_on: bool,
}

impl Pump {
    pub fn new(gpio: Output<'static>) -> Self {
        Self {
            gpio,
            commanded: false,
            override_on: false,
        }
    }

    /// M8/M9: set the commanded state, then wait for the pump to settle: 1 s
    /// after starting, 100 ms after stopping (blocking).
    pub async fn set_enable(&mut self, enable: bool) {
        self.commanded = enable;
        self.apply();
        if enable {
            Timer::after(Duration::from_millis(1000)).await;
        } else {
            Timer::after(Duration::from_millis(100)).await;
        }
    }

    /// `fset ov.pump_en`: force the pump on (true) or hand control back to
    /// M8/M9 (false). Applied immediately with no settle delay, so it is safe
    /// to call from the tick loop.
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
        self.apply();
    }

    fn apply(&mut self) {
        if self.commanded || self.override_on {
            self.gpio.set_high();
        } else {
            self.gpio.set_low();
        }
    }
}
