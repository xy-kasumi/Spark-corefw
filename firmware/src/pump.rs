// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Coolant/dielectric pump: a single active-high GPIO with settle delays.

use crate::drivers::digital_out::Pin;

pub struct Pump<D: Pin> {
    pin_en: D,
    /// Last pump-on/off commanded state.
    commanded: bool,
    /// `fset ov.pump_en` override: while set, forces the pump on regardless of
    /// `commanded`. The GPIO follows `commanded || override_on`.
    override_on: bool,
}

impl<D: Pin> Pump<D> {
    pub fn new(pin_en: D) -> Self {
        Self {
            pin_en,
            commanded: false,
            override_on: false,
        }
    }

    /// Set the commanded state, then wait for the pump to settle (blocking).
    pub async fn set_enable(&mut self, enable: bool) {
        self.commanded = enable;
        self.apply();
        if enable {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
        } else {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(100)).await;
        }
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
        self.apply();
    }

    fn apply(&mut self) {
        self.pin_en.set(self.commanded || self.override_on);
    }
}
