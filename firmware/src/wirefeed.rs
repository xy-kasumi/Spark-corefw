// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Wire feed: integrates a feedrate into motor6's step target, one step per tick.

use crate::board::MotorStepping;

/// 1 ms tick period (the orchestrator ticks at 1 kHz).
const TICK_PERIOD_S: f32 = 0.001;

pub struct Wirefeed {
    step: MotorStepping,
    unitsteps: f32,
    feeding: bool,
    pos_mm: f32,
    mm_per_tick: f32,
    rate_mm_per_min: f32,
}

impl Wirefeed {
    pub fn new(step: MotorStepping, unitsteps: f32) -> Self {
        Self {
            step,
            unitsteps,
            feeding: false,
            pos_mm: 0.0,
            mm_per_tick: 0.0,
            rate_mm_per_min: 0.0,
        }
    }

    pub fn start(&mut self, rate_mm_per_min: f32) {
        self.rate_mm_per_min = rate_mm_per_min;
        self.mm_per_tick = (rate_mm_per_min / 60.0) * TICK_PERIOD_S;
        self.feeding = true;
    }

    pub fn stop(&mut self) {
        self.feeding = false;
    }

    pub fn set_unitsteps(&mut self, unitsteps: f32) {
        self.unitsteps = unitsteps;
    }

    /// Advance one tick: integrate position and update motor6's step target.
    pub fn tick(&mut self) {
        if !self.feeding {
            return;
        }
        self.pos_mm += self.mm_per_tick;
        self.step.set_target((self.pos_mm * self.unitsteps) as i32);
    }

    pub fn feeding(&self) -> bool {
        self.feeding
    }

    pub fn pos_mm(&self) -> f32 {
        self.pos_mm
    }

    pub fn rate(&self) -> f32 {
        self.rate_mm_per_min
    }
}
