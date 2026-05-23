// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Wire feed: integrates a feedrate into a wire-spool position, advanced one
//! step per tick.

/// 1 ms tick period (the orchestrator ticks at 1 kHz).
const TICK_PERIOD_S: f32 = 0.001;

#[derive(Default)]
pub struct Wirefeed {
    feeding: bool,
    pos_mm: f32,
    mm_per_tick: f32,
    rate_mm_per_min: f32,
}

impl Wirefeed {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn start(&mut self, rate_mm_per_min: f32) {
        self.rate_mm_per_min = rate_mm_per_min;
        self.mm_per_tick = (rate_mm_per_min / 60.0) * TICK_PERIOD_S;
        self.feeding = true;
    }

    pub fn stop(&mut self) {
        self.feeding = false;
    }

    /// Advance one tick. Returns the new wire position in mm when feeding, else
    /// `None` (no advance this tick).
    pub fn tick(&mut self) -> Option<f32> {
        if !self.feeding {
            return None;
        }
        self.pos_mm += self.mm_per_tick;
        Some(self.pos_mm)
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
