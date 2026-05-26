// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Wire feed: integrates a feedrate into a wire-spool position, advanced one
//! step per tick.

/// 1 ms tick period (the orchestrator ticks at 1 kHz).
const TICK_PERIOD_S: f32 = 0.001;

/// Ticks for wire tension to stabilize after `start`.
const SETTLE_ON_TICKS: u16 = 2000;

#[derive(Default)]
pub struct Wirefeed {
    feeding: bool,
    pos_mm: f32,
    mm_per_tick: f32,
    rate_mm_per_min: f32,
    settle_ticks: u16,
}

impl Wirefeed {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn start(&mut self, rate_mm_per_min: f32) {
        self.rate_mm_per_min = rate_mm_per_min;
        self.mm_per_tick = (rate_mm_per_min / 60.0) * TICK_PERIOD_S;
        self.feeding = true;
        self.settle_ticks = SETTLE_ON_TICKS;
    }

    pub fn stop(&mut self) {
        self.feeding = false;
        self.settle_ticks = 0;
    }

    /// True when wire tension has stabilized after the most recent `start`.
    pub fn settled(&self) -> bool {
        self.settle_ticks == 0
    }

    /// Advance one tick. Returns the new wire position in mm when feeding, else
    /// `None` (no advance this tick).
    pub fn tick(&mut self) -> Option<f32> {
        self.settle_ticks = self.settle_ticks.saturating_sub(1);
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
