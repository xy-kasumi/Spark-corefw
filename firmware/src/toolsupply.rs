// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Tool supply: a 50 Hz servo whose pulse width selects open vs. closed. Moves
//! are ramped over ~1 s so the servo travels smoothly.

use embassy_time::{Duration, Timer};
use model::gcode::ToolSupplyState;

use crate::drivers::pwm_output::PwmOutput;

/// Servo carrier period (50 Hz). Pulse widths are fractions of this.
const PERIOD_MS: f32 = 20.0;
const DEFAULT_OPEN_MS: f32 = 1.6;
const DEFAULT_CLOSED_MS: f32 = 1.3;

pub struct ToolSupply<P: PwmOutput> {
    pwm: P,
    open_ms: f32,
    closed_ms: f32,
    current_ms: f32,
    current_state: ToolSupplyState,
}

impl<P: PwmOutput> ToolSupply<P> {
    pub fn new(mut pwm: P) -> Self {
        pwm.init(PERIOD_MS);
        Self {
            pwm,
            open_ms: DEFAULT_OPEN_MS,
            closed_ms: DEFAULT_CLOSED_MS,
            current_ms: DEFAULT_CLOSED_MS,
            current_state: ToolSupplyState::Closed,
        }
    }

    /// Apply the current servo position once (call after construction).
    pub fn init(&mut self) {
        self.set_servo(self.current_ms);
    }

    fn set_servo(&mut self, on_ms: f32) {
        self.pwm.set(on_ms / PERIOD_MS);
    }

    fn target_ms(&self, state: ToolSupplyState) -> f32 {
        match state {
            ToolSupplyState::Open => self.open_ms,
            ToolSupplyState::Closed => self.closed_ms,
        }
    }

    /// Ramp the servo to `target` over 100 steps of 10 ms each (blocking, ~1 s).
    pub async fn set_state(&mut self, target: ToolSupplyState) {
        const NUM_CYCLES: u16 = 100;
        let src = self.current_ms;
        let dst = self.target_ms(target);
        for cycle in 1..=NUM_CYCLES {
            let t = cycle as f32 / NUM_CYCLES as f32;
            self.set_servo(src + t * (dst - src));
            Timer::after(Duration::from_millis(10)).await;
        }
        self.current_ms = dst;
        self.current_state = target;
    }

    /// Update one state's pulse width, then re-apply the active state (moves the servo).
    pub async fn configure(&mut self, state: ToolSupplyState, on_ms: f32) {
        match state {
            ToolSupplyState::Open => self.open_ms = on_ms,
            ToolSupplyState::Closed => self.closed_ms = on_ms,
        }
        self.set_state(self.current_state).await;
    }
}
