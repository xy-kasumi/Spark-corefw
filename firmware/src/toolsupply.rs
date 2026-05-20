//! Tool supply: a 50 Hz servo whose pulse width selects open vs. closed. Moves
//! are ramped over ~1 s so the servo travels smoothly.

use embassy_stm32::peripherals::TIM1;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::SimplePwm;
use embassy_time::{Duration, Timer};
use model::gcode::ToolSupplyState;

/// Servo carrier period (50 Hz). Pulse widths are fractions of this.
const PERIOD_US: u16 = 20_000;

pub struct ToolSupply {
    pwm: SimplePwm<'static, TIM1>,
    open_ms: f32,
    closed_ms: f32,
    current_ms: f32,
    current_state: ToolSupplyState,
}

impl ToolSupply {
    pub fn new(mut pwm: SimplePwm<'static, TIM1>, open_ms: f32, closed_ms: f32) -> Self {
        pwm.set_frequency(Hertz(50));
        pwm.ch1().enable();
        Self {
            pwm,
            open_ms,
            closed_ms,
            current_ms: closed_ms,
            current_state: ToolSupplyState::Closed,
        }
    }

    /// Apply the current servo position once (call after construction).
    pub fn init(&mut self) {
        self.set_servo(self.current_ms);
    }

    fn set_servo(&mut self, on_ms: f32) {
        let on_us = (on_ms * 1000.0) as u16;
        self.pwm.ch1().set_duty_cycle_fraction(on_us, PERIOD_US);
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
