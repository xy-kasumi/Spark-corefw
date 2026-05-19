//! Motion controller: owns the model-side MotionState plus the motor outputs,
//! and is ticked on a 1 ms cadence by the firmware.

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use model::coords::PosPhys;
use model::motion::{MotionInputs, MotionState, Mode};

use crate::motor::Motors;

/// History capacity for the EDM path buffer.
///
/// Phase 3 keeps this small so `Motion` fits on the default task stack
/// (51 * 16 B ≈ 800 B). Phase 4 should bump to ~2001 (10 mm retract) once
/// `Motion` is moved into a `StaticCell`-backed static.
pub const PB_CAPACITY: usize = 51;

pub type Shared = Mutex<NoopRawMutex, Motion>;

pub struct Motion {
    state: MotionState<PB_CAPACITY>,
    motors: Motors,
}

impl Motion {
    pub fn new(motors: Motors) -> Self {
        let start = motors.current();
        Self {
            state: MotionState::new(start),
            motors,
        }
    }

    pub fn current_position(&self) -> PosPhys {
        self.motors.current()
    }

    pub fn state(&mut self) -> &mut MotionState<PB_CAPACITY> {
        &mut self.state
    }

    pub fn mode(&self) -> Mode {
        self.state.mode()
    }

    /// Advance the controller and apply the resulting target to motors.
    pub fn tick(&mut self, dt_s: f32) {
        if let Ok(out) = self.state.tick(MotionInputs { dt: dt_s }) {
            self.motors.set_target(out.target);
        }
    }

    /// Abort current motion and snap motor targets to their currently reached positions.
    pub fn cancel(&mut self) {
        let here = self.motors.current();
        self.state.cancel(here);
        self.motors.set_target(here);
    }
}
