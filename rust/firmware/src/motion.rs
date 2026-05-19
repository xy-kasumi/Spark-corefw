//! Motion controller: owns the model-side MotionState plus the motor outputs,
//! and is ticked from the orchestrator on a 1 ms cadence.

use model::coords::PosPhys;
use model::motion::{Mode, MotionInputs, MotionState};

use crate::motor::Motors;

/// EDM path-buffer history capacity: 10 mm max retract at 0.005 mm resolution.
pub const PB_CAPACITY: usize = 2001;

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

    /// Raw microstep counters for the four wired axes (m0..m3 = x/y/z/c).
    pub fn motor_step_counts(&self) -> [i32; 4] {
        [
            self.motors.x.current(),
            self.motors.y.current(),
            self.motors.z.current(),
            self.motors.c.current(),
        ]
    }

    /// Advance the controller and apply the resulting target to motors.
    pub fn tick(&mut self, dt_s: f32) {
        if let Ok(out) = self.state.tick(MotionInputs { dt: dt_s }) {
            self.motors.set_target(out.target);
        }
    }

    /// Update per-axis step calibration from `m.<i>.unitsteps`.
    /// Index map: 0→x, 1→y, 2→z, 3→c. m4..m6 are ignored (no motion target).
    pub fn set_motor_unitsteps(&mut self, motor_idx: u8, value: f32) {
        match motor_idx {
            0 => self.motors.cal.steps_per_mm_x = value,
            1 => self.motors.cal.steps_per_mm_y = value,
            2 => self.motors.cal.steps_per_mm_z = value,
            3 => self.motors.cal.steps_per_turn_c = value,
            _ => {}
        }
    }

    /// Abort current motion and snap motor targets to their currently reached positions.
    pub fn cancel(&mut self) {
        let here = self.motors.current();
        self.state.cancel(here);
        self.motors.set_target(here);
    }
}
