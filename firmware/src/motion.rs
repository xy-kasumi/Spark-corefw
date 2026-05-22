// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Motion controller: owns the model-side MotionState plus the motor outputs,
//! and is ticked from the orchestrator on a 1 ms cadence.

use model::coords;
use model::motion;
use model::settings;

use crate::motor;

/// Per-tick pulser feedback fed into the motion model (EDM advance/retract and
/// probe contact). Snapshotted by the orchestrator before ticking motion.
#[derive(Clone, Copy, Debug, Default)]
pub struct PulserFeedback {
    pub open_rate: u8,
    pub short_rate: u8,
    pub discharge: bool,
}

/// EDM path-buffer history capacity: 10 mm max retract at 0.005 mm resolution.
pub const PB_CAPACITY: usize = 2001;

/// Snapshot of motion state backing the `?edm` query. `has_edm_data` is set
/// only during an EDM-controlled move; `is_moving` covers any active move.
#[derive(Clone, Copy, Debug, Default)]
pub struct EdmState {
    pub has_edm_data: bool,
    pub is_moving: bool,
    pub forward_buffer: f32,
    pub backward_buffer: f32,
    pub distance: f32,
    pub distance_max: f32,
}

pub struct Motion {
    state: motion::MotionState<PB_CAPACITY>,
    motors: motor::Motors,
}

impl Motion {
    pub fn new(motors: motor::Motors) -> Self {
        let start = motors.current();
        Self {
            state: motion::MotionState::new(start),
            motors,
        }
    }

    pub fn current_position(&self) -> coords::PosPhys {
        self.motors.current()
    }

    pub fn state(&mut self) -> &mut motion::MotionState<PB_CAPACITY> {
        &mut self.state
    }

    pub fn mode(&self) -> motion::Mode {
        self.state.mode()
    }

    /// Motion-side fields for the `?edm` query. Pulser-side fields (eff_duty,
    /// rates, temp) are read separately from the pulser.
    pub fn edm_state(&self) -> EdmState {
        let mode = self.state.mode();
        EdmState {
            has_edm_data: mode == motion::Mode::EdmMove,
            is_moving: mode != motion::Mode::Idle,
            forward_buffer: self.state.forward_buffer(),
            backward_buffer: self.state.backward_buffer(),
            distance: self.state.distance(),
            distance_max: self.state.distance_max(),
        }
    }

    /// True when the running EDM move can accept another chained segment.
    pub fn can_enqueue(&self) -> bool {
        self.state.can_enqueue()
    }

    /// Re-anchor `axis` to `origin_mm` after a homing move: update the motor
    /// offset so the position reads `origin_mm`, then reset the controller to
    /// that new position.
    pub fn finish_home(&mut self, axis: settings::Axis, origin_mm: f32) {
        self.motors.reanchor(axis, origin_mm);
        let here = self.motors.current();
        self.state.set_position(here);
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
    pub fn tick(&mut self, dt_s: f32, fb: PulserFeedback) {
        let input = motion::MotionInputs {
            dt: dt_s,
            open_rate: fb.open_rate,
            short_rate: fb.short_rate,
            discharge: fb.discharge,
        };
        if let Ok(out) = self.state.tick(input) {
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
