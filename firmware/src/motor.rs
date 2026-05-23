// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Maps pos -> motor mapping.

use model::coords;

use crate::board;

const M_X: usize = 0;
const M_Y: usize = 1;
const M_Z: usize = 2;
const M_C: usize = 3;

const DEFAULT_UNITSTEPS: [f32; board::NUM_MOTORS] = [200.0; board::NUM_MOTORS];

/// Which motor index drives a linear axis.
pub fn axis_to_motor(axis: coords::Axis) -> usize {
    match axis {
        coords::Axis::X => M_X,
        coords::Axis::Y => M_Y,
        coords::Axis::Z => M_Z,
    }
}

pub struct Motors {
    step: [board::MotorStepping; board::NUM_MOTORS],
    /// Steps per +1 unit value. Linear motors take mm; C takes turns.
    unitsteps: [f32; board::NUM_MOTORS],
    /// Per-motor homing offset in raw steps, added on the step counter so the
    /// homed position reads as the configured origin. Only XYZ get re-anchored;
    /// C stays at 0.
    home_offset: [i32; board::NUM_MOTORS],
}

impl Motors {
    pub fn new(step: [board::MotorStepping; board::NUM_MOTORS]) -> Self {
        Self {
            step,
            unitsteps: DEFAULT_UNITSTEPS,
            home_offset: [0; board::NUM_MOTORS],
        }
    }

    pub fn set_target(&self, pos: coords::PosPhys) {
        self.step[M_X].set_target((pos.x * self.unitsteps[M_X]) as i32 + self.home_offset[M_X]);
        self.step[M_Y].set_target((pos.y * self.unitsteps[M_Y]) as i32 + self.home_offset[M_Y]);
        self.step[M_Z].set_target((pos.z * self.unitsteps[M_Z]) as i32 + self.home_offset[M_Z]);
        self.step[M_C].set_target((pos.c * self.unitsteps[M_C]) as i32);
    }

    pub fn current(&self) -> coords::PosPhys {
        coords::PosPhys {
            x: (self.step[M_X].current() - self.home_offset[M_X]) as f32 / self.unitsteps[M_X],
            y: (self.step[M_Y].current() - self.home_offset[M_Y]) as f32 / self.unitsteps[M_Y],
            z: (self.step[M_Z].current() - self.home_offset[M_Z]) as f32 / self.unitsteps[M_Z],
            c: self.step[M_C].current() as f32 / self.unitsteps[M_C],
        }
    }

    /// Re-anchor `axis` so its current physical reading becomes `origin_mm`, by
    /// setting the homing offset against the live raw step counter.
    pub fn reanchor(&mut self, axis: coords::Axis, origin_mm: f32) {
        let m = axis_to_motor(axis);
        self.home_offset[m] = self.step[m].current() - (origin_mm * self.unitsteps[m]) as i32;
    }

    pub fn set_unitsteps(&mut self, motor_idx: usize, value: f32) {
        if let Some(slot) = self.unitsteps.get_mut(motor_idx) {
            *slot = value;
        }
    }

    /// Set a single motor's target from a value in "unit" (mm or turns).
    /// Out-of-range index is ignored.
    pub fn set_motor_target(&self, motor_idx: usize, value_u: f32) {
        if let Some(step) = self.step.get(motor_idx) {
            step.set_target((value_u * self.unitsteps[motor_idx]) as i32);
        }
    }

    /// Raw microstep counters for all motors (m0..m6).
    pub fn step_counts(&self) -> [i32; board::NUM_MOTORS] {
        core::array::from_fn(|i| self.step[i].current())
    }
}
