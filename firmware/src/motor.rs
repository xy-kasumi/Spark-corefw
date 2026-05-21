// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Per-axis motor abstraction. Converts mm/turn targets to microsteps and feeds step_gen.

use model::coords;
use model::settings;

use crate::board;

/// Steps-per-mm for linear axes; steps-per-turn for C.
#[derive(Clone, Copy, Debug)]
pub struct MotorAxisConfig {
    pub steps_per_mm_x: f32,
    pub steps_per_mm_y: f32,
    pub steps_per_mm_z: f32,
    pub steps_per_turn_c: f32,
}

impl Default for MotorAxisConfig {
    fn default() -> Self {
        Self {
            steps_per_mm_x: 200.0,
            steps_per_mm_y: 200.0,
            steps_per_mm_z: 200.0,
            steps_per_turn_c: 200.0,
        }
    }
}

pub struct Motors {
    pub x: board::MotorStepping,
    pub y: board::MotorStepping,
    pub z: board::MotorStepping,
    pub c: board::MotorStepping,
    pub cal: MotorAxisConfig,
    /// Per-axis homing offset in steps (x/y/z), added on the raw step counter so
    /// the homed position reads as the configured origin. C has no home offset.
    pub home_offset: [i32; 3],
}

impl Motors {
    pub fn set_target(&self, pos: coords::PosPhys) {
        self.x
            .set_target((pos.x * self.cal.steps_per_mm_x) as i32 + self.home_offset[0]);
        self.y
            .set_target((pos.y * self.cal.steps_per_mm_y) as i32 + self.home_offset[1]);
        self.z
            .set_target((pos.z * self.cal.steps_per_mm_z) as i32 + self.home_offset[2]);
        self.c
            .set_target((pos.c * self.cal.steps_per_turn_c) as i32);
    }

    pub fn current(&self) -> coords::PosPhys {
        coords::PosPhys {
            x: (self.x.current() - self.home_offset[0]) as f32 / self.cal.steps_per_mm_x,
            y: (self.y.current() - self.home_offset[1]) as f32 / self.cal.steps_per_mm_y,
            z: (self.z.current() - self.home_offset[2]) as f32 / self.cal.steps_per_mm_z,
            c: self.c.current() as f32 / self.cal.steps_per_turn_c,
        }
    }

    /// Re-anchor `axis` so its current physical reading becomes `origin_mm`, by
    /// setting the homing offset against the live raw step counter.
    pub fn reanchor(&mut self, axis: settings::Axis, origin_mm: f32) {
        let (raw, spm) = match axis {
            settings::Axis::X => (self.x.current(), self.cal.steps_per_mm_x),
            settings::Axis::Y => (self.y.current(), self.cal.steps_per_mm_y),
            settings::Axis::Z => (self.z.current(), self.cal.steps_per_mm_z),
        };
        self.home_offset[axis.idx()] = raw - (origin_mm * spm) as i32;
    }
}
