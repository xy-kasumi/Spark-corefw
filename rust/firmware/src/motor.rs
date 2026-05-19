//! Per-axis motor abstraction. Converts mm/turn targets to microsteps and feeds step_gen.

use model::coords::PosPhys;

use crate::board::Step;

/// Steps-per-mm calibration for each linear axis; steps-per-turn for C.
/// Populated from settings in Phase 4.
#[derive(Clone, Copy, Debug)]
pub struct Calibration {
    pub steps_per_mm_x: f32,
    pub steps_per_mm_y: f32,
    pub steps_per_mm_z: f32,
    pub steps_per_turn_c: f32,
}

pub struct Motors {
    pub x: Step,
    pub y: Step,
    pub z: Step,
    pub c: Step,
    pub cal: Calibration,
}

impl Motors {
    pub fn set_target(&self, pos: PosPhys) {
        self.x.set_target((pos.x * self.cal.steps_per_mm_x) as i32);
        self.y.set_target((pos.y * self.cal.steps_per_mm_y) as i32);
        self.z.set_target((pos.z * self.cal.steps_per_mm_z) as i32);
        self.c.set_target((pos.c * self.cal.steps_per_turn_c) as i32);
    }

    pub fn current(&self) -> PosPhys {
        PosPhys {
            x: self.x.current() as f32 / self.cal.steps_per_mm_x,
            y: self.y.current() as f32 / self.cal.steps_per_mm_y,
            z: self.z.current() as f32 / self.cal.steps_per_mm_z,
            c: self.c.current() as f32 / self.cal.steps_per_turn_c,
        }
    }
}
