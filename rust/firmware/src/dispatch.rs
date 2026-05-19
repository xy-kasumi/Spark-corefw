//! Map parsed model::gcode::Command to motion/etc. side effects.

use model::coords::PosPhys;
use model::gcode::{Command, MoveSpec};

use crate::motion::Motion;

/// Fixed rapid speed (mm/s). G0 in C uses a hardcoded fast feed; no F input.
const RAPID_SPEED_MM_PER_S: f32 = 100.0;

pub fn exec(cmd: Command, motion: &mut Motion) {
    match cmd {
        Command::Rapid(spec) => exec_rapid(spec, motion),
        Command::Linear(_) => unimplemented!("Phase 4: G1 needs pulser feedback loop"),
    }
}

fn exec_rapid(spec: MoveSpec, motion: &mut Motion) {
    let current = motion.current_position();
    let target = apply_spec(current, &spec);
    motion.state().start_rapid(target, RAPID_SPEED_MM_PER_S);
}

fn apply_spec(current: PosPhys, s: &MoveSpec) -> PosPhys {
    PosPhys {
        x: s.x.unwrap_or(current.x),
        y: s.y.unwrap_or(current.y),
        z: s.z.unwrap_or(current.z),
        c: s.c.unwrap_or(current.c),
    }
}
