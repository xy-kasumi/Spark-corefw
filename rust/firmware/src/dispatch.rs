//! Application layer: handle classified frames, parse payloads, drive motion,
//! emit wire replies. Comm hands us classified bytes; we know what they mean.

use model::coords::PosPhys;
use model::gcode::{self, Command, MoveSpec, ParseError};

use crate::motion::Motion;
use crate::serial::Serial;

/// Fixed rapid speed (mm/s). G0 in C uses a hardcoded fast feed; no F input.
const RAPID_SPEED_MM_PER_S: f32 = 100.0;

pub fn signal(bytes: &[u8], motion: &mut Motion, serial: &Serial) {
    match bytes {
        b"!" => {
            motion.cancel();
            serial.tx_push(b"cancelled\r\n");
        }
        _ => {
            // ?queue / ?pos / etc. — Phase 4.
            serial.tx_push(b"err unknown-signal\r\n");
        }
    }
}

pub fn command(bytes: &[u8], motion: &mut Motion, serial: &Serial) {
    match gcode::parse(bytes) {
        Ok(cmd) => {
            exec(cmd, motion);
            serial.tx_push(b"ok\r\n");
        }
        Err(e) => {
            serial.tx_push(b"err ");
            serial.tx_push(err_name(e));
            serial.tx_push(b"\r\n");
        }
    }
}

fn exec(cmd: Command, motion: &mut Motion) {
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

fn err_name(e: ParseError) -> &'static [u8] {
    match e {
        ParseError::Empty => b"empty",
        ParseError::UnknownCommand => b"unknown",
        ParseError::BadAxis => b"bad-axis",
        ParseError::BadNumber => b"bad-number",
        ParseError::ExpectedSeparator => b"missing-sep",
        ParseError::TrailingGarbage => b"trailing",
    }
}
