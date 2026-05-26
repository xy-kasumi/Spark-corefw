// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Query-signal executor. It just converts snapshot to text. Must finish within << tick duration (1ms).

use core::fmt::Write;
use core::sync::atomic;

use model::command;
use model::coords;
use model::coords::CoordSys;
use model::motion;
use model::pstate;

use crate::commands;
use crate::outbox;
use crate::pulser;

/// Snapshot of machine (enough to answer [`QuerySignal`])
#[derive(Clone, Copy)]
pub struct MachineStats {
    pub pos: coords::PosPhys,
    pub edm: Option<motion::EdmReport>,
    pub active: coords::CoordSys,
    pub offset: coords::PosPhys,
    pub smooth_pulse_ratio: pulser::PulseRatio,
}

pub fn exec_query<const N: usize>(
    sig: command::QuerySignal,
    stats: &MachineStats,
    cmd_queue: &commands::CmdQueue,
    out: &mut outbox::OutputBuf<N>,
) {
    match sig {
        command::QuerySignal::Queue => {
            let num = cmd_queue.len() + commands::OUTSTANDING.load(atomic::Ordering::Relaxed);
            out.push(
                pstate::Line::new(pstate::PsType::Queue)
                    .int("cap", commands::CMD_QUEUE_CAP as i32)
                    .int("num", num as i32),
            );
        }
        command::QuerySignal::Pos => {
            let pos = stats.pos;
            let active = stats.active;
            let off = stats.offset;

            let mut line = pstate::Line::new(pstate::PsType::Pos)
                .str_val("sys", sys_name(active))
                .float("m.x", pos.x)
                .float("m.y", pos.y)
                .float("m.z", pos.z)
                .float("m.c", pos.c * 360.0);
            if active != CoordSys::Machine {
                let cs = pos.with_offset_removed(off);
                let p = pos_prefix(active);
                line = line
                    .float(&key(p, 'x'), cs.x)
                    .float(&key(p, 'y'), cs.y)
                    .float(&key(p, 'z'), cs.z)
                    .float(&key(p, 'c'), cs.c * 360.0);
            }
            out.push(line);
        }
        command::QuerySignal::Edm => {
            // Motion and pulser fields come from the same tick snapshot.
            let mut line = pstate::Line::new(pstate::PsType::Edm);
            if let Some(edm) = stats.edm {
                line = line
                    .float("eff_duty", stats.smooth_pulse_ratio.good)
                    .float("open", stats.smooth_pulse_ratio.open)
                    .float("short", stats.smooth_pulse_ratio.short)
                    .float("retr_rem", edm.retract_remaining)
                    .float("dist", edm.distance)
                    .float("dist_max", edm.distance_max);
            }
            out.push(line);
        }
        command::QuerySignal::Unknown => {
            // Recognized signal byte, unknown verb: ignore to avoid clogging the stream.
        }
    }
}

/// Build a `?pos` axis key like `w.x` from a system prefix and axis letter.
fn key(prefix: &str, axis: char) -> heapless::String<8> {
    let mut k = heapless::String::new();
    let _ = write!(k, "{}.{}", prefix, axis);
    k
}
/// Full name for the `?pos` `sys` field.
fn sys_name(cs: CoordSys) -> &'static str {
    match cs {
        CoordSys::Machine => "machine",
        CoordSys::Grinder => "grinder",
        CoordSys::Work => "work",
    }
}

/// Key prefix for `?pos` axis fields.
fn pos_prefix(cs: CoordSys) -> &'static str {
    match cs {
        CoordSys::Machine => "m",
        CoordSys::Grinder => "g",
        CoordSys::Work => "w",
    }
}
