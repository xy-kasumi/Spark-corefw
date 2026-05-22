// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Query-signal executor. It just converts snapshot to text. Must finish within << tick duration (1ms).

use core::fmt::Write;
use core::sync::atomic;

use model::coords;
use model::coords::CoordSys;
use model::pstate;
use model::signal;

use crate::commands;
use crate::line_tx;
use crate::motion;

/// Snapshot of machine (enough to answer [`QuerySignal`])
#[derive(Clone, Copy)]
pub struct MachineStats {
    pub pos: coords::PosPhys,
    pub edm: motion::EdmState,
    pub active: coords::CoordSys,
    pub offset: coords::PosPhys,
    pub eff_duty: f32,
    pub open_rate: u8,
    pub short_rate: u8,
    pub temp: u8,
}

pub fn exec_query(
    sig: signal::QuerySignal,
    stats: &MachineStats,
    cmd_queue: &commands::CmdQueue,
    line_tx: &line_tx::LineTx,
) {
    match sig {
        signal::QuerySignal::Queue => {
            let num = cmd_queue.len() + commands::OUTSTANDING.load(atomic::Ordering::Relaxed);
            let line = pstate::Line::new(pstate::PsType::Queue)
                .begin()
                .int("cap", commands::CMD_QUEUE_CAP as i32)
                .int("num", num as i32)
                .end();
            let _ = line_tx.try_send(line);
        }
        signal::QuerySignal::Pos => {
            let pos = stats.pos;
            let active = stats.active;
            let off = stats.offset;

            // Line 1: machine coordinates, always with the `m.` prefix.
            let line1 = pstate::Line::new(pstate::PsType::Pos)
                .begin()
                .str_val("sys", sys_name(active))
                .float("m.x", pos.x)
                .float("m.y", pos.y)
                .float("m.z", pos.z)
                .float("m.c", pos.c * 360.0);
            if active == CoordSys::Machine {
                let _ = line_tx.try_send(line1.end());
            } else {
                // Leave line 1 open; line 2 carries the active-system position.
                let _ = line_tx.try_send(line1);
                let cs = pos.with_offset_removed(off);
                let p = pos_prefix(active);
                let _ = line_tx.try_send(
                    pstate::Line::new(pstate::PsType::Pos)
                        .float(&key(p, 'x'), cs.x)
                        .float(&key(p, 'y'), cs.y)
                        .float(&key(p, 'z'), cs.z)
                        .float(&key(p, 'c'), cs.c * 360.0)
                        .end(),
                );
            }
        }
        signal::QuerySignal::Edm => {
            // Motion and pulser fields come from the same tick snapshot.
            let edm = stats.edm;
            let eff_duty = stats.eff_duty;
            let r_open = stats.open_rate as f32 / 255.0;
            let r_short = stats.short_rate as f32 / 255.0;
            let temp = stats.temp;
            let _ = line_tx.try_send(pstate::Line::new(pstate::PsType::Edm).begin());
            if edm.has_edm_data {
                let _ = line_tx.try_send(
                    pstate::Line::new(pstate::PsType::Edm)
                        .float("eff_duty", eff_duty)
                        .float("open", r_open)
                        .float("short", r_short)
                        .int("temp", temp as i32),
                );
            }
            if edm.is_moving {
                let _ = line_tx.try_send(
                    pstate::Line::new(pstate::PsType::Edm)
                        .float("pb_f", edm.forward_buffer)
                        .float("pb_b", edm.backward_buffer)
                        .float("dist", edm.distance)
                        .float("dist_max", edm.distance_max),
                );
            }
            let _ = line_tx.try_send(pstate::Line::new(pstate::PsType::Edm).end());
        }
        signal::QuerySignal::Unknown => {
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

/// Key prefix for `?pos` axis fields. Note toolsupply is `t`, not `ts`.
fn pos_prefix(cs: CoordSys) -> &'static str {
    match cs {
        CoordSys::Machine => "m",
        CoordSys::Grinder => "g",
        CoordSys::Work => "w",
    }
}
