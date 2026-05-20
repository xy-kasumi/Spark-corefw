// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Query-signal executor. It just converts snapshot to text. Must finish within << tick duration (1ms).

use core::fmt::Write;
use core::sync::atomic::Ordering;

use heapless::String;
use model::coords::{ActiveCoordSys, PosPhys};
use model::pstate::{Line, PsType};
use model::signal::QuerySignal;

use crate::commands::{CmdQueue, CMD_QUEUE_CAP, OUTSTANDING};
use crate::line_tx::LineTx;
use crate::motion::EdmState;

/// Snapshot of machine (enough to answer [`QuerySignal`])
#[derive(Clone, Copy)]
pub struct MachineStats {
    pub pos: PosPhys,
    pub edm: EdmState,
    pub active: ActiveCoordSys,
    pub offset: PosPhys,
    pub eff_duty: f32,
    pub open_rate: u8,
    pub short_rate: u8,
    pub temp: u8,
}

pub fn exec_query(sig: QuerySignal, stats: &MachineStats, cmd_queue: &CmdQueue, line_tx: &LineTx) {
    match sig {
        QuerySignal::Queue => {
            let num = cmd_queue.len() + OUTSTANDING.load(Ordering::Relaxed);
            let line = Line::new(PsType::Queue)
                .begin()
                .int("cap", CMD_QUEUE_CAP as i32)
                .int("num", num as i32)
                .end();
            let _ = line_tx.try_send(line);
        }
        QuerySignal::Pos => {
            let pos = stats.pos;
            let active = stats.active;
            let off = stats.offset;

            // Line 1: machine coordinates, always with the `m.` prefix.
            let line1 = Line::new(PsType::Pos)
                .begin()
                .str_val("sys", active.sys_name())
                .float("m.x", pos.x)
                .float("m.y", pos.y)
                .float("m.z", pos.z)
                .float("m.c", pos.c * 360.0);
            if active.is_machine() {
                let _ = line_tx.try_send(line1.end());
            } else {
                // Leave line 1 open; line 2 carries the active-system position.
                let _ = line_tx.try_send(line1);
                let cs = pos.with_offset_removed(off);
                let p = active.pos_prefix();
                let _ = line_tx.try_send(
                    Line::new(PsType::Pos)
                        .float(&key(p, 'x'), cs.x)
                        .float(&key(p, 'y'), cs.y)
                        .float(&key(p, 'z'), cs.z)
                        .float(&key(p, 'c'), cs.c * 360.0)
                        .end(),
                );
            }
        }
        QuerySignal::Edm => {
            // Motion and pulser fields come from the same tick snapshot (as in C).
            let edm = stats.edm;
            let eff_duty = stats.eff_duty;
            let r_open = stats.open_rate as f32 / 255.0;
            let r_short = stats.short_rate as f32 / 255.0;
            let temp = stats.temp;
            let _ = line_tx.try_send(Line::new(PsType::Edm).begin());
            if edm.has_edm_data {
                let _ = line_tx.try_send(
                    Line::new(PsType::Edm)
                        .float("eff_duty", eff_duty)
                        .float("open", r_open)
                        .float("short", r_short)
                        .int("temp", temp as i32),
                );
            }
            if edm.is_moving {
                let _ = line_tx.try_send(
                    Line::new(PsType::Edm)
                        .float("pb_f", edm.forward_buffer)
                        .float("pb_b", edm.backward_buffer)
                        .float("dist", edm.distance)
                        .float("dist_max", edm.distance_max),
                );
            }
            let _ = line_tx.try_send(Line::new(PsType::Edm).end());
        }
        QuerySignal::Unknown => {
            // Recognized signal byte, unknown verb: ignore to avoid clogging the stream.
        }
    }
}

/// Build a `?pos` axis key like `w.x` from a system prefix and axis letter.
fn key(prefix: &str, axis: char) -> String<8> {
    let mut k = String::new();
    let _ = write!(k, "{}.{}", prefix, axis);
    k
}
