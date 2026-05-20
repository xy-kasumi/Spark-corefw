//! Query-signal executor. The `QuerySignal` enum + byte-level parser live in
//! `model::signal`; this module is the firmware-side handler that runs inline
//! in the tick-loop RX phase, so it must finish quickly. The `!` cancel is
//! handled in `main.rs`, not here.

use core::fmt::Write;
use core::sync::atomic::Ordering;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::String;
use model::coordstate::CoordState;
use model::pstate::{Line, PsType};
use model::signal::QuerySignal;

use crate::board::Pulser;
use crate::commands::{CmdQueue, CMD_QUEUE_CAP, OUTSTANDING};
use crate::line_tx::LineTx;
use crate::motion::Motion;

pub async fn exec_query(
    sig: QuerySignal,
    motion: &Mutex<NoopRawMutex, Motion>,
    coord: &Mutex<NoopRawMutex, CoordState>,
    pulser: &Mutex<NoopRawMutex, Pulser>,
    cmd_queue: &CmdQueue,
    line_tx: &LineTx,
) {
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
            // Lock order motion -> coord (matches commands.rs).
            let pos = {
                let m = motion.lock().await;
                m.current_position()
            };
            let (active, off) = {
                let c = coord.lock().await;
                (c.active(), c.offset_of(c.active()))
            };

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
            // Sequential (non-nested) locks: motion first, then pulser. The C
            // handler reads the tick's snapshot; we read both modules live.
            let edm = motion.lock().await.edm_state();
            let (eff_duty, r_open, r_short, temp) = {
                let p = pulser.lock().await;
                (
                    p.eff_duty(),
                    p.open_rate() as f32 / 255.0,
                    p.short_rate() as f32 / 255.0,
                    p.temp(),
                )
            };
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
