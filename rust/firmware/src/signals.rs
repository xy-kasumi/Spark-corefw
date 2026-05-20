//! Host signal executor. The `Signal` enum + byte-level parser live in
//! `model::signal`; this module is the firmware-side handler that runs inline
//! in the tick-loop RX phase, so it must finish quickly.

use core::fmt::Write;
use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use heapless::String;
use model::coordstate::CoordState;
use model::pstate::{Line, PsType};
use model::signal::Signal;

use crate::board::Pulser;
use crate::commands::{CmdQueue, CMD_QUEUE_CAP, OUTSTANDING};
use crate::line_tx::LineTx;
use crate::motion::Motion;

/// Bumped on every cancel. The executor snapshots it around long moves (homing)
/// to tell a completed move from a cancelled one without a shared latch.
pub static CANCEL_GEN: AtomicU32 = AtomicU32::new(0);

pub async fn exec(
    sig: Signal,
    motion: &Mutex<NoopRawMutex, Motion>,
    coord: &Mutex<NoopRawMutex, CoordState>,
    pulser: &Mutex<NoopRawMutex, Pulser>,
    cmd_queue: &CmdQueue,
    line_tx: &LineTx,
) {
    match sig {
        Signal::Cancel => {
            CANCEL_GEN.fetch_add(1, Ordering::Relaxed);
            // Lock order motion -> coord (matches commands.rs).
            {
                let mut m = motion.lock().await;
                m.cancel();
            }
            coord.lock().await.cancel();
            pulser.lock().await.deenergize().await;
            while cmd_queue.try_receive().is_ok() {}
        }
        Signal::QueryQueue => {
            let num = cmd_queue.len() + OUTSTANDING.load(Ordering::Relaxed);
            let line = Line::new(PsType::Queue)
                .begin()
                .int("cap", CMD_QUEUE_CAP as i32)
                .int("num", num as i32)
                .end();
            let _ = line_tx.try_send(line);
        }
        Signal::QueryPos => {
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
        Signal::Unknown => {
            // FIXME: ?edm and other queries not implemented.
        }
    }
}

/// Build a `?pos` axis key like `w.x` from a system prefix and axis letter.
fn key(prefix: &str, axis: char) -> String<8> {
    let mut k = String::new();
    let _ = write!(k, "{}.{}", prefix, axis);
    k
}
