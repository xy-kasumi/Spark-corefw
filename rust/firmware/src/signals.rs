//! Host signal executor. The `Signal` enum + byte-level parser live in
//! `model::signal`; this module is the firmware-side handler that runs inline
//! in the tick-loop RX phase, so it must finish quickly.

use core::sync::atomic::Ordering;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use model::pstate::{Line, PsType};
use model::signal::Signal;

use crate::commands::{CmdQueue, CMD_QUEUE_CAP, OUTSTANDING};
use crate::line_tx::LineTx;
use crate::motion::Motion;

pub async fn exec(
    sig: Signal,
    motion: &Mutex<NoopRawMutex, Motion>,
    cmd_queue: &CmdQueue,
    line_tx: &LineTx,
) {
    match sig {
        Signal::Cancel => {
            {
                let mut m = motion.lock().await;
                m.cancel();
            }
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
            // FIXME: coordinate-system selection (G53/G54/...) unimplemented; always machine.
            let pos = {
                let m = motion.lock().await;
                m.current_position()
            };
            let line = Line::new(PsType::Pos)
                .begin()
                .str_val("sys", "machine")
                .float("m.x", pos.x)
                .float("m.y", pos.y)
                .float("m.z", pos.z)
                .float("m.c", pos.c * 360.0)
                .end();
            let _ = line_tx.try_send(line);
        }
        Signal::Unknown => {
            // FIXME: ?edm and other queries not implemented.
        }
    }
}
