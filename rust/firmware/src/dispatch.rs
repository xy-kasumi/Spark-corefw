//! Signal handling: ! / ? immediate-action signals from the host. Runs inline
//! from `tick_loop`'s rx-parse phase, so it must finish quickly.

use core::sync::atomic::Ordering;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use model::pstate::{Line, PsType};

use crate::command::{CmdQueue, CMD_QUEUE_CAP, OUTSTANDING};
use crate::line_tx::LineTx;
use crate::motion::Motion;

pub async fn signal(
    bytes: &[u8],
    motion: &'static Mutex<NoopRawMutex, Motion>,
    cmd_queue: &'static CmdQueue,
    line_tx: &'static LineTx,
) {
    match bytes {
        b"!" => {
            {
                let mut m = motion.lock().await;
                m.cancel();
            }
            while cmd_queue.try_receive().is_ok() {}
        }
        b"?queue" => {
            let num = cmd_queue.len() + OUTSTANDING.load(Ordering::Relaxed);
            let line = Line::new(PsType::Queue)
                .begin()
                .int("cap", CMD_QUEUE_CAP as i32)
                .int("num", num as i32)
                .end();
            let _ = line_tx.try_send(line);
        }
        _ => {
            // ?pos / ?edm land in a later phase.
        }
    }
}
