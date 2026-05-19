//! Line-atomic TX layer over [`Serial`]. Producers build a [`Line`] (≤128 B)
//! and hand it off as a single message; a dedicated pump task drains the
//! channel and writes each line + LF through `serial.tx_push` in one burst,
//! so no two producers can interleave mid-line on the wire.
//!
//! Channel depth is sized in *lines*, not bytes — the failure mode under
//! backpressure is "how many lines can pile up before drops start."

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use model::pstate::Line;
use static_cell::StaticCell;

use crate::drivers::serial::Serial;

pub const TX_LINE_CAP: usize = 8;

pub struct LineTx {
    chan: Channel<NoopRawMutex, Line, TX_LINE_CAP>,
}

impl LineTx {
    /// Build the line-TX layer and spawn its pumper. One init() per program.
    pub fn init(spawner: &Spawner, serial: &'static Serial) -> &'static Self {
        static CELL: StaticCell<LineTx> = StaticCell::new();
        let me = CELL.init(LineTx {
            chan: Channel::new(),
        });
        spawner.must_spawn(pump(me, serial));
        me
    }

    /// Non-blocking enqueue. Returns the line back on full so the caller can
    /// observe drops. Use from anywhere that must not stall (signal handlers,
    /// tick loop body).
    pub fn try_send(&self, line: Line) -> Result<(), Line> {
        self.chan.try_send(line).map_err(|e| match e {
            embassy_sync::channel::TrySendError::Full(l) => l,
        })
    }

    /// Awaiting enqueue. Suspends the calling task on backpressure. Use from
    /// command-execution code where pacing the producer to UART speed is fine.
    /// Currently unused — slow command dumps (stat/stg) will rely on this.
    #[allow(dead_code)]
    pub async fn send(&self, line: Line) {
        self.chan.send(line).await;
    }
}

#[embassy_executor::task]
async fn pump(line_tx: &'static LineTx, serial: &'static Serial) {
    loop {
        let line = line_tx.chan.receive().await;
        serial.tx_push(line.as_bytes());
        serial.tx_push(b"\n");
    }
}
