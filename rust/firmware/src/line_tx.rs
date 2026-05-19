//! Line-atomic TX queue. Producers hand off a whole [`Line`] (≤128 B) as one
//! message; the tick loop drains each line into the serial TX ring without
//! splitting across drains, so producers never interleave mid-line on the wire.
//!
//! Capacity is in *lines*, not bytes — backpressure drops whole lines.

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use model::pstate::Line;
use static_cell::StaticCell;

pub const TX_LINE_CAP: usize = 100;

pub struct LineTx {
    chan: Channel<NoopRawMutex, Line, TX_LINE_CAP>,
}

impl LineTx {
    /// Build the line-TX queue. One init() per program.
    pub fn init() -> &'static Self {
        static CELL: StaticCell<LineTx> = StaticCell::new();
        CELL.init(LineTx {
            chan: Channel::new(),
        })
    }

    /// Non-blocking enqueue. On full, returns the line back so the caller can observe drops.
    /// Use from anywhere that must not stall (signal handlers, tick loop body).
    pub fn try_send(&self, line: Line) -> Result<(), Line> {
        self.chan.try_send(line).map_err(|e| match e {
            embassy_sync::channel::TrySendError::Full(l) => l,
        })
    }

    /// Awaiting enqueue. Suspends on backpressure — use where pacing the producer
    /// to UART speed is fine (e.g. command execution).
    pub async fn send(&self, line: Line) {
        self.chan.send(line).await;
    }

    /// Non-blocking dequeue. The tick-loop drainer is the only caller.
    pub fn try_recv(&self) -> Option<Line> {
        self.chan.try_receive().ok()
    }
}
