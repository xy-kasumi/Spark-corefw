//! Line-atomic TX queue. Producers build a [`Line`] (≤128 B) and hand it off
//! as a single message; the orchestrator tick loop drains the queue into the
//! serial TX ring, never splitting a line across drain ticks, so no two
//! producers can interleave mid-line on the wire.
//!
//! Channel depth is sized in *lines*, not bytes — the failure mode under
//! backpressure is "how many lines can pile up before drops start."

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
    pub async fn send(&self, line: Line) {
        self.chan.send(line).await;
    }

    /// Non-blocking dequeue. The tick-loop drainer is the only caller.
    pub fn try_recv(&self) -> Option<Line> {
        self.chan.try_receive().ok()
    }
}
