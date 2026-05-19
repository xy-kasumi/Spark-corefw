//! Line-atomic TX queue. Producers hand off a whole [`Line`] (≤128 B) as one
//! message; the tick loop drains each line into the serial TX ring without
//! splitting across drains, so producers never interleave mid-line on the wire.
//!
//! Capacity is in *lines*, not bytes — backpressure drops whole lines.

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use model::pstate::Line;
use static_cell::StaticCell;

use crate::drivers::serial::Serial;

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

    /// Push as many queued lines as `serial`'s TX ring will accept this tick.
    /// `state` is the consumer's resume cursor (not queue state); `&mut`
    /// keeps the single-drainer invariant a compile-time fact.
    pub fn drain(&self, serial: &Serial, state: &mut DrainState) {
        loop {
            if state.line.is_none() {
                match self.chan.try_receive().ok() {
                    Some(l) => {
                        state.line = Some(l);
                        state.offset = 0;
                    }
                    None => return,
                }
            }
            let bytes = state.line.as_ref().unwrap().as_bytes();
            if state.offset < bytes.len() {
                let n = serial.tx_push(&bytes[state.offset..]);
                state.offset += n;
                if state.offset < bytes.len() {
                    return;
                }
            }
            if serial.tx_push(b"\n") == 0 {
                return;
            }
            state.line = None;
        }
    }
}

/// Per-loop state for [`LineTx::drain`]: the line currently being shipped and
/// how many of its bytes have already been pushed to the serial TX ring. A
/// trailing LF is still owed once the payload is fully pushed; only then do
/// we pull the next line.
pub struct DrainState {
    line: Option<Line>,
    offset: usize,
}

impl DrainState {
    pub const fn new() -> Self {
        Self {
            line: None,
            offset: 0,
        }
    }
}
