// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Line-atomic TX queue. Producers stage whole [`pstate::Line`]s into a local
//! [`OutputBuf`], then [`LineTx::flush_drop`] (sync, drop-on-full) or
//! [`LineTx::flush`] (async, awaits room) moves them into the wire queue. The
//! tick loop drains the wire queue byte-by-byte into the serial TX ring,
//! never splitting a line across drains.

use embassy_sync::blocking_mutex::raw;
use embassy_sync::channel;
use model::pstate;

use crate::drivers::serial;

pub const TX_LINE_CAP: usize = 100;

/// Per-producer line staging buffer. `push` is infallible; over-capacity lines
/// silently drop. Each producer declares its own `N` at the call site, sized
/// to its worst-case burst.
pub struct OutputBuf<const N: usize> {
    lines: heapless::Vec<pstate::Line, N>,
}

impl<const N: usize> OutputBuf<N> {
    pub const fn new() -> Self {
        Self {
            lines: heapless::Vec::new(),
        }
    }

    pub fn push(&mut self, line: pstate::Line) {
        let _ = self.lines.push(line);
    }

    pub fn push_error(&mut self, args: core::fmt::Arguments<'_>) {
        self.push(pstate::error_msg(args));
    }

    /// Move the staged lines out for flushing.
    fn take_lines(&mut self) -> heapless::Vec<pstate::Line, N> {
        core::mem::take(&mut self.lines)
    }
}

impl<const N: usize> Default for OutputBuf<N> {
    fn default() -> Self {
        Self::new()
    }
}

pub struct LineTx {
    chan: channel::Channel<raw::NoopRawMutex, pstate::Line, TX_LINE_CAP>,
}

impl LineTx {
    /// Build the line-TX queue. One init() per program.
    pub fn init() -> &'static Self {
        static CELL: static_cell::StaticCell<LineTx> = static_cell::StaticCell::new();
        CELL.init(LineTx {
            chan: channel::Channel::new(),
        })
    }

    /// Sync. Moves each line in `buf` into the wire queue. Lines that don't fit
    /// (queue full) drop silently; subsequent lines from the same burst are
    /// still attempted. For producers that can't yield (tick body, init/fault).
    pub fn flush_drop<const N: usize>(&self, buf: &mut OutputBuf<N>) {
        for line in buf.take_lines() {
            let _ = self.chan.try_send(line);
        }
    }

    /// Async. Awaits queue room as needed. For producers that pace to UART speed
    /// (cmd_loop's per-command tail).
    pub async fn flush<const N: usize>(&self, buf: &mut OutputBuf<N>) {
        for line in buf.take_lines() {
            self.chan.send(line).await;
        }
    }

    /// True when no line is queued or mid-drain, so raw bytes (terminal echo)
    /// can be pushed to the wire without landing inside a protocol line.
    pub fn is_idle(&self, state: &DrainState) -> bool {
        state.line.is_none() && self.chan.is_empty()
    }

    /// Push as many queued lines as `serial`'s TX ring will accept this tick.
    /// `state` is the consumer's resume cursor (not queue state); `&mut`
    /// keeps the single-drainer invariant a compile-time fact.
    pub fn drain(&self, serial: &serial::Device, state: &mut DrainState) {
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
    line: Option<pstate::Line>,
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
