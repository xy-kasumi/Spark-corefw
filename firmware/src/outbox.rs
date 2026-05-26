// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

use embassy_sync::blocking_mutex::raw;
use embassy_sync::pipe;
use model::pstate;

use crate::drivers::serial;

const DRAIN_SCRATCH: usize = 64;

/// Buffer to hold single logical batch of output without I/O.
pub struct OutputBuf<const N: usize> {
    bytes: heapless::Vec<u8, N>,
    overflowed: bool,
}

impl<const N: usize> OutputBuf<N> {
    pub const fn new() -> Self {
        Self {
            bytes: heapless::Vec::new(),
            overflowed: false,
        }
    }

    pub fn push(&mut self, line: pstate::Line) {
        if line.overflowed() {
            self.overflowed = true;
        }
        let payload = line.as_bytes();
        // +1 for trailing LF.
        if self.bytes.len() + payload.len() + 1 > N {
            self.overflowed = true;
            return;
        }
        let _ = self.bytes.extend_from_slice(payload);
        let _ = self.bytes.push(b'\n');
    }

    pub fn push_error(&mut self, args: core::fmt::Arguments<'_>) {
        self.push(pstate::error_msg(args));
    }

    #[allow(dead_code)] // Exposed for producers to surface drops; no in-tree caller yet.
    pub fn overflowed(&self) -> bool {
        self.overflowed
    }

    fn take(&mut self) -> heapless::Vec<u8, N> {
        self.overflowed = false;
        core::mem::take(&mut self.bytes)
    }
}

impl<const N: usize> Default for OutputBuf<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Temp byte buffer between logic and serial device.
pub struct Outbox<const N: usize> {
    pipe: pipe::Pipe<raw::NoopRawMutex, N>,
}

impl<const N: usize> Outbox<N> {
    pub const fn new() -> Self {
        Self {
            pipe: pipe::Pipe::new(),
        }
    }

    /// Async. Move `buf`'s bytes into the wire ring, awaiting room as needed.
    pub async fn flush<const M: usize>(&self, buf: &mut OutputBuf<M>) {
        let bytes = buf.take();
        let mut sent = 0;
        while sent < bytes.len() {
            sent += self.pipe.write(&bytes[sent..]).await;
        }
    }

    /// True when no bytes are queued or mid-drain, so raw bytes (terminal echo)
    /// can be pushed to the wire without landing inside a protocol line.
    pub fn is_idle(&self, state: &DrainState) -> bool {
        state.offset >= state.len && self.pipe.is_empty()
    }

    /// Push as many queued bytes as `serial`'s TX ring will accept this tick.
    /// `state` is the consumer's scratch (not ring state); `&mut` keeps the
    /// single-drainer invariant a compile-time fact.
    pub fn drain(&self, serial: &serial::Device, state: &mut DrainState) {
        loop {
            if state.offset >= state.len {
                state.offset = 0;
                state.len = match self.pipe.try_read(&mut state.scratch) {
                    Ok(n) => n,
                    Err(_) => return,
                };
                if state.len == 0 {
                    return;
                }
            }
            let pushed = serial.tx_push(&state.scratch[state.offset..state.len]);
            if pushed == 0 {
                return;
            }
            state.offset += pushed;
        }
    }
}

/// Per-loop state for [`Outbox::drain`]: a scratch holding bytes pulled from
/// the wire ring that the serial TX ring hasn't yet accepted.
pub struct DrainState {
    scratch: [u8; DRAIN_SCRATCH],
    len: usize,
    offset: usize,
}

impl DrainState {
    pub const fn new() -> Self {
        Self {
            scratch: [0; DRAIN_SCRATCH],
            len: 0,
            offset: 0,
        }
    }
}
