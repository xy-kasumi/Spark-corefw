//! Line framing + typed parsing per protocol.md transport layer.
//!
//! [`Framer`] strips CR, terminates on LF, and silently discards overflowing
//! or empty lines, classifying the result as signal (`!` / `?xxx`) or command
//! bytes. [`Parser`] sits on top of `Framer` and runs the per-kind parsers
//! (`signal::parse`, `command::parse`) so callers see a single [`Parsed`]
//! enum per completed line.

use heapless::Vec;

use crate::command::{self, Command, ParseError};
use crate::signal::{self, Signal};

/// Spec caps payload at 100 VCHAR; round up to a power of two.
pub const LINE_CAP: usize = 128;

pub enum Frame<'a> {
    /// First-byte signal: `!` (cancel) or `?xxx` (query). Includes the leading byte.
    Signal(&'a [u8]),
    Command(&'a [u8]),
}

enum State {
    Building,
    /// Line exceeded LINE_CAP; drop bytes until the terminating LF.
    Poisoned,
    /// Last feed() returned a Frame that borrows self.buf; clear on next call.
    Holding,
}

pub struct Framer {
    buf: Vec<u8, LINE_CAP>,
    state: State,
}

impl Framer {
    pub const fn new() -> Self {
        Self {
            buf: Vec::new(),
            state: State::Building,
        }
    }

    /// Feed one byte. Returns Some(Frame) on the LF that completes a
    /// conformant non-empty line. CR is silently dropped per spec.
    pub fn feed(&mut self, b: u8) -> Option<Frame<'_>> {
        if matches!(self.state, State::Holding) {
            self.buf.clear();
            self.state = State::Building;
        }
        match b {
            b'\r' => None,
            // Backspace / DEL edit the in-progress line, the same human-from-a-
            // serial-terminal input normalization as CR-stripping above. Only
            // meaningful while building; in other states there is no live line.
            0x08 | 0x7F => {
                if matches!(self.state, State::Building) {
                    self.buf.pop();
                }
                None
            }
            b'\n' => match self.state {
                State::Poisoned => {
                    self.buf.clear();
                    self.state = State::Building;
                    None
                }
                State::Building => {
                    if self.buf.is_empty() {
                        None
                    } else {
                        self.state = State::Holding;
                        Some(classify(&self.buf))
                    }
                }
                State::Holding => unreachable!(),
            },
            _ => {
                if matches!(self.state, State::Building) && self.buf.push(b).is_err() {
                    self.state = State::Poisoned;
                }
                None
            }
        }
    }

    /// Bytes buffered in the line currently being assembled. Reports 0 unless
    /// actively building (a held or poisoned line is not live). Used only by
    /// interactive echo to gate backspace erase; the protocol path ignores it.
    pub fn line_len(&self) -> usize {
        match self.state {
            State::Building => self.buf.len(),
            _ => 0,
        }
    }
}

fn classify(line: &[u8]) -> Frame<'_> {
    match line.first() {
        Some(b'!') | Some(b'?') => Frame::Signal(line),
        _ => Frame::Command(line),
    }
}

pub enum Parsed<'a> {
    Signal(Signal),
    Command(Command),
    /// Command line failed to parse; carries source bytes + error for diagnostics.
    CommandError(&'a [u8], ParseError),
}

pub struct Parser {
    framer: Framer,
}

impl Parser {
    pub const fn new() -> Self {
        Self {
            framer: Framer::new(),
        }
    }

    /// Length of the in-progress line; see [`Framer::line_len`].
    pub fn line_len(&self) -> usize {
        self.framer.line_len()
    }

    /// Feed one byte. Returns `Some` on the LF that completes a non-empty line.
    pub fn feed(&mut self, b: u8) -> Option<Parsed<'_>> {
        let frame = self.framer.feed(b)?;
        Some(match frame {
            Frame::Signal(s) => Parsed::Signal(signal::parse(s)),
            Frame::Command(c) => match command::parse(c) {
                Ok(cmd) => Parsed::Command(cmd),
                Err(e) => Parsed::CommandError(c, e),
            },
        })
    }
}
