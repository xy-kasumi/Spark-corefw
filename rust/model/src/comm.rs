//! Line framing per protocol.md transport layer: strip CR, terminate on LF,
//! silently discard overflowing or empty lines. Classifies the resulting line
//! as signal (first byte `!` or `?`) or command. Payload semantics live above.

use heapless::Vec;

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
        Self { buf: Vec::new(), state: State::Building }
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
}

fn classify(line: &[u8]) -> Frame<'_> {
    match line.first() {
        Some(b'!') | Some(b'?') => Frame::Signal(line),
        _ => Frame::Command(line),
    }
}
