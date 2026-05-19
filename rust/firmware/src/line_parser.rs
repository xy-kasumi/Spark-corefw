//! Line-oriented input parser. Wraps `model::comm::Framer` with typed parsing
//! for both signal and command lines so the tick loop sees a single [`Parsed`]
//! enum per completed line.

use model::comm::{Frame, Framer};

use crate::commands::{self, Command, ParseError};
use crate::signals::Signal;

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

    /// Feed one byte. Returns `Some` on the LF that completes a non-empty line.
    pub fn feed(&mut self, b: u8) -> Option<Parsed<'_>> {
        let frame = self.framer.feed(b)?;
        Some(match frame {
            Frame::Signal(s) => Parsed::Signal(parse_signal(s)),
            Frame::Command(c) => match commands::parse(c) {
                Ok(cmd) => Parsed::Command(cmd),
                Err(e) => Parsed::CommandError(c, e),
            },
        })
    }
}

/// Classify a framed signal line. `bytes` includes the leading `!` or `?`.
fn parse_signal(bytes: &[u8]) -> Signal {
    match bytes {
        b"!" => Signal::Cancel,
        b"?queue" => Signal::QueryQueue,
        b"?pos" => Signal::QueryPos,
        _ => Signal::Unknown,
    }
}
