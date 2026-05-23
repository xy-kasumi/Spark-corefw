// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Byte stream -> line splitter (see `docs/serial.md`).

pub const LINE_CAP: usize = 128;

enum State {
    Building,
    /// Line exceeded LINE_CAP; drop bytes until the terminating LF.
    Poisoned,
    /// Last feed() returned a slice that borrows self.buf; clear on next call.
    Holding,
}

pub struct Framer {
    buf: heapless::Vec<u8, LINE_CAP>,
    state: State,
}

impl Framer {
    pub const fn new() -> Self {
        Self {
            buf: heapless::Vec::new(),
            state: State::Building,
        }
    }

    /// Feed one byte. Returns Some(bytes) on the LF that completes a
    /// conformant non-empty line. CR is silently dropped per spec.
    pub fn feed(&mut self, b: u8) -> Option<&[u8]> {
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
                        Some(&self.buf)
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
