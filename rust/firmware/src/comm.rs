//! Sync line framer + parse + dispatch + reply, fed one byte per call from
//! the orchestrator's per-tick RX drain.

use heapless::Vec;
use model::gcode::{self, Command, ParseError};

use crate::dispatch;
use crate::motion::Motion;
use crate::serial::Serial;

pub const LINE_CAP: usize = 128;
pub type LineBuf = Vec<u8, LINE_CAP>;

/// Feed one received byte. On line termination, parse + dispatch + emit reply.
/// `!` cancels motion immediately and emits an ack.
pub fn handle_byte(b: u8, line: &mut LineBuf, motion: &mut Motion, serial: &Serial) {
    match b {
        b'!' => {
            motion.cancel();
            line.clear();
            serial.tx_push(b"cancelled\r\n");
        }
        b'\n' | b'\r' => {
            if !line.is_empty() {
                match handle_line(line, motion) {
                    Ok(()) => serial.tx_push(b"ok\r\n"),
                    Err(e) => {
                        serial.tx_push(b"err ");
                        serial.tx_push(err_name(e));
                        serial.tx_push(b"\r\n");
                    }
                }
                line.clear();
            }
        }
        _ => {
            let _ = line.push(b); // Phase 4: surface overflow as protocol error
        }
    }
}

fn handle_line(line: &[u8], motion: &mut Motion) -> Result<(), ParseError> {
    let cmd: Command = gcode::parse(line)?;
    dispatch::exec(cmd, motion);
    Ok(())
}

fn err_name(e: ParseError) -> &'static [u8] {
    match e {
        ParseError::Empty => b"empty",
        ParseError::UnknownCommand => b"unknown",
        ParseError::BadAxis => b"bad-axis",
        ParseError::BadNumber => b"bad-number",
        ParseError::ExpectedSeparator => b"missing-sep",
        ParseError::TrailingGarbage => b"trailing",
    }
}
