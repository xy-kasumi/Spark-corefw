//! Queue-ready command representation. Bytes are parsed in tick_loop's rx
//! phase; only validated values reach the command queue, so cmd_loop never
//! re-parses or has to surface syntax errors mid-execution.

use core::sync::atomic::AtomicUsize;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use model::gcode::{self, ParseError};

pub const CMD_QUEUE_CAP: usize = 64;

pub type CmdQueue = Channel<NoopRawMutex, Command, CMD_QUEUE_CAP>;

/// Commands popped from [`CmdQueue`] but not yet finished executing — covers
/// both the currently-running command and the one held in cmd_loop's peek
/// buffer. Combined with `cmd_queue.len()` it gives the spec's "num" field
/// for `?queue` (items in queue including currently running).
pub static OUTSTANDING: AtomicUsize = AtomicUsize::new(0);

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    Gcode(gcode::Command),
    // Set / Get / Stat land as their own variants when the parsers exist.
}

pub fn parse(bytes: &[u8]) -> Result<Command, ParseError> {
    gcode::parse(bytes).map(Command::Gcode)
}
