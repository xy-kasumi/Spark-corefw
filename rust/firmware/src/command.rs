//! Embassy-side queue plumbing for host commands.
//! `Command`, `ParseError`, `parse` live in `model::command` (host-testable).

use core::sync::atomic::AtomicUsize;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
pub use model::command::{parse, Command};

pub const CMD_QUEUE_CAP: usize = 64;

pub type CmdQueue = Channel<NoopRawMutex, Command, CMD_QUEUE_CAP>;

/// Commands popped from [`CmdQueue`] but not yet finished — covers the running
/// command and the one in cmd_loop's peek buffer.
/// `cmd_queue.len() + OUTSTANDING` gives `?queue`'s "num" field.
pub static OUTSTANDING: AtomicUsize = AtomicUsize::new(0);
