//! Embassy-side queue plumbing for host commands. The typed `Command`,
//! `ParseError`, and `parse` all live in `model::command` (host-testable);
//! only the channel + outstanding-count atomic belongs here.

use core::sync::atomic::AtomicUsize;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
pub use model::command::{parse, Command};

pub const CMD_QUEUE_CAP: usize = 64;

pub type CmdQueue = Channel<NoopRawMutex, Command, CMD_QUEUE_CAP>;

/// Commands popped from [`CmdQueue`] but not yet finished executing — covers
/// both the currently-running command and the one held in cmd_loop's peek
/// buffer. Combined with `cmd_queue.len()` it gives the spec's "num" field
/// for `?queue` (items in queue including currently running).
pub static OUTSTANDING: AtomicUsize = AtomicUsize::new(0);
