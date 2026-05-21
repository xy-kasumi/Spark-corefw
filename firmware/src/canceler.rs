// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Cancel state shared by the orchestrator's RX phase and the command executor.
//!
//! A cancel (`!`) does two things: it bumps a generation counter and arms a
//! drain window. The window lets the RX phase blackhole incoming commands for
//! [`CANCEL_TICKS`] ticks, so a single `!` empties the queue instead of racing
//! bytes still in flight from the host. The generation lets the executor tell a
//! command that finished normally from one a cancel landed on, since its
//! lookahead is already pulled out of the queue and the drain can't reach it.

use core::sync::atomic;

/// Drain window length, in orchestrator ticks (1 kHz). Long enough to outlast
/// host-side and on-wire buffering that a single `!` can't otherwise reach; a
/// soft e-stop has no reason to be twitchy.
const CANCEL_TICKS: u16 = 500;

/// Shared cancel state. The RX phase arms and ages it; the executor watches it.
/// Single-writer for the window (only the RX phase calls [`cancel`](Self::cancel)
/// and [`tick`](Self::tick)), so the load/modify/store in `tick` needs no RMW.
pub struct Canceler {
    gen: atomic::AtomicU32,
    ticks_left: atomic::AtomicU16,
}

impl Canceler {
    pub const fn new() -> Self {
        Self {
            gen: atomic::AtomicU32::new(0),
            ticks_left: atomic::AtomicU16::new(0),
        }
    }

    /// Bump the generation and (re)arm the drain window.
    pub fn cancel(&self) {
        self.gen.fetch_add(1, atomic::Ordering::Relaxed);
        self.ticks_left
            .store(CANCEL_TICKS, atomic::Ordering::Relaxed);
    }

    /// Age the drain window by one tick. Call once per orchestrator tick.
    pub fn tick(&self) {
        let left = self.ticks_left.load(atomic::Ordering::Relaxed);
        if left > 0 {
            self.ticks_left.store(left - 1, atomic::Ordering::Relaxed);
        }
    }

    /// True while the drain window is open: incoming commands should be discarded.
    pub fn active(&self) -> bool {
        self.ticks_left.load(atomic::Ordering::Relaxed) > 0
    }

    /// Snapshot the cancel generation. Pair with [`CancelWatch::cancelled`] to
    /// detect a cancel that landed while a command was running.
    pub fn watch(&'static self) -> Watcher {
        Watcher {
            canceler: self,
            gen: self.gen.load(atomic::Ordering::Relaxed),
        }
    }
}

/// A snapshot of the cancel generation taken at [`Canceler::watch`].
#[derive(Clone, Copy)]
pub struct Watcher {
    canceler: &'static Canceler,
    gen: u32,
}

impl Watcher {
    /// True if a cancel has fired since this watch was taken.
    pub fn cancelled(&self) -> bool {
        self.canceler.gen.load(atomic::Ordering::Relaxed) != self.gen
    }
}

pub static CANCELER: Canceler = Canceler::new();
