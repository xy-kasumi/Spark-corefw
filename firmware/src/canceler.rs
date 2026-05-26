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
//!
//! Fault is a sticky one-way variant of cancel. Once entered, never clears;
//! [`active`](Canceler::active) stays true forever so the RX gate silences all
//! writes for the rest of this power cycle.

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
    fault: atomic::AtomicBool,
}

impl Canceler {
    pub const fn new() -> Self {
        Self {
            gen: atomic::AtomicU32::new(0),
            ticks_left: atomic::AtomicU16::new(0),
            fault: atomic::AtomicBool::new(false),
        }
    }

    /// Bump the generation and (re)arm the drain window.
    pub fn cancel(&self) {
        self.gen.fetch_add(1, atomic::Ordering::Relaxed);
        self.ticks_left
            .store(CANCEL_TICKS, atomic::Ordering::Relaxed);
    }

    /// Latch the fault flag. Returns `true` on the entering transition, `false`
    /// if already in fault. Also bumps the generation and arms the drain window
    /// so in-flight watchers fire one more time.
    pub fn enter_fault(&self) -> bool {
        if self
            .fault
            .compare_exchange(
                false,
                true,
                atomic::Ordering::Relaxed,
                atomic::Ordering::Relaxed,
            )
            .is_err()
        {
            return false;
        }
        self.gen.fetch_add(1, atomic::Ordering::Relaxed);
        self.ticks_left
            .store(CANCEL_TICKS, atomic::Ordering::Relaxed);
        true
    }

    /// True once the fault latch is set; stays true until power-cycle.
    pub fn faulted(&self) -> bool {
        self.fault.load(atomic::Ordering::Relaxed)
    }

    /// Age the drain window by one tick. Call once per orchestrator tick.
    pub fn tick(&self) {
        let left = self.ticks_left.load(atomic::Ordering::Relaxed);
        if left > 0 {
            self.ticks_left.store(left - 1, atomic::Ordering::Relaxed);
        }
    }

    /// True while the drain window is open or the fault latch is set:
    /// incoming write commands should be discarded.
    pub fn active(&self) -> bool {
        self.faulted() || self.ticks_left.load(atomic::Ordering::Relaxed) > 0
    }

    /// Snapshot the cancel generation. Pair with [`Watcher::cancelled`] to
    /// detect a cancel that landed while a command was running.
    pub fn watch(&self) -> Watcher<'_> {
        Watcher {
            canceler: self,
            gen: self.gen.load(atomic::Ordering::Relaxed),
        }
    }
}

/// A snapshot of the cancel generation taken at [`Canceler::watch`].
#[derive(Clone, Copy)]
pub struct Watcher<'a> {
    canceler: &'a Canceler,
    gen: u32,
}

impl Watcher<'_> {
    /// True if a cancel has fired since this watch was taken.
    pub fn cancelled(&self) -> bool {
        self.canceler.gen.load(atomic::Ordering::Relaxed) != self.gen
    }
}
