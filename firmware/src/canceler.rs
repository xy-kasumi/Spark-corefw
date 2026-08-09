// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Manages machine-wide safety status.
//! * cancel: window where all commands should be ignored & hardware set to be safe state
//! * fault: latching flag that forbids "write" command until power cycle
//!
//! Fault is weaker than cancel, to allow "read" command for debugging.

use core::sync::atomic;

/// Drain window length, long enough to discard all in-transit commands & device traisients.
const CANCEL_TICKS: u16 = 500;

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

    /// Set latching fault flag.
    /// Returns `true` on the entering. `false` if already in fault-mode.
    /// Upon first entry to fault, one-time cancelation window also happens.
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

    /// True when in cancelation window.
    pub fn canceled(&self) -> bool {
        self.ticks_left.load(atomic::Ordering::Relaxed) > 0
    }

    /// Age the drain window by one tick. Call once per orchestrator tick.
    pub fn tick(&self) {
        let left = self.ticks_left.load(atomic::Ordering::Relaxed);
        if left > 0 {
            self.ticks_left.store(left - 1, atomic::Ordering::Relaxed);
        }
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

/// Cancelation detector that doesn't require constant polling. Taken at [`Canceler::watch`].
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
