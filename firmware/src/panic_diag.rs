// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Diagnostic panic handler. Records the panic's source location into a fixed
//! RAM struct (read back over SWD by `diag.sh`), then spins forever.
//!
//! Behaviourally identical to `panic-halt` from the hardware's point of view:
//! it touches no peripheral, so every GPIO holds its last state and nothing is
//! reset or de-energized. The only difference is the breadcrumb it leaves in RAM.
//!
//! `file_ptr`/`file_len` point at the `&'static str` filename living in flash
//! (`.rodata`), so the reader can pull the string straight off the target.

use core::panic::PanicInfo;
use core::sync::atomic::{compiler_fence, AtomicU32, Ordering};

/// Written last, so a reader that sees it knows the other fields are valid.
/// Spells "PAN1".
pub const MAGIC: u32 = 0x5041_4e31;

#[repr(C)]
pub struct PanicReport {
    pub magic: AtomicU32,
    pub line: AtomicU32,
    pub file_ptr: AtomicU32,
    pub file_len: AtomicU32,
}

#[no_mangle]
pub static PANIC_REPORT: PanicReport = PanicReport {
    magic: AtomicU32::new(0),
    line: AtomicU32::new(0),
    file_ptr: AtomicU32::new(0),
    file_len: AtomicU32::new(0),
};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    if let Some(loc) = info.location() {
        let file = loc.file();
        PANIC_REPORT
            .file_ptr
            .store(file.as_ptr() as u32, Ordering::Relaxed);
        PANIC_REPORT
            .file_len
            .store(file.len() as u32, Ordering::Relaxed);
        PANIC_REPORT.line.store(loc.line(), Ordering::Relaxed);
    }
    PANIC_REPORT.magic.store(MAGIC, Ordering::SeqCst);
    loop {
        compiler_fence(Ordering::SeqCst);
    }
}
