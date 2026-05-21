// SPDX-FileCopyrightText: 夕月霞
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

use core::panic;
use core::sync::atomic;

/// Written last, so a reader that sees it knows the other fields are valid.
/// Spells "PAN1".
pub const MAGIC: u32 = 0x5041_4e31;

#[repr(C)]
pub struct Report {
    pub magic: atomic::AtomicU32,
    pub line: atomic::AtomicU32,
    pub file_ptr: atomic::AtomicU32,
    pub file_len: atomic::AtomicU32,
}

#[no_mangle]
pub static PANIC_REPORT: Report = Report {
    magic: atomic::AtomicU32::new(0),
    line: atomic::AtomicU32::new(0),
    file_ptr: atomic::AtomicU32::new(0),
    file_len: atomic::AtomicU32::new(0),
};

#[panic_handler]
fn panic(info: &panic::PanicInfo) -> ! {
    if let Some(loc) = info.location() {
        let file = loc.file();
        PANIC_REPORT
            .file_ptr
            .store(file.as_ptr() as u32, atomic::Ordering::Relaxed);
        PANIC_REPORT
            .file_len
            .store(file.len() as u32, atomic::Ordering::Relaxed);
        PANIC_REPORT
            .line
            .store(loc.line(), atomic::Ordering::Relaxed);
    }
    PANIC_REPORT.magic.store(MAGIC, atomic::Ordering::SeqCst);
    loop {
        atomic::compiler_fence(atomic::Ordering::SeqCst);
    }
}
