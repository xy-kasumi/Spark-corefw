// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Serial-terminal niceties for human users, deliberately quarantined from the
//! protocol path so the "nice text shell" never distorts the normal-path APIs.
//!
//! Line *correctness* lives in `model::comm::Framer` (it drops backspaced bytes
//! as ordinary input normalization, the same bucket as CR-stripping). Everything
//! here is pure *display* feedback that a host program neither needs nor sees: a
//! machine sender emits no backspaces, and the newline echo collapses to an
//! empty line the framer discards.

use crate::drivers::serial;

/// Emit terminal echo for one inbound byte, mirroring the C transport:
/// - LF -> `\r\n`, so the user's Enter shows as a proper line break.
/// - BS/DEL over a non-empty in-progress line -> ` \x08`, erasing the glyph the
///   terminal already cursored back over.
///
/// `line_len` is the framer's pre-feed line length (the single source of truth,
/// so no parallel buffer is kept here). `tx_idle` must be true only when no
/// protocol line is in flight, so echo bytes can't interleave one on the wire.
pub fn echo(b: u8, line_len: usize, tx_idle: bool, serial: &serial::Device) {
    if !tx_idle {
        return;
    }
    match b {
        b'\n' => {
            serial.tx_push(b"\r\n");
        }
        0x08 | 0x7F if line_len > 0 => {
            serial.tx_push(b" \x08");
        }
        _ => {}
    }
}
