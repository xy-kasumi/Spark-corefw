// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Modal coordinate-system state: which system G-code coordinates are relative
//! to (G53-G56), the live per-system offsets, and the last commanded target
//! used as the base for partial moves.
//!
//! Mirrors the C `gcode.c` modal state (`current_coord_system`, `coord_offsets`,
//! `last_target`/`last_target_avail`). Pure and host-testable; the firmware owns
//! one instance behind a mutex shared by the executor and the signal handler.

use crate::coords::{ActiveCoordSys, CoordSys, PosPhys};
use crate::gcode::MoveSpec;
use crate::settings::Axis;

pub struct CoordState {
    active: ActiveCoordSys,
    /// Last commanded target, in active-system coordinates. `None` after a
    /// cancel, which forces the next move to re-base off the live machine
    /// position.
    last_target: Option<PosPhys>,
    /// Per-system XYZ origins in machine coordinates, indexed by
    /// [`CoordSys::idx`]. C-axis is never offset.
    offsets: [PosPhys; 3],
}

impl CoordState {
    pub const fn new() -> Self {
        Self {
            active: ActiveCoordSys::Machine,
            // C zero-inits the static and leaves last_target_avail = true.
            last_target: Some(PosPhys::ZERO),
            offsets: [PosPhys::ZERO; 3],
        }
    }

    pub fn active(&self) -> ActiveCoordSys {
        self.active
    }

    /// The XYZ origin (machine coords) of `active`; `ZERO` for the machine system.
    pub fn offset_of(&self, active: ActiveCoordSys) -> PosPhys {
        match active {
            ActiveCoordSys::Machine => PosPhys::ZERO,
            ActiveCoordSys::Offset(cs) => self.offsets[cs.idx()],
        }
    }

    /// Set one axis of one system's origin (from the `cs.*.pos.*` settings path).
    pub fn set_offset(&mut self, cs: CoordSys, axis: Axis, value: f32) {
        let o = &mut self.offsets[cs.idx()];
        match axis {
            Axis::X => o.x = value,
            Axis::Y => o.y = value,
            Axis::Z => o.z = value,
        }
    }

    /// Select a new active system (G53-G56), re-anchoring `last_target` through
    /// machine coordinates so the held target stays at the same physical point.
    pub fn select(&mut self, new: ActiveCoordSys) {
        if let Some(lt) = self.last_target {
            let machine = lt.with_offset_added(self.offset_of(self.active));
            self.last_target = Some(machine.with_offset_removed(self.offset_of(new)));
        }
        self.active = new;
    }

    /// Resolve a move spec to a machine-coordinate target. Unspecified axes come
    /// from `last_target` (in active coords), or the live machine position
    /// converted into active coords when no target is cached. Updates
    /// `last_target`.
    pub fn resolve_move(&mut self, spec: &MoveSpec, current_machine: PosPhys) -> PosPhys {
        let off = self.offset_of(self.active);
        let mut base = self
            .last_target
            .unwrap_or_else(|| current_machine.with_offset_removed(off));
        if let Some(x) = spec.x {
            base.x = x;
        }
        if let Some(y) = spec.y {
            base.y = y;
        }
        if let Some(z) = spec.z {
            base.z = z;
        }
        if let Some(c) = spec.c {
            base.c = c;
        }
        self.last_target = Some(base);
        base.with_offset_added(off)
    }

    /// Mark `last_target` unavailable (move canceled).
    pub fn cancel(&mut self) {
        self.last_target = None;
    }
}

impl Default for CoordState {
    fn default() -> Self {
        Self::new()
    }
}
