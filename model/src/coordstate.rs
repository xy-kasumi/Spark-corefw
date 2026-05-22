// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

use crate::coords;
use crate::gcode;
use crate::settings;

pub struct CoordState {
    active: coords::ActiveCoordSys,
    /// Last commanded target, in active-system coordinates. `None` after a
    /// cancel, which forces the next move to re-base off the live machine
    /// position.
    last_target: Option<coords::PosPhys>,
    /// Per-system XYZ origins in machine coordinates, indexed by
    /// [`CoordSys::idx`]. C-axis is never offset.
    offsets: [coords::PosPhys; 3],
}

impl CoordState {
    pub const fn new() -> Self {
        Self {
            active: coords::ActiveCoordSys::Machine,
            // Initial state: a zeroed target is already available.
            last_target: Some(coords::PosPhys::ZERO),
            offsets: [coords::PosPhys::ZERO; 3],
        }
    }

    pub fn active(&self) -> coords::ActiveCoordSys {
        self.active
    }

    /// The XYZ origin (machine coords) of `active`; `ZERO` for the machine system.
    pub fn offset_of(&self, active: coords::ActiveCoordSys) -> coords::PosPhys {
        match active {
            coords::ActiveCoordSys::Machine => coords::PosPhys::ZERO,
            coords::ActiveCoordSys::Offset(cs) => self.offsets[cs.idx()],
        }
    }

    /// Set one axis of one system's origin (from the `cs.*.pos.*` settings path).
    pub fn set_offset(&mut self, cs: coords::CoordSys, axis: settings::Axis, value: f32) {
        let o = &mut self.offsets[cs.idx()];
        match axis {
            settings::Axis::X => o.x = value,
            settings::Axis::Y => o.y = value,
            settings::Axis::Z => o.z = value,
        }
    }

    /// Select a new active system, re-anchoring `last_target` through
    /// machine coordinates so the held target stays at the same physical point.
    pub fn select(&mut self, new: coords::ActiveCoordSys) {
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
    pub fn resolve_move(
        &mut self,
        spec: &gcode::MoveSpec,
        current_machine: coords::PosPhys,
    ) -> coords::PosPhys {
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
