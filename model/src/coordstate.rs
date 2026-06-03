// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

use crate::coords;
use crate::gcode;

pub struct CoordState {
    active: coords::CoordSys,
    /// Last commanded target, in active-system coordinates. `None` after a
    /// cancel, which forces the next move to re-base off the live machine
    /// position.
    last_target: Option<coords::PosPhys>,
    /// Per-system XYZ origins in machine coordinates. C-axis is never offset.
    /// (CoordSys::Machine slot is wasted)
    offsets: enum_map::EnumMap<coords::CoordSys, coords::PosPhys>,
    /// Extra work-Y origin shift from G29 calibration, added on top of
    /// `offsets[Work].y`. Only G29 touches it; `set_offset` leaves it alone.
    work_offset_y: f32,
}

impl CoordState {
    pub fn new() -> Self {
        Self {
            active: coords::CoordSys::Machine,
            // Initial state: a zeroed target is already available.
            last_target: Some(coords::PosPhys::ZERO),
            offsets: enum_map::EnumMap::default(),
            work_offset_y: 0.0,
        }
    }

    pub fn active(&self) -> coords::CoordSys {
        self.active
    }

    /// The XYZ origin (machine coords) of `active`; `ZERO` for the machine system.
    /// The work system folds in the G29 calibration ([`calibrate_work_y`]).
    ///
    /// [`calibrate_work_y`]: Self::calibrate_work_y
    pub fn offset_of(&self, active: coords::CoordSys) -> coords::PosPhys {
        let mut off = self.offsets[active];
        if active == coords::CoordSys::Work {
            off.y += self.work_offset_y;
        }
        off
    }

    /// The G29 work-Y calibration shift (`calib.work.y`).
    pub fn work_offset_y(&self) -> f32 {
        self.work_offset_y
    }

    /// Set one axis of one system's origin (from the `cs.*.pos.*` settings path).
    pub fn set_offset(&mut self, cs: coords::CoordSys, axis: coords::Axis, value: f32) {
        debug_assert!(
            cs != coords::CoordSys::Machine,
            "machine coordsys has no settable offset"
        );
        let o = &mut self.offsets[cs];
        match axis {
            coords::Axis::X => o.x = value,
            coords::Axis::Y => o.y = value,
            coords::Axis::Z => o.z = value,
        }
    }

    /// Clear the G29 work-Y calibration (back to `offsets[Work].y` alone). G29
    /// does this first so its probe positions are taken in the un-calibrated frame.
    pub fn clear_work_y_calibration(&mut self) {
        self.set_work_offset_y(0.0);
    }

    /// Set the work-Y calibration so machine-Y `center` reads work-Y 0 (G29).
    pub fn calibrate_work_y(&mut self, center_machine_y: f32) {
        self.set_work_offset_y(center_machine_y - self.offsets[coords::CoordSys::Work].y);
    }

    /// Update `work_offset_y`, re-anchoring `last_target` through machine
    /// coordinates so the held target stays at the same physical point.
    fn set_work_offset_y(&mut self, value: f32) {
        let machine = self
            .last_target
            .map(|lt| lt.with_offset_added(self.offset_of(self.active)));
        self.work_offset_y = value;
        self.last_target = machine.map(|m| m.with_offset_removed(self.offset_of(self.active)));
    }

    /// Select a new active system, re-anchoring `last_target` through
    /// machine coordinates so the held target stays at the same physical point.
    pub fn select(&mut self, new: coords::CoordSys) {
        if let Some(lt) = self.last_target {
            let machine = lt.with_offset_added(self.offset_of(self.active));
            self.last_target = Some(machine.with_offset_removed(self.offset_of(new)));
        }
        self.active = new;
    }

    /// Convert a machine-coordinate position into the active system — the offset
    /// inverse of what [`resolve_move`] applies.
    ///
    /// [`resolve_move`]: Self::resolve_move
    pub fn to_active(&self, machine: coords::PosPhys) -> coords::PosPhys {
        machine.with_offset_removed(self.offset_of(self.active))
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
