// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Per-axis homing parameters (`a.<axis>.home.*`). A settings-agnostic mediator:
//! the write dispatcher pushes values in via [`HomingConfig::set`]; `exec_home`
//! reads them out via [`HomingConfig::axis`]. Neither side touches the repo.
//!
//! Born with sane defaults so the module is always valid; the boot apply-all
//! overrides them with the configured values.

use model::settings;

#[derive(Clone, Copy, Debug)]
pub struct Axis {
    /// Home position's coordinate.
    pub origin: f32,
    /// Sequential homing phase; same-phase axes home together.
    pub phase: f32,
    /// Homing direction: -1 towards negative, +1 towards positive.
    pub side: f32,
    /// Max distance (mm) to travel when homing.
    pub travel: f32,
}

impl Default for Axis {
    fn default() -> Self {
        Self {
            origin: 0.0,
            phase: 0.0,
            side: 1.0,
            travel: 500.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Config {
    // TODO: should not be using int-index for coords::Axis-indexed data
    axes: [Axis; 3],
}

impl Config {
    /// Apply one `a.<axis>.home.<prop>` value. `Err` on an unknown property
    /// (the dispatcher turns that into an unknown-key error).
    pub fn set(&mut self, axis: settings::Axis, prop: &str, val: f32) -> Result<(), ()> {
        let a = &mut self.axes[settings::axis_idx(axis)];
        match prop {
            "origin" => a.origin = val,
            "phase" => a.phase = val,
            "side" => a.side = val,
            "travel" => a.travel = val,
            _ => return Err(()),
        }
        Ok(())
    }

    pub fn axis(&self, axis: settings::Axis) -> Axis {
        self.axes[settings::axis_idx(axis)]
    }
}
