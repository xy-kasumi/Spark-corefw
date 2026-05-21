// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! The settings repo: an explicit `key -> f32` map keyed by the raw dotted wire
//! path (per `docs/settings.md`), plus value parsing. Source of truth for
//! `read`/`get`; host-testable with no firmware deps.
//!
//! Per-key range validation and the side effects of a change (reconfiguring
//! TMC, motion, etc.) live in the firmware write dispatcher, not here.

use core::fmt::Write;

use heapless::String;

pub const N_MOTORS: usize = 7;

/// Settings key max length.
/// cf. `ts.servo.closems` is 16 bytes
pub const STG_KEY_CAP: usize = 20;
/// Max number of key segments
/// cf. `cs.w.pos.x` has 4 segs
pub const STG_KEY_SEGS_CAP : usize = 5;

pub use crate::coords::{Axis, CoordSys};

/// [`Repo`]'s capacity. Needs to be bigger than number of keys.
const STG_CAP: usize = 64;

/// Valid settings value (finite).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SettingsVal(f32);

impl SettingsVal {
    /// Parse a decimal float, rejecting NaN/inf (per `docs/settings.md`).
    pub fn parse(s: &str) -> Option<Self> {
        let v: f32 = s.parse().ok()?;
        v.is_finite().then_some(Self(v))
    }

    pub fn get(self) -> f32 {
        self.0
    }
}

/// `read`/`write` failed because the key is not in the repo.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Unknown;

/// The settings repo.
/// Write goes through subsystems' dispatcher and, map is updated only on success.
/// Repo is never consumed by subsystems.
///
/// Iteration is insertion order (see [`Repo::defaults`]).
#[derive(Clone, Debug, PartialEq)]
pub struct Repo {
    map: heapless::LinearMap<String<STG_KEY_CAP>, f32, STG_CAP>,
}

impl Repo {
    /// The one place that lists every key and its default; key order falls out
    /// of this function's loop structure.
    pub fn defaults() -> Self {
        let mut map = heapless::LinearMap::<String<STG_KEY_CAP>, f32, STG_CAP>::new();
        {
            let mut ins = |args: core::fmt::Arguments, v: f32| {
                let mut k = String::<STG_KEY_CAP>::new();
                k.write_fmt(args).expect("key fits PATH_CAP");
                map.insert(k, v).expect("CAP fits all keys");
            };
            for a in [Axis::X, Axis::Y, Axis::Z] {
                let n = a.name();
                ins(format_args!("a.{n}.home.origin"), 0.0);
                ins(format_args!("a.{n}.home.phase"), default_phase(a));
                ins(format_args!("a.{n}.home.side"), default_side(a));
                ins(format_args!("a.{n}.home.travel"), 500.0);
            }
            for c in [CoordSys::G, CoordSys::W, CoordSys::Ts] {
                let cn = c.name();
                for a in [Axis::X, Axis::Y, Axis::Z] {
                    ins(format_args!("cs.{cn}.pos.{}", a.name()), 0.0);
                }
            }
            for m in 0..N_MOTORS {
                ins(format_args!("m.{m}.current"), 30.0);
                ins(format_args!("m.{m}.microstep"), 32.0);
                ins(format_args!("m.{m}.thresh"), 2.0);
                ins(format_args!("m.{m}.unitsteps"), default_unitsteps(m));
            }
            ins(format_args!("ts.servo.closems"), 1.3);
            ins(format_args!("ts.servo.openms"), 1.6);
        }
        Self { map }
    }

    pub fn get(&self, key: &str) -> Option<f32> {
        self.map
            .iter()
            .find(|(k, _)| k.as_str() == key)
            .map(|(_, v)| *v)
    }

    pub fn contains(&self, key: &str) -> bool {
        self.map.iter().any(|(k, _)| k.as_str() == key)
    }

    /// Commit a value. Errors if the key is absent — the repo's key set is
    /// fixed at [`defaults`](Self::defaults).
    pub fn set(&mut self, key: &str, v: SettingsVal) -> Result<(), Unknown> {
        let slot = self
            .map
            .iter_mut()
            .find(|(k, _)| k.as_str() == key)
            .map(|(_, v)| v)
            .ok_or(Unknown)?;
        *slot = v.get();
        Ok(())
    }

    pub fn iter(&self) -> impl Iterator<Item = (&str, f32)> {
        self.map.iter().map(|(k, v)| (k.as_str(), *v))
    }
}

/// Default home phase: X=1, Y=2, Z=0 (mirrors the legacy `settings[]`).
fn default_phase(a: Axis) -> f32 {
    match a {
        Axis::X => 1.0,
        Axis::Y => 2.0,
        Axis::Z => 0.0,
    }
}

/// Default home side: Y homes negative, X/Z positive.
fn default_side(a: Axis) -> f32 {
    match a {
        Axis::Y => -1.0,
        _ => 1.0,
    }
}

/// Default microsteps per +1 unit. Motors 1/2 invert; 5 is the wire spool, 6 the
/// wire feed (mirrors the legacy `settings[]`).
fn default_unitsteps(motor: usize) -> f32 {
    match motor {
        1 | 2 => -200.0,
        5 => 6400.0,
        6 => 203.8,
        _ => 200.0,
    }
}

#[cfg(test)]
mod repo_tests {
    use super::*;

    #[test]
    fn settingsval_rejects_nonfinite() {
        assert_eq!(SettingsVal::parse("3.14").map(|v| v.get()), Some(3.14));
        assert!(SettingsVal::parse("NaN").is_none());
        assert!(SettingsVal::parse("inf").is_none());
        assert!(SettingsVal::parse("-inf").is_none());
        assert!(SettingsVal::parse("abc").is_none());
    }
}
