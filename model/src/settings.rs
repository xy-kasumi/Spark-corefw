// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Repo of all settings. Source of truth for `get`.
//! Write is handled by dispatcher.

use core::fmt::Write;

pub const N_MOTORS: usize = 7;

/// Settings key max length.
/// cf. `a.x.home.origin` is 15 bytes
pub const STG_KEY_CAP: usize = 20;
/// Max number of key segments
/// cf. `cs.w.pos.x` has 4 segs
pub const STG_KEY_SEGS_CAP: usize = 5;

pub use crate::coords::{Axis, CoordSys};

/// Settings-key segment. `Machine` has no settable offset and so no key prefix.
pub fn cs_name(cs: CoordSys) -> &'static str {
    match cs {
        CoordSys::Work => "w",
        CoordSys::Grinder => "g",
        CoordSys::Machine => unreachable!("machine coordsys has no settings prefix"),
    }
}

pub fn cs_parse(s: &str) -> Option<CoordSys> {
    match s {
        "w" => Some(CoordSys::Work),
        "g" => Some(CoordSys::Grinder),
        _ => None,
    }
}

/// Settings-key segment.
pub fn axis_name(axis: Axis) -> &'static str {
    match axis {
        Axis::X => "x",
        Axis::Y => "y",
        Axis::Z => "z",
    }
}

pub fn axis_parse(s: &str) -> Option<Axis> {
    match s {
        "x" => Some(Axis::X),
        "y" => Some(Axis::Y),
        "z" => Some(Axis::Z),
        _ => None,
    }
}

/// [`Repo`]'s capacity. Needs to be bigger than number of keys.
const STG_CAP: usize = 64;

/// Valid settings value (finite).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Value(f32);

impl Value {
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
    map: heapless::LinearMap<heapless::String<STG_KEY_CAP>, f32, STG_CAP>,
}

impl Repo {
    /// The one place that lists every key and its default; key order falls out
    /// of this function's loop structure.
    pub fn defaults() -> Self {
        let mut map = heapless::LinearMap::<heapless::String<STG_KEY_CAP>, f32, STG_CAP>::new();
        {
            let mut ins = |args: core::fmt::Arguments, v: f32| {
                let mut k = heapless::String::<STG_KEY_CAP>::new();
                k.write_fmt(args).expect("key fits PATH_CAP");
                map.insert(k, v).expect("CAP fits all keys");
            };
            for a in [Axis::X, Axis::Y, Axis::Z] {
                let n = axis_name(a);
                ins(format_args!("a.{n}.home.origin"), 0.0);
                ins(format_args!("a.{n}.home.phase"), default_phase(a));
                ins(format_args!("a.{n}.home.side"), default_side(a));
                ins(format_args!("a.{n}.home.travel"), 500.0);
            }
            for c in [CoordSys::Grinder, CoordSys::Work] {
                let cn = cs_name(c);
                for a in [Axis::X, Axis::Y, Axis::Z] {
                    ins(format_args!("cs.{cn}.pos.{}", axis_name(a)), 0.0);
                }
            }
            for m in 0..N_MOTORS {
                ins(format_args!("m.{m}.current"), 30.0);
                ins(format_args!("m.{m}.microstep"), 32.0);
                ins(format_args!("m.{m}.thresh"), 2.0);
                ins(format_args!("m.{m}.unitsteps"), default_unitsteps(m));
            }
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
    pub fn set(&mut self, key: &str, v: Value) -> Result<(), Unknown> {
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

/// Default home phase: X=1, Y=2, Z=0.
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
/// wire feed.
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
    fn value_is_finite() {
        assert_eq!(Value::parse("3.14").map(|v| v.get()), Some(3.14));
        assert!(Value::parse("NaN").is_none());
        assert!(Value::parse("inf").is_none());
        assert!(Value::parse("-inf").is_none());
        assert!(Value::parse("abc").is_none());
    }
}
