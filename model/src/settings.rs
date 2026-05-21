// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Validated typed cache of (key, value) pairs the host can set/get over the
//! wire. Wire format is dotted paths to f32 (per `docs/settings.md`); storage
//! is a typed aggregate so refactors stay compile-checked.
//!
//! Per-key range validation and the side effects of changing a setting
//! (reconfiguring TMC, motion, etc.) live outside this module — this layer is
//! pure data + parsing + iteration so it can be host-tested without firmware.

use core::fmt::Write;

use heapless::String;

pub const N_MOTORS: usize = 7;
pub const N_AXES: usize = 3;
pub const N_COORD_SYSTEMS: usize = 3;

/// Path string capacity. Longest current path is "ts.servo.closems" (16
/// bytes); 20 leaves a little headroom for future names.
pub const PATH_CAP: usize = 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    pub fn idx(self) -> usize {
        match self {
            Axis::X => 0,
            Axis::Y => 1,
            Axis::Z => 2,
        }
    }

    fn name(self) -> &'static str {
        match self {
            Axis::X => "x",
            Axis::Y => "y",
            Axis::Z => "z",
        }
    }

    fn parse(s: &str) -> Option<Self> {
        match s {
            "x" => Some(Axis::X),
            "y" => Some(Axis::Y),
            "z" => Some(Axis::Z),
            _ => None,
        }
    }
}

pub use crate::coords::CoordSys;

impl CoordSys {
    fn name(self) -> &'static str {
        match self {
            CoordSys::W => "w",
            CoordSys::G => "g",
            CoordSys::Ts => "ts",
        }
    }

    fn parse(s: &str) -> Option<Self> {
        match s {
            "w" => Some(CoordSys::W),
            "g" => Some(CoordSys::G),
            "ts" => Some(CoordSys::Ts),
            _ => None,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct MotorCfg {
    pub current: f32,
    pub microstep: f32,
    pub thresh: f32,
    pub unitsteps: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct AxisHomeCfg {
    pub origin: f32,
    pub phase: f32,
    pub side: f32,
    pub travel: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct CoordSysCfg {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct ToolSupplyCfg {
    pub close_ms: f32,
    pub open_ms: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct Settings {
    pub motors: [MotorCfg; N_MOTORS],
    pub axes: [AxisHomeCfg; N_AXES],
    pub cs: [CoordSysCfg; N_COORD_SYSTEMS],
    pub ts: ToolSupplyCfg,
}

impl Settings {
    /// Defaults mirror the C `settings[]` array exactly so wire-format output
    /// at boot matches the C firmware.
    pub const fn defaults() -> Self {
        let common_motor = MotorCfg {
            current: 30.0,
            microstep: 32.0,
            thresh: 2.0,
            unitsteps: 200.0,
        };
        let mut s = Self {
            motors: [common_motor; N_MOTORS],
            axes: [AxisHomeCfg {
                origin: 0.0,
                phase: 0.0,
                side: 1.0,
                travel: 500.0,
            }; N_AXES],
            cs: [CoordSysCfg {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }; N_COORD_SYSTEMS],
            ts: ToolSupplyCfg {
                close_ms: 1.3,
                open_ms: 1.6,
            },
        };
        s.motors[1].unitsteps = -200.0;
        s.motors[2].unitsteps = -200.0;
        s.motors[5].unitsteps = 6400.0;
        s.motors[6].unitsteps = 203.8;
        s.axes[0].phase = 1.0; // X
        s.axes[1].phase = 2.0; // Y
        s.axes[1].side = -1.0; // Y
        s
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SetError {
    /// Value is NaN or infinite.
    NotFinite,
}

/// Every leaf path in the settings tree. Adding/removing a setting means
/// adding/removing a variant here; the exhaustive matches on `path`, `read`,
/// and `write` then force every other site to be updated.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SettingId {
    AxisHomeOrigin(Axis),
    AxisHomePhase(Axis),
    AxisHomeSide(Axis),
    AxisHomeTravel(Axis),
    CsPos(CoordSys, Axis),
    MotorCurrent(u8),
    MotorMicrostep(u8),
    MotorThresh(u8),
    MotorUnitsteps(u8),
    TsServoCloseMs,
    TsServoOpenMs,
}

impl SettingId {
    pub fn parse(path: &str) -> Option<Self> {
        let mut segs = path.split('.');
        let head = segs.next()?;
        let id = match head {
            "a" => parse_axis(&mut segs)?,
            "cs" => parse_cs(&mut segs)?,
            "m" => parse_motor(&mut segs)?,
            "ts" => parse_ts(&mut segs)?,
            _ => return None,
        };
        if segs.next().is_some() {
            return None;
        }
        Some(id)
    }

    pub fn path(self) -> String<PATH_CAP> {
        let mut s = String::<PATH_CAP>::new();
        // write! into a heapless::String only fails on capacity overflow; all
        // current paths fit, and the test below pins that.
        let _ = match self {
            SettingId::AxisHomeOrigin(a) => write!(s, "a.{}.home.origin", a.name()),
            SettingId::AxisHomePhase(a) => write!(s, "a.{}.home.phase", a.name()),
            SettingId::AxisHomeSide(a) => write!(s, "a.{}.home.side", a.name()),
            SettingId::AxisHomeTravel(a) => write!(s, "a.{}.home.travel", a.name()),
            SettingId::CsPos(c, a) => write!(s, "cs.{}.pos.{}", c.name(), a.name()),
            SettingId::MotorCurrent(i) => write!(s, "m.{}.current", i),
            SettingId::MotorMicrostep(i) => write!(s, "m.{}.microstep", i),
            SettingId::MotorThresh(i) => write!(s, "m.{}.thresh", i),
            SettingId::MotorUnitsteps(i) => write!(s, "m.{}.unitsteps", i),
            SettingId::TsServoCloseMs => write!(s, "ts.servo.closems"),
            SettingId::TsServoOpenMs => write!(s, "ts.servo.openms"),
        };
        s
    }

    pub fn read(self, s: &Settings) -> f32 {
        match self {
            SettingId::AxisHomeOrigin(a) => s.axes[a.idx()].origin,
            SettingId::AxisHomePhase(a) => s.axes[a.idx()].phase,
            SettingId::AxisHomeSide(a) => s.axes[a.idx()].side,
            SettingId::AxisHomeTravel(a) => s.axes[a.idx()].travel,
            SettingId::CsPos(c, a) => {
                let cs = &s.cs[c.idx()];
                match a {
                    Axis::X => cs.x,
                    Axis::Y => cs.y,
                    Axis::Z => cs.z,
                }
            }
            SettingId::MotorCurrent(i) => s.motors[i as usize].current,
            SettingId::MotorMicrostep(i) => s.motors[i as usize].microstep,
            SettingId::MotorThresh(i) => s.motors[i as usize].thresh,
            SettingId::MotorUnitsteps(i) => s.motors[i as usize].unitsteps,
            SettingId::TsServoCloseMs => s.ts.close_ms,
            SettingId::TsServoOpenMs => s.ts.open_ms,
        }
    }

    pub fn write(self, s: &mut Settings, v: f32) -> Result<(), SetError> {
        if !v.is_finite() {
            return Err(SetError::NotFinite);
        }
        match self {
            SettingId::AxisHomeOrigin(a) => s.axes[a.idx()].origin = v,
            SettingId::AxisHomePhase(a) => s.axes[a.idx()].phase = v,
            SettingId::AxisHomeSide(a) => s.axes[a.idx()].side = v,
            SettingId::AxisHomeTravel(a) => s.axes[a.idx()].travel = v,
            SettingId::CsPos(c, a) => {
                let cs = &mut s.cs[c.idx()];
                match a {
                    Axis::X => cs.x = v,
                    Axis::Y => cs.y = v,
                    Axis::Z => cs.z = v,
                }
            }
            SettingId::MotorCurrent(i) => s.motors[i as usize].current = v,
            SettingId::MotorMicrostep(i) => s.motors[i as usize].microstep = v,
            SettingId::MotorThresh(i) => s.motors[i as usize].thresh = v,
            SettingId::MotorUnitsteps(i) => s.motors[i as usize].unitsteps = v,
            SettingId::TsServoCloseMs => s.ts.close_ms = v,
            SettingId::TsServoOpenMs => s.ts.open_ms = v,
        }
        Ok(())
    }
}

/// Iterate every setting in wire order. The order matches the C `settings[]`
/// array so the `stg` p-state byte-stream stays stable.
pub fn iter_all() -> impl Iterator<Item = SettingId> {
    use Axis::{X, Y, Z};
    use CoordSys::{Ts, G, W};
    use SettingId::*;

    let axes = [X, Y, Z].into_iter().flat_map(|a| {
        [
            AxisHomeOrigin(a),
            AxisHomePhase(a),
            AxisHomeSide(a),
            AxisHomeTravel(a),
        ]
    });
    let cs = [G, W, Ts]
        .into_iter()
        .flat_map(|c| [CsPos(c, X), CsPos(c, Y), CsPos(c, Z)]);
    let motors = (0u8..N_MOTORS as u8).flat_map(|i| {
        [
            MotorCurrent(i),
            MotorMicrostep(i),
            MotorThresh(i),
            MotorUnitsteps(i),
        ]
    });
    let ts = [TsServoCloseMs, TsServoOpenMs].into_iter();
    axes.chain(cs).chain(motors).chain(ts)
}

fn parse_axis<'a>(segs: &mut core::str::Split<'a, char>) -> Option<SettingId> {
    let axis = Axis::parse(segs.next()?)?;
    if segs.next()? != "home" {
        return None;
    }
    Some(match segs.next()? {
        "origin" => SettingId::AxisHomeOrigin(axis),
        "phase" => SettingId::AxisHomePhase(axis),
        "side" => SettingId::AxisHomeSide(axis),
        "travel" => SettingId::AxisHomeTravel(axis),
        _ => return None,
    })
}

fn parse_cs<'a>(segs: &mut core::str::Split<'a, char>) -> Option<SettingId> {
    let cs = CoordSys::parse(segs.next()?)?;
    if segs.next()? != "pos" {
        return None;
    }
    let axis = Axis::parse(segs.next()?)?;
    Some(SettingId::CsPos(cs, axis))
}

fn parse_motor<'a>(segs: &mut core::str::Split<'a, char>) -> Option<SettingId> {
    let idx: u8 = segs.next()?.parse().ok()?;
    if idx as usize >= N_MOTORS {
        return None;
    }
    Some(match segs.next()? {
        "current" => SettingId::MotorCurrent(idx),
        "microstep" => SettingId::MotorMicrostep(idx),
        "thresh" => SettingId::MotorThresh(idx),
        "unitsteps" => SettingId::MotorUnitsteps(idx),
        _ => return None,
    })
}

fn parse_ts<'a>(segs: &mut core::str::Split<'a, char>) -> Option<SettingId> {
    if segs.next()? != "servo" {
        return None;
    }
    Some(match segs.next()? {
        "closems" => SettingId::TsServoCloseMs,
        "openms" => SettingId::TsServoOpenMs,
        _ => return None,
    })
}
