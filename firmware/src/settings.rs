// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Write dispatcher: the one place that maps a raw settings key string onto the
//! owning subsystem's setter. Parses indices/axes out of the key but invents no
//! per-key names; subsystems themselves stay settings-agnostic. [`write`] is the
//! single path that validates, applies, and commits a value to the repo.

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use model::coordstate::CoordState;
use model::gcode::ToolSupplyState;
use model::pstate::{Line, PsType};
use model::settings::{Axis, CoordSys, Repo, SettingsVal, STG_KEY_SEGS_CAP};

use crate::board::{MotorConfig, ToolSupply, NUM_MOTORS};
use crate::homing::HomingConfig;
use crate::line_tx::LineTx;
use crate::motion::Motion;
use crate::wirefeed::Wirefeed;

pub type SharedTmc = Mutex<NoopRawMutex, [MotorConfig; NUM_MOTORS]>;

#[derive(Debug)]
pub enum Error {
    UnknownKey,
    ApplyFailed,
}

fn motor_idx(s: &str) -> Result<usize, Error> {
    let idx: usize = s.parse().map_err(|_| Error::UnknownKey)?;
    (idx < NUM_MOTORS).then_some(idx).ok_or(Error::UnknownKey)
}

/// The single place that maps a raw key string onto subsystem's setter.
async fn dispatch(
    key: &str,
    val: f32,
    motion: &Mutex<NoopRawMutex, Motion>,
    tmc: &SharedTmc,
    coord: &Mutex<NoopRawMutex, CoordState>,
    wirefeed: &Mutex<NoopRawMutex, Wirefeed>,
    toolsupply: &Mutex<NoopRawMutex, ToolSupply>,
    homing: &Mutex<NoopRawMutex, HomingConfig>,
) -> Result<(), Error> {
    let mut parts: [&str; STG_KEY_SEGS_CAP] = [""; STG_KEY_SEGS_CAP];
    let mut n = 0;
    for seg in key.split('.') {
        if n == parts.len() {
            return Err(Error::UnknownKey);
        }
        parts[n] = seg;
        n += 1;
    }

    match parts[..n] {
        ["m", i, "current"] => {
            let v = val as u32;
            tmc.lock().await[motor_idx(i)?]
                .set_current(v, v)
                .await
                .map_err(|_| Error::ApplyFailed)
        }
        ["m", i, "microstep"] => tmc.lock().await[motor_idx(i)?]
            .set_microstep(val as u32)
            .await
            .map_err(|_| Error::ApplyFailed),
        ["m", i, "thresh"] => {
            let idx = motor_idx(i)?;
            // Negative threshold means "disable stall detection": no register write.
            if val >= 0.0 {
                tmc.lock().await[idx]
                    .set_stallguard_threshold(val as u8)
                    .await
                    .map_err(|_| Error::ApplyFailed)?;
            }
            Ok(())
        }
        ["m", "6", "unitsteps"] => {
            wirefeed.lock().await.set_unitsteps(val);
            Ok(())
        }
        ["m", i, "unitsteps"] => {
            // m0..=m3 feed Motion's per-axis calibration; m4/m5 have no target.
            motion
                .lock()
                .await
                .set_motor_unitsteps(motor_idx(i)? as u8, val);
            Ok(())
        }
        ["cs", c, "pos", a] => {
            let cs = CoordSys::parse(c).ok_or(Error::UnknownKey)?;
            let axis = Axis::parse(a).ok_or(Error::UnknownKey)?;
            coord.lock().await.set_offset(cs, axis, val);
            Ok(())
        }
        ["a", a, "home", prop] => {
            let axis = Axis::parse(a).ok_or(Error::UnknownKey)?;
            homing
                .lock()
                .await
                .set(axis, prop, val)
                .map_err(|_| Error::UnknownKey)
        }
        ["ts", "servo", "openms"] => {
            toolsupply
                .lock()
                .await
                .configure(ToolSupplyState::Open, val)
                .await;
            Ok(())
        }
        ["ts", "servo", "closems"] => {
            toolsupply
                .lock()
                .await
                .configure(ToolSupplyState::Closed, val)
                .await;
            Ok(())
        }
        _ => Err(Error::UnknownKey),
    }
}


#[allow(clippy::too_many_arguments)]
pub async fn write(
    repo: &mut Repo,
    key: &str,
    val: SettingsVal,
    motion: &Mutex<NoopRawMutex, Motion>,
    tmc: &SharedTmc,
    coord: &Mutex<NoopRawMutex, CoordState>,
    wirefeed: &Mutex<NoopRawMutex, Wirefeed>,
    toolsupply: &Mutex<NoopRawMutex, ToolSupply>,
    homing: &Mutex<NoopRawMutex, HomingConfig>,
) -> Result<(), Error> {
    if !repo.contains(key) {
        return Err(Error::UnknownKey);
    }
    dispatch(key, val.get(), motion, tmc, coord, wirefeed, toolsupply, homing).await?;
    let _ = repo.set(key, val); // infallible: key present
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub async fn apply_all(
    repo: &Repo,
    motion: &Mutex<NoopRawMutex, Motion>,
    tmc: &SharedTmc,
    coord: &Mutex<NoopRawMutex, CoordState>,
    wirefeed: &Mutex<NoopRawMutex, Wirefeed>,
    toolsupply: &Mutex<NoopRawMutex, ToolSupply>,
    homing: &Mutex<NoopRawMutex, HomingConfig>,
    line_tx: &LineTx,
) -> bool {
    for (key, v) in repo.iter() {
        if dispatch(key, v, motion, tmc, coord, wirefeed, toolsupply, homing)
            .await
            .is_err()
        {
            let _ = line_tx.try_send(Line::new(PsType::Init).bool("settings.ok", false));
            let _ = line_tx.try_send(Line::new(PsType::Init).str_val("settings.msg", key));
            return false;
        }
    }
    let _ = line_tx.try_send(Line::new(PsType::Init).bool("settings.ok", true));
    true
}
