// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Write dispatcher: the one place that maps a raw settings key string onto the
//! owning subsystem's setter. Parses indices/axes out of the key but invents no
//! per-key names; subsystems themselves stay settings-agnostic. [`write`] is the
//! single path that validates, applies, and commits a value to the repo.

use embassy_sync::blocking_mutex::raw;
use embassy_sync::mutex;
use model::coordstate;
use model::gcode;
use model::pstate;
use model::settings;

use crate::board;
use crate::homing;
use crate::line_tx;
use crate::motion;
use crate::wirefeed;

pub type SharedTmc = mutex::Mutex<raw::NoopRawMutex, [board::MotorConfig; board::NUM_MOTORS]>;

#[derive(Debug)]
pub enum Error {
    UnknownKey,
    ApplyFailed,
}

fn motor_idx(s: &str) -> Result<usize, Error> {
    let idx: usize = s.parse().map_err(|_| Error::UnknownKey)?;
    (idx < board::NUM_MOTORS)
        .then_some(idx)
        .ok_or(Error::UnknownKey)
}

/// The single place that maps a raw key string onto subsystem's setter.
async fn dispatch(
    key: &str,
    val: f32,
    motion: &mutex::Mutex<raw::NoopRawMutex, motion::Motion>,
    tmc: &SharedTmc,
    coord: &mutex::Mutex<raw::NoopRawMutex, coordstate::CoordState>,
    wirefeed: &mutex::Mutex<raw::NoopRawMutex, wirefeed::Wirefeed>,
    toolsupply: &mutex::Mutex<raw::NoopRawMutex, board::ToolSupply>,
    homing: &mutex::Mutex<raw::NoopRawMutex, homing::Config>,
) -> Result<(), Error> {
    let mut parts: [&str; settings::STG_KEY_SEGS_CAP] = [""; settings::STG_KEY_SEGS_CAP];
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
            let cs = settings::CoordSys::parse(c).ok_or(Error::UnknownKey)?;
            let axis = settings::Axis::parse(a).ok_or(Error::UnknownKey)?;
            coord.lock().await.set_offset(cs, axis, val);
            Ok(())
        }
        ["a", a, "home", prop] => {
            let axis = settings::Axis::parse(a).ok_or(Error::UnknownKey)?;
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
                .configure(gcode::ToolSupplyState::Open, val)
                .await;
            Ok(())
        }
        ["ts", "servo", "closems"] => {
            toolsupply
                .lock()
                .await
                .configure(gcode::ToolSupplyState::Closed, val)
                .await;
            Ok(())
        }
        _ => Err(Error::UnknownKey),
    }
}

#[allow(clippy::too_many_arguments)]
pub async fn write(
    repo: &mut settings::Repo,
    key: &str,
    val: settings::Value,
    motion: &mutex::Mutex<raw::NoopRawMutex, motion::Motion>,
    tmc: &SharedTmc,
    coord: &mutex::Mutex<raw::NoopRawMutex, coordstate::CoordState>,
    wirefeed: &mutex::Mutex<raw::NoopRawMutex, wirefeed::Wirefeed>,
    toolsupply: &mutex::Mutex<raw::NoopRawMutex, board::ToolSupply>,
    homing: &mutex::Mutex<raw::NoopRawMutex, homing::Config>,
) -> Result<(), Error> {
    if !repo.contains(key) {
        return Err(Error::UnknownKey);
    }
    dispatch(
        key,
        val.get(),
        motion,
        tmc,
        coord,
        wirefeed,
        toolsupply,
        homing,
    )
    .await?;
    let _ = repo.set(key, val); // infallible: key present
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub async fn apply_all(
    repo: &settings::Repo,
    motion: &mutex::Mutex<raw::NoopRawMutex, motion::Motion>,
    tmc: &SharedTmc,
    coord: &mutex::Mutex<raw::NoopRawMutex, coordstate::CoordState>,
    wirefeed: &mutex::Mutex<raw::NoopRawMutex, wirefeed::Wirefeed>,
    toolsupply: &mutex::Mutex<raw::NoopRawMutex, board::ToolSupply>,
    homing: &mutex::Mutex<raw::NoopRawMutex, homing::Config>,
    line_tx: &line_tx::LineTx,
) -> bool {
    for (key, v) in repo.iter() {
        if dispatch(key, v, motion, tmc, coord, wirefeed, toolsupply, homing)
            .await
            .is_err()
        {
            let _ = line_tx
                .try_send(pstate::Line::new(pstate::PsType::Init).bool("settings.ok", false));
            let _ = line_tx
                .try_send(pstate::Line::new(pstate::PsType::Init).str_val("settings.msg", key));
            return false;
        }
    }
    let _ = line_tx.try_send(pstate::Line::new(pstate::PsType::Init).bool("settings.ok", true));
    true
}
