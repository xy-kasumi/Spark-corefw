// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Write dispatcher: the one place that maps a raw settings key string onto the
//! owning subsystem's setter. Parses indices/axes out of the key but invents no
//! per-key names; subsystems themselves stay settings-agnostic. [`write`] is the
//! single path that validates, applies, and commits a value to the repo.

use embassy_sync::blocking_mutex::raw;
use embassy_sync::mutex;
use model::settings;

use crate::board;
use crate::homing;
use crate::SharedCore;

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
    core: &SharedCore,
    tmc: &SharedTmc,
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
        ["m", i, "unitsteps"] => {
            core.lock().await.motors.set_unitsteps(motor_idx(i)?, val);
            Ok(())
        }
        ["cs", c, "pos", a] => {
            let cs = settings::cs_parse(c).ok_or(Error::UnknownKey)?;
            let axis = settings::axis_parse(a).ok_or(Error::UnknownKey)?;
            core.lock().await.coord.set_offset(cs, axis, val);
            Ok(())
        }
        ["a", a, "home", prop] => {
            let axis = settings::axis_parse(a).ok_or(Error::UnknownKey)?;
            homing
                .lock()
                .await
                .set(axis, prop, val)
                .map_err(|_| Error::UnknownKey)
        }
        _ => Err(Error::UnknownKey),
    }
}

pub async fn write(
    repo: &mut settings::Repo,
    key: &str,
    val: settings::Value,
    core: &SharedCore,
    tmc: &SharedTmc,
    homing: &mutex::Mutex<raw::NoopRawMutex, homing::Config>,
) -> Result<(), Error> {
    if !repo.contains(key) {
        return Err(Error::UnknownKey);
    }
    dispatch(key, val.get(), core, tmc, homing).await?;
    let _ = repo.set(key, val); // infallible: key present
    Ok(())
}

/// On dispatch failure, returns the offending key (borrowed from `repo`).
pub async fn apply_all<'a>(
    repo: &'a settings::Repo,
    core: &SharedCore,
    tmc: &SharedTmc,
    homing: &mutex::Mutex<raw::NoopRawMutex, homing::Config>,
) -> Result<(), &'a str> {
    for (key, v) in repo.iter() {
        if dispatch(key, v, core, tmc, homing).await.is_err() {
            return Err(key);
        }
    }
    Ok(())
}
