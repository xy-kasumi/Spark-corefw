// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Apply layer: routes a `SettingId` + value into the right subsystem's configure method.
//! The one place that knows which firmware-side struct each wire path maps onto;
//! subsystems themselves stay settings-agnostic.
//!
//! Variants without a target subsystem are no-ops; the `Settings` cache still records
//! the user's intent so `get` reflects it.

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use model::coordstate::CoordState;
use model::gcode::ToolSupplyState;
use model::pstate::{Line, PsType};
use model::settings::{self, SettingId, Settings};

use crate::board::{MotorConfig, ToolSupply, NUM_MOTORS};
use crate::line_tx::LineTx;
use crate::motion::Motion;
use crate::wirefeed::Wirefeed;

pub type SharedTmc = Mutex<NoopRawMutex, [MotorConfig; NUM_MOTORS]>;

#[derive(Debug)]
pub enum ApplyError {
    /// A TMC bus write failed or its readback didn't match.
    Tmc,
}

/// Apply one (id, value) to the firmware. Caller should commit to its `Settings`
/// cache only on Ok.
pub async fn apply_one(
    id: SettingId,
    value: f32,
    motion: &Mutex<NoopRawMutex, Motion>,
    tmc: &SharedTmc,
    coord: &Mutex<NoopRawMutex, CoordState>,
    wirefeed: &Mutex<NoopRawMutex, Wirefeed>,
    toolsupply: &Mutex<NoopRawMutex, ToolSupply>,
) -> Result<(), ApplyError> {
    match id {
        SettingId::MotorMicrostep(i) => {
            let mut t = tmc.lock().await;
            t[i as usize]
                .set_microstep(value as u32)
                .await
                .map_err(|_| ApplyError::Tmc)?;
        }
        SettingId::MotorCurrent(i) => {
            let v = value as u32;
            let mut t = tmc.lock().await;
            t[i as usize]
                .set_current(v, v)
                .await
                .map_err(|_| ApplyError::Tmc)?;
        }
        SettingId::MotorThresh(i) => {
            // FIXME: negative threshold means disable stall detection; currently no-op.
            if value >= 0.0 {
                let mut t = tmc.lock().await;
                t[i as usize]
                    .set_stallguard_threshold(value as u8)
                    .await
                    .map_err(|_| ApplyError::Tmc)?;
            }
        }
        SettingId::MotorUnitsteps(6) => {
            wirefeed.lock().await.set_unitsteps(value);
        }
        SettingId::MotorUnitsteps(i) => {
            // m0..=m3 feed Motion's per-axis calibration. m4/m5 have no motion target.
            let mut m = motion.lock().await;
            m.set_motor_unitsteps(i, value);
        }
        SettingId::CsPos(cs, axis) => {
            coord.lock().await.set_offset(cs, axis, value);
        }
        SettingId::TsServoOpenMs => {
            toolsupply
                .lock()
                .await
                .configure(ToolSupplyState::Open, value)
                .await;
        }
        SettingId::TsServoCloseMs => {
            toolsupply
                .lock()
                .await
                .configure(ToolSupplyState::Closed, value)
                .await;
        }
        SettingId::AxisHomeOrigin(_)
        | SettingId::AxisHomePhase(_)
        | SettingId::AxisHomeSide(_)
        | SettingId::AxisHomeTravel(_) => {}
    }
    Ok(())
}

/// Apply every setting in `s`, emitting `settings.ok` (and `settings.msg` with
/// the failing path on first failure, after which no further settings are
/// applied) into the caller's open `init` p-state group. The caller owns the
/// group's `begin`/`end`.
pub async fn apply_all(
    s: &Settings,
    motion: &Mutex<NoopRawMutex, Motion>,
    tmc: &SharedTmc,
    coord: &Mutex<NoopRawMutex, CoordState>,
    wirefeed: &Mutex<NoopRawMutex, Wirefeed>,
    toolsupply: &Mutex<NoopRawMutex, ToolSupply>,
    line_tx: &LineTx,
) -> bool {
    for id in settings::iter_all() {
        if apply_one(id, id.read(s), motion, tmc, coord, wirefeed, toolsupply)
            .await
            .is_err()
        {
            let path = id.path();
            let _ = line_tx.try_send(Line::new(PsType::Init).bool("settings.ok", false));
            let _ =
                line_tx.try_send(Line::new(PsType::Init).str_val("settings.msg", path.as_str()));
            return false;
        }
    }
    let _ = line_tx.try_send(Line::new(PsType::Init).bool("settings.ok", true));
    true
}
