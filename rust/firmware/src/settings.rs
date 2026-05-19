//! Apply layer: routes a `SettingId` + value into the right subsystem's configure method.
//! The one place that knows which firmware-side struct each wire path maps onto;
//! subsystems themselves stay settings-agnostic.
//!
//! Variants without a target subsystem are no-ops; the `Settings` cache still records
//! the user's intent so `get` reflects it.

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use model::pstate::{Line, PsType};
use model::settings::{self, SettingId, Settings};

use crate::board::{MotorConfig, NUM_MOTORS};
use crate::line_tx::LineTx;
use crate::motion::Motion;

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
        SettingId::MotorUnitsteps(i) => {
            // m0..=m3 feed Motion's per-axis calibration. m4..=m6 are ignored (no motion target).
            let mut m = motion.lock().await;
            m.set_motor_unitsteps(i, value);
        }
        SettingId::MotorIdlems(_)
        | SettingId::AxisHomeOrigin(_)
        | SettingId::AxisHomePhase(_)
        | SettingId::AxisHomeSide(_)
        | SettingId::AxisHomeTravel(_)
        | SettingId::CsPos(_, _)
        | SettingId::TsServoCloseMs
        | SettingId::TsServoOpenMs => {}
    }
    Ok(())
}

/// Apply every setting in `s`. Emits a single `init` p-state line:
/// `init < settings.ok:true >` on success, or
/// `init < settings.ok:false settings.msg:"<failing-path>" >` on first
/// failure (subsequent settings are not applied).
pub async fn apply_all(
    s: &Settings,
    motion: &Mutex<NoopRawMutex, Motion>,
    tmc: &SharedTmc,
    line_tx: &LineTx,
) -> bool {
    for id in settings::iter_all() {
        if apply_one(id, id.read(s), motion, tmc).await.is_err() {
            let path = id.path();
            let line = Line::new(PsType::Init)
                .begin()
                .bool("settings.ok", false)
                .str_val("settings.msg", path.as_str())
                .end();
            let _ = line_tx.try_send(line);
            return false;
        }
    }
    let _ = line_tx.try_send(
        Line::new(PsType::Init)
            .begin()
            .bool("settings.ok", true)
            .end(),
    );
    true
}
