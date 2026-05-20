//! Host command pipeline: queue plumbing and command executor. The `Command`
//! enum + parser live in `model::command`; this module re-exports `Command`
//! for the executor's callers.

use core::fmt::Write;
use core::sync::atomic::AtomicUsize;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use heapless::String;
use model::coords::PosPhys;
use model::gcode::{Command as GCmd, MoveSpec};
use model::motion::Mode;
use model::pstate::{ErrorLine, Line, PsType};
use model::settings::{self, Settings};

pub use model::command::Command;

use crate::board::{Pulser, MOTOR_NAMES, NUM_MOTORS};
use crate::drivers::tmc2209::{REG_CHOPCONF, REG_GCONF, REG_IOIN, REG_SG_RESULT};
use crate::line_tx::LineTx;
use crate::motion::Motion;
use crate::settings::{apply_one, SharedTmc};

pub const CMD_QUEUE_CAP: usize = 64;

pub type CmdQueue = Channel<NoopRawMutex, Command, CMD_QUEUE_CAP>;

/// Commands popped from [`CmdQueue`] but not yet finished — covers the running
/// command and the one in the executor's peek buffer.
/// `cmd_queue.len() + OUTSTANDING` gives `?queue`'s "num" field.
pub static OUTSTANDING: AtomicUsize = AtomicUsize::new(0);

const RAPID_SPEED_MM_PER_S: f32 = 10.0;

pub async fn exec(
    cmd: Command,
    _peek: Option<&Command>,
    motion: &Mutex<NoopRawMutex, Motion>,
    tmc: &SharedTmc,
    pulser: &Mutex<NoopRawMutex, Pulser>,
    line_tx: &LineTx,
    settings: &mut Settings,
) {
    match cmd {
        Command::Gcode(GCmd::Rapid(spec)) => {
            let mut m = motion.lock().await;
            let target = apply_spec(m.current_position(), &spec);
            m.state().start_rapid(target, RAPID_SPEED_MM_PER_S);
        }
        Command::Gcode(GCmd::Linear(_)) => {
            let line = ErrorLine::new()
                .msg(format_args!("G1 not yet implemented"))
                .finish();
            let _ = line_tx.try_send(line);
        }
        Command::Set(id, v) => {
            // Try-apply-then-commit: cache only updates if hardware accepted the change.
            if apply_one(id, v, motion, tmc).await.is_err() {
                let _ = line_tx.try_send(
                    ErrorLine::new()
                        .msg(format_args!("setting failed"))
                        .finish(),
                );
            } else {
                let _ = id.write(settings, v);
            }
        }
        Command::Get => {
            dump_settings(line_tx, settings).await;
        }
        Command::Stat => {
            dump_stat(line_tx, motion, tmc, pulser).await;
        }
    }
}

/// Emit one logical `stg` p-state framed by `stg <` / `stg >`, one kv line per setting.
async fn dump_settings(line_tx: &LineTx, settings: &Settings) {
    line_tx.send(Line::new(PsType::Settings).begin()).await;
    for id in settings::iter_all() {
        let line = Line::new(PsType::Settings).float(id.path().as_str(), id.read(settings));
        line_tx.send(line).await;
    }
    line_tx.send(Line::new(PsType::Settings).end()).await;
}

/// Emit one `stat` p-state framed by `stat <` / `stat >`, one kv line per per-module field.
///
/// Slow: each TMC register read awaits a UART roundtrip + 10 ms settle, so polling all 7
/// drivers across 4 registers takes several hundred ms.
async fn dump_stat(
    line_tx: &LineTx,
    motion: &Mutex<NoopRawMutex, Motion>,
    tmc: &SharedTmc,
    pulser: &Mutex<NoopRawMutex, Pulser>,
) {
    line_tx.send(Line::new(PsType::Stat).begin()).await;

    let (mode, steps) = {
        let m = motion.lock().await;
        (m.mode(), m.motor_step_counts())
    };
    let mode_name = match mode {
        Mode::Idle => "idle",
        Mode::Rapid => "rapid",
    };
    line_tx
        .send(Line::new(PsType::Stat).str_val("motion.mode", mode_name))
        .await;
    for (i, &steps_i) in steps.iter().enumerate() {
        let mut key: String<32> = String::new();
        let _ = write!(&mut key, "motor.{}.current_steps", MOTOR_NAMES[i]);
        line_tx
            .send(Line::new(PsType::Stat).int(&key, steps_i))
            .await;
    }

    const REGS: &[(&str, u8)] = &[
        ("GCONF", REG_GCONF),
        ("IOIN", REG_IOIN),
        ("SG_RESULT", REG_SG_RESULT),
        ("CHOPCONF", REG_CHOPCONF),
    ];
    {
        let mut t = tmc.lock().await;
        for i in 0..NUM_MOTORS {
            for (name, addr) in REGS {
                let mut key: String<32> = String::new();
                let _ = write!(&mut key, "motor.{}.driver.{}", MOTOR_NAMES[i], name);
                let line = match t[i].read_reg(*addr).await {
                    Ok(v) => Line::new(PsType::Stat).hex32(&key, v),
                    Err(_) => Line::new(PsType::Stat).str_val(&key, "error"),
                };
                line_tx.send(line).await;
            }
        }
    }

    // Snapshot under the lock, then emit: holding the pulser lock across a
    // `line_tx.send().await` could deadlock the tick loop (its sole TX drainer)
    // when the TX queue is full.
    let stat = {
        let mut p = pulser.lock().await;
        p.read_stat().await
    };
    if !stat.init_ok {
        line_tx
            .send(Line::new(PsType::Stat).str_val("pulser.status", "init failed"))
            .await;
    } else {
        line_tx
            .send(Line::new(PsType::Stat).bool("pulser.energized", stat.energized))
            .await;
        line_tx
            .send(Line::new(PsType::Stat).int("pulser.poll_count", stat.poll_count as i32))
            .await;
        line_tx
            .send(Line::new(PsType::Stat).int("pulser.i2c_fail", stat.i2c_fail as i32))
            .await;
        line_tx
            .send(Line::new(PsType::Stat).float("pulser.edm.r_pulse", stat.r_pulse))
            .await;
        line_tx
            .send(Line::new(PsType::Stat).float("pulser.edm.r_short", stat.r_short))
            .await;
        line_tx
            .send(Line::new(PsType::Stat).float("pulser.edm.r_open", stat.r_open))
            .await;
        let temp = match stat.temp_c {
            Some(v) => Line::new(PsType::Stat).int("pulser.temp_c", v as i32),
            None => Line::new(PsType::Stat).str_val("pulser.temp_c", "error"),
        };
        line_tx.send(temp).await;
        send_stat_f32(line_tx, "pulser.pulse_current_a", stat.pulse_current_a).await;
        send_stat_f32(line_tx, "pulser.pulse_dur_us", stat.pulse_dur_us).await;
        send_stat_f32(line_tx, "pulser.max_duty_pct", stat.max_duty_pct).await;
    }

    line_tx.send(Line::new(PsType::Stat).end()).await;
}

/// Send a `stat` float field, or `key:"error"` when the value is absent.
async fn send_stat_f32(line_tx: &LineTx, key: &str, value: Option<f32>) {
    let line = match value {
        Some(v) => Line::new(PsType::Stat).float(key, v),
        None => Line::new(PsType::Stat).str_val(key, "error"),
    };
    line_tx.send(line).await;
}

fn apply_spec(current: PosPhys, s: &MoveSpec) -> PosPhys {
    PosPhys {
        x: s.x.unwrap_or(current.x),
        y: s.y.unwrap_or(current.y),
        z: s.z.unwrap_or(current.z),
        c: s.c.unwrap_or(current.c),
    }
}
