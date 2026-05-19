//! Host command pipeline: queue plumbing, command executor, and the shared
//! [`Command`] enum (re-exported from `model::command` so the parser stays
//! host-testable).

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

pub use model::command::{parse, Command, ParseError};

use crate::board::{MOTOR_NAMES, NUM_MOTORS};
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
            dump_stat(line_tx, motion, tmc).await;
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
async fn dump_stat(line_tx: &LineTx, motion: &Mutex<NoopRawMutex, Motion>, tmc: &SharedTmc) {
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

    line_tx.send(Line::new(PsType::Stat).end()).await;
}

fn apply_spec(current: PosPhys, s: &MoveSpec) -> PosPhys {
    PosPhys {
        x: s.x.unwrap_or(current.x),
        y: s.y.unwrap_or(current.y),
        z: s.z.unwrap_or(current.z),
        c: s.c.unwrap_or(current.c),
    }
}
