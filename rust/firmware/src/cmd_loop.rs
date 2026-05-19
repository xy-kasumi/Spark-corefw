//! Command execution task. Pops parsed [`Command`] from the queue and runs it,
//! with a one-slot peek buffer so the executor can see the next command before
//! committing the current one (G1-chain continuity will use this in a later
//! phase — wired through but unused for now).

use core::sync::atomic::Ordering;

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use model::coords::PosPhys;
use model::gcode::{Command as GCmd, MoveSpec};
use model::pstate::{ErrorLine, Line, PsType, LINE_CAP};
use model::settings::{self, Settings};

use crate::command::{CmdQueue, Command, OUTSTANDING};
use crate::line_tx::LineTx;
use crate::motion::Motion;
use crate::settings::{apply_one, SharedTmc};

const RAPID_SPEED_MM_PER_S: f32 = 100.0;

#[embassy_executor::task]
pub async fn run(
    cmd_queue: &'static CmdQueue,
    motion: &'static Mutex<NoopRawMutex, Motion>,
    tmc: &'static SharedTmc,
    line_tx: &'static LineTx,
) {
    // Settings live with the only writer for now. When apply lands (and
    // subsystems start reading), this moves to a shared static.
    let mut settings = Settings::defaults();

    let mut peek_buf: Option<Command> = None;
    loop {
        // Track OUTSTANDING for ?queue accounting. Single-thread executor:
        // each `await` is the only yield point, so the +/- pairs don't race
        // against the signal reader as long as we bump *after* a successful pop.
        let curr = match peek_buf.take() {
            Some(c) => c,
            None => {
                let c = cmd_queue.receive().await;
                OUTSTANDING.fetch_add(1, Ordering::Relaxed);
                c
            }
        };
        let peek = match cmd_queue.try_receive() {
            Ok(c) => {
                OUTSTANDING.fetch_add(1, Ordering::Relaxed);
                Some(c)
            }
            Err(_) => None,
        };
        exec(curr, peek.as_ref(), motion, tmc, line_tx, &mut settings).await;
        OUTSTANDING.fetch_sub(1, Ordering::Relaxed);
        peek_buf = peek;
    }
}

async fn exec(
    cmd: Command,
    _peek: Option<&Command>,
    motion: &'static Mutex<NoopRawMutex, Motion>,
    tmc: &'static SharedTmc,
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
            // Try-apply-then-commit: only update the cache if the hardware
            // actually accepted the change. Mirrors C's settings_set.
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
    }
}

/// Stream every (path, value) as one logical `stg` p-state, split across
/// however many lines fit under `LINE_CAP`. First chunk carries `<`, last
/// carries `>`; middle chunks carry just the tag and entries.
async fn dump_settings(line_tx: &LineTx, settings: &Settings) {
    let mut line = Line::new(PsType::Settings).begin();
    for id in settings::iter_all() {
        let path = id.path();
        // Worst-case room: " key:value >". 20 bytes is generous for an f32.
        let need = 1 + path.len() + 1 + 20 + 2;
        if line.as_bytes().len() + need > LINE_CAP {
            line_tx.send(line).await;
            line = Line::new(PsType::Settings);
        }
        line = line.float(path.as_str(), id.read(settings));
    }
    line_tx.send(line.end()).await;
}

fn apply_spec(current: PosPhys, s: &MoveSpec) -> PosPhys {
    PosPhys {
        x: s.x.unwrap_or(current.x),
        y: s.y.unwrap_or(current.y),
        z: s.z.unwrap_or(current.z),
        c: s.c.unwrap_or(current.c),
    }
}
