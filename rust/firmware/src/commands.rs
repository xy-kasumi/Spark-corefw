//! Host command pipeline: queue plumbing and command executor. The `Command`
//! enum + parser live in `model::command`; this module re-exports `Command`
//! for the executor's callers.

use core::fmt::Write;
use core::sync::atomic::{AtomicUsize, Ordering};

use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use heapless::String;
use model::coordstate::CoordState;
use model::gcode::{Command as GCmd, HomeAxes, PulserConfig};
use model::motion::Mode;
use model::pstate::{ErrorLine, Line, PsType};
use model::settings::{self, Axis, Settings};

pub use model::command::Command;

use crate::board::{MotorStepping, Pulser, MOTOR_NAMES, NUM_MOTORS};
use crate::drivers::tmc2209::{REG_CHOPCONF, REG_GCONF, REG_IOIN, REG_SG_RESULT};
use crate::line_tx::LineTx;
use crate::motion::Motion;
use crate::pump::Pump;
use crate::settings::{apply_one, SharedTmc};
use crate::signals::CANCEL_GEN;
use crate::toolsupply::ToolSupply;
use crate::wirefeed::Wirefeed;

pub const CMD_QUEUE_CAP: usize = 64;

pub type CmdQueue = Channel<NoopRawMutex, Command, CMD_QUEUE_CAP>;

/// Commands popped from [`CmdQueue`] but not yet finished — covers the running
/// command and the one in the executor's peek buffer.
/// `cmd_queue.len() + OUTSTANDING` gives `?queue`'s "num" field.
pub static OUTSTANDING: AtomicUsize = AtomicUsize::new(0);

/// Rapid feed, also used for homing moves (matches C `VELOCITY_MM_PER_S`).
const RAPID_SPEED_MM_PER_S: f32 = 10.0;
/// Probe feed (matches C `PROBE_VELOCITY_MM_PER_S`).
const PROBE_SPEED_MM_PER_S: f32 = 1.0;

/// True if `cmd` is a G1 move — the only command that chains via the path buffer.
pub fn is_g1(cmd: &Command) -> bool {
    matches!(cmd, Command::Gcode(GCmd::Linear(_)))
}

pub async fn exec(
    cmd: Command,
    cont_prev: bool,
    cont_next: bool,
    motion: &Mutex<NoopRawMutex, Motion>,
    tmc: &SharedTmc,
    pulser: &Mutex<NoopRawMutex, Pulser>,
    coord: &Mutex<NoopRawMutex, CoordState>,
    pump: &Mutex<NoopRawMutex, Pump>,
    wirefeed: &Mutex<NoopRawMutex, Wirefeed>,
    toolsupply: &Mutex<NoopRawMutex, ToolSupply>,
    step: &[MotorStepping; NUM_MOTORS],
    line_tx: &LineTx,
    settings: &mut Settings,
    pulser_cfg: &mut PulserConfig,
) {
    match cmd {
        Command::Gcode(GCmd::Rapid(spec)) => {
            {
                // Lock order motion -> coord (matches signals.rs) to avoid deadlock.
                let mut m = motion.lock().await;
                let here = m.current_position();
                let target = coord.lock().await.resolve_move(&spec, here);
                m.state().start_rapid(target, RAPID_SPEED_MM_PER_S);
            }
            // Block until the rapid finishes; otherwise the next queued command
            // would overwrite this still-running move (matches C G0 handler).
            wait_move_end(motion, pulser, cont_next).await;
        }
        Command::Gcode(GCmd::Linear(spec)) => {
            let (target, chaining) = {
                let m = motion.lock().await;
                let here = m.current_position();
                let target = coord.lock().await.resolve_move(&spec, here);
                // Only chain onto a still-running EDM move; a cancel drops it to
                // Idle and breaks the chain even if cont_prev was set.
                (target, cont_prev && m.mode() == Mode::EdmMove)
            };
            if chaining {
                motion.lock().await.state().enqueue_edm(target, cont_next);
            } else {
                pulser
                    .lock()
                    .await
                    .energize(
                        pulser_cfg.tool_negative,
                        pulser_cfg.pulse_us,
                        pulser_cfg.current_a,
                        pulser_cfg.duty_pct,
                    )
                    .await;
                motion.lock().await.state().start_edm(target, cont_next);
            }
            wait_move_end(motion, pulser, cont_next).await;
        }
        Command::Gcode(GCmd::Probe(spec)) => {
            let target = {
                let m = motion.lock().await;
                let here = m.current_position();
                coord.lock().await.resolve_move(&spec, here)
            };
            pulser
                .lock()
                .await
                .energize(
                    pulser_cfg.tool_negative,
                    pulser_cfg.pulse_us,
                    pulser_cfg.current_a,
                    pulser_cfg.duty_pct,
                )
                .await;
            motion
                .lock()
                .await
                .state()
                .start_probe(target, PROBE_SPEED_MM_PER_S);
            wait_move_end(motion, pulser, false).await;
        }
        Command::Gcode(GCmd::Home(axes)) => {
            exec_home(axes, motion, line_tx, settings).await;
        }
        Command::Gcode(GCmd::SelectCoordSys(a)) => {
            coord.lock().await.select(a);
        }
        Command::Gcode(GCmd::Pump(enable)) => {
            pump.lock().await.set_enable(enable).await;
        }
        Command::Gcode(GCmd::WirefeedStart(rate)) => {
            wirefeed.lock().await.start(rate);
            // Wait 2 s for wire tension to stabilize (matches C M10 handler).
            Timer::after(Duration::from_millis(2000)).await;
        }
        Command::Gcode(GCmd::WirefeedStop) => {
            wirefeed.lock().await.stop();
        }
        Command::Gcode(GCmd::ToolSupply(state)) => {
            toolsupply.lock().await.set_state(state).await;
        }
        Command::Gcode(GCmd::Pulser(cfg)) => {
            // Modal: M3/M4 only update the config the next G1/G38.3 energizes with.
            *pulser_cfg = cfg;
        }
        Command::Set(id, v) => {
            // Try-apply-then-commit: cache only updates if hardware accepted the change.
            match apply_one(id, v, motion, tmc, coord, wirefeed, toolsupply, step).await {
                Ok(()) => {
                    let _ = id.write(settings, v);
                }
                Err(e) => {
                    let _ = line_tx.try_send(
                        ErrorLine::new()
                            .msg(format_args!(
                                "setting failed: {}={} ({:?})",
                                id.path().as_str(),
                                v,
                                e
                            ))
                            .finish(),
                    );
                }
            }
        }
        Command::Get => {
            dump_settings(line_tx, settings).await;
        }
        Command::Stat => {
            dump_stat(line_tx, motion, tmc, pulser, wirefeed).await;
        }
    }
}

/// Wait for the current move to finish, polling on the tick cadence while the
/// tick loop concurrently advances motion (mirrors C `wait_move_command_end`).
/// With `cont_next`, return once the path can accept the next chained segment
/// (pulser stays energized); otherwise wait for full stop and de-energize.
async fn wait_move_end(
    motion: &Mutex<NoopRawMutex, Motion>,
    pulser: &Mutex<NoopRawMutex, Pulser>,
    cont_next: bool,
) {
    if cont_next {
        while !motion.lock().await.can_enqueue() {
            Timer::after(Duration::from_millis(1)).await;
        }
    } else {
        while motion.lock().await.mode() != Mode::Idle {
            Timer::after(Duration::from_millis(1)).await;
        }
        pulser.lock().await.deenergize().await;
    }
}

/// G28: home each requested axis (or all, in phase order) by slamming `side*travel`
/// into the hard stop, then re-anchoring the axis to its configured origin. Stall
/// sensing is dead on this board, so the move always stops at target.
async fn exec_home(
    axes: HomeAxes,
    motion: &Mutex<NoopRawMutex, Motion>,
    line_tx: &LineTx,
    settings: &Settings,
) {
    if axes.c {
        let _ = line_tx.try_send(
            ErrorLine::new()
                .msg(format_args!("C homing not supported"))
                .finish(),
        );
        return;
    }
    let count = axes.x as u8 + axes.y as u8 + axes.z as u8;
    if count > 1 {
        let _ = line_tx.try_send(ErrorLine::new().msg(format_args!("too many axes")).finish());
        return;
    }

    // count == 0 means home all in phase order; otherwise the single named axis.
    let mut order = [Axis::X, Axis::Y, Axis::Z];
    order.sort_unstable_by(|a, b| {
        settings.axes[a.idx()]
            .phase
            .total_cmp(&settings.axes[b.idx()].phase)
    });
    for axis in order {
        let named = match axis {
            Axis::X => axes.x,
            Axis::Y => axes.y,
            Axis::Z => axes.z,
        };
        if count == 1 && !named {
            continue;
        }

        let cfg = settings.axes[axis.idx()];
        let gen = CANCEL_GEN.load(Ordering::Relaxed);
        {
            let mut m = motion.lock().await;
            let mut target = m.current_position();
            match axis {
                Axis::X => target.x += cfg.side * cfg.travel,
                Axis::Y => target.y += cfg.side * cfg.travel,
                Axis::Z => target.z += cfg.side * cfg.travel,
            }
            m.state().start_rapid(target, RAPID_SPEED_MM_PER_S);
        }
        while motion.lock().await.mode() != Mode::Idle {
            Timer::after(Duration::from_millis(1)).await;
        }
        if CANCEL_GEN.load(Ordering::Relaxed) != gen {
            break; // cancelled mid-home: do not re-anchor to a bogus origin
        }
        motion.lock().await.finish_home(axis, cfg.origin);
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
    wirefeed: &Mutex<NoopRawMutex, Wirefeed>,
) {
    line_tx.send(Line::new(PsType::Stat).begin()).await;

    let (mode, steps) = {
        let m = motion.lock().await;
        (m.mode(), m.motor_step_counts())
    };
    let mode_name = match mode {
        Mode::Idle => "idle",
        Mode::Rapid => "rapid",
        Mode::EdmMove => "edm",
        Mode::Probing => "probe",
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

    let (feeding, pos, rate) = {
        let w = wirefeed.lock().await;
        (w.feeding(), w.pos_mm(), w.rate())
    };
    line_tx
        .send(Line::new(PsType::Stat).bool("wirefeed.feeding", feeding))
        .await;
    line_tx
        .send(Line::new(PsType::Stat).float("wirefeed.pos", pos))
        .await;
    line_tx
        .send(Line::new(PsType::Stat).float("wirefeed.rate", rate))
        .await;

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
