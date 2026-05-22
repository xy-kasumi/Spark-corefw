// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Host command pipeline: queue plumbing and command executor. The `Command`
//! enum + parser live in `model::command`; this module re-exports `Command`
//! for the executor's callers.

use core::fmt::Write;
use core::sync::atomic;

use embassy_sync::blocking_mutex::raw;
use embassy_sync::channel;
use embassy_sync::mutex;
use model::coordstate;
use model::gcode;
use model::pstate;

pub use model::command::Command;

use crate::board;
use crate::canceler;
use crate::drivers::tmc2209;
use crate::homing;
use crate::line_tx;
use crate::motion;
use crate::pulser;
use crate::settings;
use crate::wirefeed;

pub const CMD_QUEUE_CAP: usize = 64;

pub type CmdQueue = channel::Channel<raw::NoopRawMutex, Command, CMD_QUEUE_CAP>;

/// Commands popped from [`CmdQueue`] but not yet finished — covers the running
/// command and the one in the executor's peek buffer.
/// `cmd_queue.len() + OUTSTANDING` gives `?queue`'s "num" field.
pub static OUTSTANDING: atomic::AtomicUsize = atomic::AtomicUsize::new(0);

/// Rapid feed, also used for homing moves (matches C `VELOCITY_MM_PER_S`).
const RAPID_SPEED_MM_PER_S: f32 = 10.0;
/// Probe feed (matches C `PROBE_VELOCITY_MM_PER_S`).
const PROBE_SPEED_MM_PER_S: f32 = 1.0;

/// True if `cmd` is a G1 move — the only command that chains via the path buffer.
pub fn is_g1(cmd: &Command) -> bool {
    matches!(cmd, Command::Gcode(gcode::Parsed::Feed(_)))
}

pub async fn exec(
    cmd: Command,
    cont_prev: bool,
    cont_next: bool,
    motion: &mutex::Mutex<raw::NoopRawMutex, motion::Motion>,
    tmc: &settings::SharedTmc,
    pulser: &mutex::Mutex<raw::NoopRawMutex, board::Pulser>,
    coord: &mutex::Mutex<raw::NoopRawMutex, coordstate::CoordState>,
    pump: &mutex::Mutex<raw::NoopRawMutex, board::Pump>,
    wirefeed: &mutex::Mutex<raw::NoopRawMutex, wirefeed::Wirefeed>,
    homing: &mutex::Mutex<raw::NoopRawMutex, homing::Config>,
    line_tx: &line_tx::LineTx,
    repo: &mut model::settings::Repo,
    pulser_cfg: &mut pulser::Config,
) {
    match cmd {
        Command::Gcode(gcode::Parsed::Rapid(spec)) => {
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
        Command::Gcode(gcode::Parsed::Feed(spec)) => {
            let (target, chaining) = {
                let m = motion.lock().await;
                let here = m.current_position();
                let target = coord.lock().await.resolve_move(&spec, here);
                // Only chain onto a still-running EDM move; a cancel drops it to
                // Idle and breaks the chain even if cont_prev was set.
                (
                    target,
                    cont_prev && m.mode() == model::motion::Mode::EdmMove,
                )
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
        Command::Gcode(gcode::Parsed::Probe(spec)) => {
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
        Command::Gcode(gcode::Parsed::Home(target)) => {
            exec_home(target, motion, homing).await;
        }
        Command::Gcode(gcode::Parsed::SelectCoordSys(a)) => {
            coord.lock().await.select(a);
        }
        Command::Gcode(gcode::Parsed::PumpOn) => {
            pump.lock().await.set_enable(true).await;
        }
        Command::Gcode(gcode::Parsed::PumpOff) => {
            pump.lock().await.set_enable(false).await;
        }
        Command::Gcode(gcode::Parsed::WirefeedStart(rate)) => {
            wirefeed.lock().await.start(rate);
            // Wait 2 s for wire tension to stabilize (matches C M10 handler).
            embassy_time::Timer::after(embassy_time::Duration::from_millis(2000)).await;
        }
        Command::Gcode(gcode::Parsed::WirefeedStop) => {
            wirefeed.lock().await.stop();
        }
        Command::Gcode(gcode::Parsed::Pulser(params)) => {
            // Modal: M3/M4 only update the config the next G1/G38.3 energizes
            // with. Omitted P/Q/R resolve to the spec defaults here, in the
            // executor — the parser stays free of pulser policy.
            let d = pulser::Config::default();
            *pulser_cfg = pulser::Config {
                tool_negative: params.tool_negative,
                pulse_us: params.pulse_us.unwrap_or(d.pulse_us),
                current_a: params.current_a.unwrap_or(d.current_a),
                duty_pct: params.duty_pct.unwrap_or(d.duty_pct),
            };
        }
        Command::Set(key, val) => {
            if let Err(e) =
                settings::write(repo, &key, val, motion, tmc, coord, wirefeed, homing).await
            {
                let line = match e {
                    settings::Error::UnknownKey => pstate::ErrorLine::new()
                        .msg(format_args!("unknown key {}", key.as_str()))
                        .finish(),
                    settings::Error::ApplyFailed => pstate::ErrorLine::new()
                        .msg(format_args!("failed to set {} {}", key.as_str(), val.get()))
                        .finish(),
                };
                let _ = line_tx.try_send(line);
            }
        }
        Command::Get => {
            dump_settings(line_tx, repo).await;
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
    motion: &mutex::Mutex<raw::NoopRawMutex, motion::Motion>,
    pulser: &mutex::Mutex<raw::NoopRawMutex, board::Pulser>,
    cont_next: bool,
) {
    if cont_next {
        while !motion.lock().await.can_enqueue() {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
        }
    } else {
        while motion.lock().await.mode() != model::motion::Mode::Idle {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
        }
        pulser.lock().await.deenergize().await;
    }
}

/// G28: home the target axis, or all axes in phase order, by slamming `side*travel`
/// into the hard stop, then re-anchoring the axis to its configured origin. Stall
/// sensing is dead on this board, so the move always stops at target.
async fn exec_home(
    target: gcode::HomeSpec,
    motion: &mutex::Mutex<raw::NoopRawMutex, motion::Motion>,
    homing: &mutex::Mutex<raw::NoopRawMutex, homing::Config>,
) {
    // Snapshot once; homing params don't change mid-G28.
    let homing = *homing.lock().await;

    let mut order = [
        model::settings::Axis::X,
        model::settings::Axis::Y,
        model::settings::Axis::Z,
    ];
    order.sort_unstable_by(|a, b| homing.axis(*a).phase.total_cmp(&homing.axis(*b).phase));
    for axis in order {
        if let gcode::HomeSpec::One(named) = target {
            if axis != named {
                continue;
            }
        }

        let cfg = homing.axis(axis);
        let watch = canceler::CANCELER.watch();
        {
            let mut m = motion.lock().await;
            let mut target = m.current_position();
            match axis {
                model::settings::Axis::X => target.x += cfg.side * cfg.travel,
                model::settings::Axis::Y => target.y += cfg.side * cfg.travel,
                model::settings::Axis::Z => target.z += cfg.side * cfg.travel,
            }
            m.state().start_rapid(target, RAPID_SPEED_MM_PER_S);
        }
        while motion.lock().await.mode() != model::motion::Mode::Idle {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
        }
        if watch.cancelled() {
            break; // cancelled mid-home: do not re-anchor to a bogus origin
        }
        motion.lock().await.finish_home(axis, cfg.origin);
    }
}

/// Emit one logical `stg` p-state framed by `stg <` / `stg >`, one kv line per setting.
async fn dump_settings(line_tx: &line_tx::LineTx, repo: &model::settings::Repo) {
    line_tx
        .send(pstate::Line::new(pstate::PsType::Settings).begin())
        .await;
    for (key, value) in repo.iter() {
        line_tx
            .send(pstate::Line::new(pstate::PsType::Settings).float(key, value))
            .await;
    }
    line_tx
        .send(pstate::Line::new(pstate::PsType::Settings).end())
        .await;
}

/// Emit one `stat` p-state framed by `stat <` / `stat >`, one kv line per per-module field.
///
/// Slow: each TMC register read awaits a UART roundtrip + 10 ms settle, so polling all 7
/// drivers across 4 registers takes several hundred ms.
async fn dump_stat(
    line_tx: &line_tx::LineTx,
    motion: &mutex::Mutex<raw::NoopRawMutex, motion::Motion>,
    tmc: &settings::SharedTmc,
    pulser: &mutex::Mutex<raw::NoopRawMutex, board::Pulser>,
    wirefeed: &mutex::Mutex<raw::NoopRawMutex, wirefeed::Wirefeed>,
) {
    line_tx
        .send(pstate::Line::new(pstate::PsType::Stat).begin())
        .await;

    let (mode, steps) = {
        let m = motion.lock().await;
        (m.mode(), m.motor_step_counts())
    };
    let mode_name = match mode {
        model::motion::Mode::Idle => "idle",
        model::motion::Mode::Rapid => "rapid",
        model::motion::Mode::EdmMove => "edm",
        model::motion::Mode::Probing => "probe",
    };
    line_tx
        .send(pstate::Line::new(pstate::PsType::Stat).str_val("motion.mode", mode_name))
        .await;
    for (i, &steps_i) in steps.iter().enumerate() {
        let mut key: heapless::String<32> = heapless::String::new();
        let _ = write!(&mut key, "motor.m{}.current_steps", i);
        line_tx
            .send(pstate::Line::new(pstate::PsType::Stat).int(&key, steps_i))
            .await;
    }

    const REGS: &[(&str, u8)] = &[
        ("GCONF", tmc2209::REG_GCONF),
        ("IOIN", tmc2209::REG_IOIN),
        ("SG_RESULT", tmc2209::REG_SG_RESULT),
        ("CHOPCONF", tmc2209::REG_CHOPCONF),
    ];
    {
        let mut t = tmc.lock().await;
        for i in 0..board::NUM_MOTORS {
            for (name, addr) in REGS {
                let mut key: heapless::String<32> = heapless::String::new();
                let _ = write!(&mut key, "motor.m{}.driver.{}", i, name);
                let line = match t[i].read_reg(*addr).await {
                    Ok(v) => pstate::Line::new(pstate::PsType::Stat).hex32(&key, v),
                    Err(_) => pstate::Line::new(pstate::PsType::Stat).str_val(&key, "error"),
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
            .send(pstate::Line::new(pstate::PsType::Stat).str_val("pulser.status", "init failed"))
            .await;
    } else {
        line_tx
            .send(pstate::Line::new(pstate::PsType::Stat).bool("pulser.energized", stat.energized))
            .await;
        line_tx
            .send(
                pstate::Line::new(pstate::PsType::Stat)
                    .int("pulser.poll_count", stat.poll_count as i32),
            )
            .await;
        line_tx
            .send(
                pstate::Line::new(pstate::PsType::Stat)
                    .int("pulser.i2c_fail", stat.i2c_fail as i32),
            )
            .await;
        line_tx
            .send(pstate::Line::new(pstate::PsType::Stat).float("pulser.edm.r_pulse", stat.r_pulse))
            .await;
        line_tx
            .send(pstate::Line::new(pstate::PsType::Stat).float("pulser.edm.r_short", stat.r_short))
            .await;
        line_tx
            .send(pstate::Line::new(pstate::PsType::Stat).float("pulser.edm.r_open", stat.r_open))
            .await;
        let temp = match stat.temp_c {
            Some(v) => pstate::Line::new(pstate::PsType::Stat).int("pulser.temp_c", v as i32),
            None => pstate::Line::new(pstate::PsType::Stat).str_val("pulser.temp_c", "error"),
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
        .send(pstate::Line::new(pstate::PsType::Stat).bool("wirefeed.feeding", feeding))
        .await;
    line_tx
        .send(pstate::Line::new(pstate::PsType::Stat).float("wirefeed.pos", pos))
        .await;
    line_tx
        .send(pstate::Line::new(pstate::PsType::Stat).float("wirefeed.rate", rate))
        .await;

    line_tx
        .send(pstate::Line::new(pstate::PsType::Stat).end())
        .await;
}

/// Send a `stat` float field, or `key:"error"` when the value is absent.
async fn send_stat_f32(line_tx: &line_tx::LineTx, key: &str, value: Option<f32>) {
    let line = match value {
        Some(v) => pstate::Line::new(pstate::PsType::Stat).float(key, v),
        None => pstate::Line::new(pstate::PsType::Stat).str_val(key, "error"),
    };
    line_tx.send(line).await;
}
