// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

use core::fmt::Write;
use core::sync::atomic;

use embassy_sync::blocking_mutex::raw;
use embassy_sync::channel;
use embassy_sync::mutex;
use model::coords;
use model::gcode;
use model::motion;
use model::pstate;

use model::command::Command;

use crate::board;
use crate::canceler;
use crate::drivers::tmc2209;
use crate::homing;
use crate::line_tx;
use crate::pulser;
use crate::settings;
use crate::SharedCore;

pub const CMD_QUEUE_CAP: usize = 64;

pub type CmdQueue = channel::Channel<raw::NoopRawMutex, Command, CMD_QUEUE_CAP>;

/// Commands popped from [`CmdQueue`] but not yet finished — covers the running
/// command and the one in the executor's peek buffer.
/// `cmd_queue.len() + OUTSTANDING` gives `?queue`'s "num" field.
pub static OUTSTANDING: atomic::AtomicUsize = atomic::AtomicUsize::new(0);

/// Rapid feed, also used for homing moves.
const RAPID_SPEED_MM_PER_S: f32 = 10.0;
/// Probe feed.
const PROBE_SPEED_MM_PER_S: f32 = 1.0;

#[allow(clippy::too_many_arguments)]
pub async fn exec(
    cmd: Command,
    cont_prev: bool,
    cont_next: bool,
    core: &SharedCore,
    tmc: &settings::SharedTmc,
    homing: &mutex::Mutex<raw::NoopRawMutex, homing::Config>,
    line_tx: &line_tx::LineTx,
    canceler: &canceler::Canceler,
    repo: &mut model::settings::Repo,
    pulser_cfg: &mut pulser::Config,
) {
    match cmd {
        Command::Gcode(gcode::Parsed::Rapid(spec)) => {
            {
                let mut c = core.lock().await;
                let here = c.motors.current();
                let target = c.coord.resolve_move(&spec, here);
                c.motion.start_rapid(target, RAPID_SPEED_MM_PER_S);
            }
            // Block until the rapid finishes; otherwise the next queued command
            // would overwrite this still-running move.
            wait_move_end(core, cont_next).await;
        }
        Command::Gcode(gcode::Parsed::Feed(spec)) => {
            let (target, chaining) = {
                let mut c = core.lock().await;
                let here = c.motors.current();
                let target = c.coord.resolve_move(&spec, here);
                // Only chain onto a still-running EDM move; a cancel drops it to
                // Idle and breaks the chain even if cont_prev was set.
                let chaining = cont_prev && c.motion.mode() == motion::Mode::EdmMove;
                if chaining {
                    c.motion.enqueue_edm(target, cont_next);
                }
                (target, chaining)
            };
            if !chaining {
                // Pulser carve-out: I²C writes hold Core across .await.
                core.lock()
                    .await
                    .pulser
                    .energize(
                        pulser_cfg.tool_negative,
                        pulser_cfg.pulse_us,
                        pulser_cfg.current_a,
                        pulser_cfg.duty_pct,
                    )
                    .await;
                core.lock().await.motion.start_edm(target, cont_next);
            }
            wait_move_end(core, cont_next).await;
        }
        Command::Gcode(gcode::Parsed::Probe(spec)) => {
            let target = {
                let mut c = core.lock().await;
                let here = c.motors.current();
                c.coord.resolve_move(&spec, here)
            };
            // Pulser carve-out: I²C writes hold Core across .await.
            core.lock()
                .await
                .pulser
                .energize(
                    pulser_cfg.tool_negative,
                    pulser_cfg.pulse_us,
                    pulser_cfg.current_a,
                    pulser_cfg.duty_pct,
                )
                .await;
            core.lock()
                .await
                .motion
                .start_probe(target, PROBE_SPEED_MM_PER_S);
            wait_move_end(core, false).await;
        }
        Command::Gcode(gcode::Parsed::Home(target)) => {
            exec_home(target, core, homing, canceler).await;
        }
        Command::Gcode(gcode::Parsed::SelectCoordSys(a)) => {
            core.lock().await.coord.select(a);
        }
        Command::Gcode(gcode::Parsed::PumpOn) => {
            core.lock().await.pump.set_enable(true);
            wait_pump_settled(core).await;
        }
        Command::Gcode(gcode::Parsed::PumpOff) => {
            core.lock().await.pump.set_enable(false);
            wait_pump_settled(core).await;
        }
        Command::Gcode(gcode::Parsed::WirefeedStart(rate)) => {
            core.lock().await.wirefeed.start(rate);
            // Wait 2 s for wire tension to stabilize.
            embassy_time::Timer::after(embassy_time::Duration::from_millis(2000)).await;
        }
        Command::Gcode(gcode::Parsed::WirefeedStop) => {
            core.lock().await.wirefeed.stop();
        }
        Command::Gcode(gcode::Parsed::SetPulse(params)) => {
            // Modal: the pulser command only updates the config the next feed/probe
            // energizes with. Omitted P/Q/R resolve to the spec defaults here, in the
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
            if let Err(e) = settings::write(repo, &key, val, core, tmc, homing).await {
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
            dump_stat(line_tx, core, tmc).await;
        }
    }
}

/// Wait for the current move to finish, polling on the tick cadence while the
/// tick loop concurrently advances motion.
/// With `cont_next`, return once the path can accept the next chained segment
/// (pulser stays energized); otherwise wait for full stop and de-energize.
async fn wait_move_end(core: &SharedCore, cont_next: bool) {
    if cont_next {
        while !core.lock().await.motion.can_enqueue() {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
        }
    } else {
        while core.lock().await.motion.mode() != motion::Mode::Idle {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
        }
        // Pulser carve-out: I²C write holds Core across .await.
        core.lock().await.pulser.deenergize().await;
    }
}

/// Wait for the pump's settle countdown to drain on the tick cadence.
async fn wait_pump_settled(core: &SharedCore) {
    while !core.lock().await.pump.settled() {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
    }
}

/// Home the target axis, or all axes in phase order, by slamming `side*travel`
/// into the hard stop, then re-anchoring the axis to its configured origin. Stall
/// sensing is dead on this board, so the move always stops at target.
async fn exec_home(
    target: gcode::HomeSpec,
    core: &SharedCore,
    homing: &mutex::Mutex<raw::NoopRawMutex, homing::Config>,
    canceler: &canceler::Canceler,
) {
    // Snapshot once; homing params don't change mid-home.
    let homing = *homing.lock().await;

    let mut order = [coords::Axis::X, coords::Axis::Y, coords::Axis::Z];
    order.sort_unstable_by(|a, b| homing.axis(*a).phase.total_cmp(&homing.axis(*b).phase));
    for axis in order {
        if let gcode::HomeSpec::One(named) = target {
            if axis != named {
                continue;
            }
        }

        let cfg = homing.axis(axis);
        let watch = canceler.watch();
        {
            let mut c = core.lock().await;
            let mut target = c.motors.current();
            match axis {
                coords::Axis::X => target.x += cfg.side * cfg.travel,
                coords::Axis::Y => target.y += cfg.side * cfg.travel,
                coords::Axis::Z => target.z += cfg.side * cfg.travel,
            }
            c.motion.start_rapid(target, RAPID_SPEED_MM_PER_S);
        }
        while core.lock().await.motion.mode() != motion::Mode::Idle {
            embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
        }
        if watch.cancelled() {
            break; // cancelled mid-home: do not re-anchor to a bogus origin
        }
        // Re-anchor motors to the configured origin, then sync motion's tracked
        // position to the new physical reading.
        let mut c = core.lock().await;
        c.motors.reanchor(axis, cfg.origin);
        let here = c.motors.current();
        c.motion.set_position(here);
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

/// Emit one big `stat` p-state for debugging.
/// It is slow takes several hundred ms. (esp TMC register dump)
async fn dump_stat(line_tx: &line_tx::LineTx, core: &SharedCore, tmc: &settings::SharedTmc) {
    line_tx
        .send(pstate::Line::new(pstate::PsType::Stat).begin())
        .await;

    let (mode, steps) = {
        let c = core.lock().await;
        (c.motion.mode(), c.motors.step_counts())
    };
    let mode_name = match mode {
        motion::Mode::Idle => "idle",
        motion::Mode::Rapid => "rapid",
        motion::Mode::EdmMove => "edm",
        motion::Mode::Probing => "probe",
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

    // Snapshot under the lock, then emit: holding Core across a `line_tx.send().await`
    // could deadlock the tick loop (its sole TX drainer) when the TX queue is full.
    // Pulser carve-out: I²C reads hold Core across .await.
    let stat = core.lock().await.pulser.read_stat().await;
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
            .send(
                pstate::Line::new(pstate::PsType::Stat).float("pulser.edm.r_good", stat.ratio.good),
            )
            .await;
        line_tx
            .send(
                pstate::Line::new(pstate::PsType::Stat)
                    .float("pulser.edm.r_short", stat.ratio.short),
            )
            .await;
        line_tx
            .send(
                pstate::Line::new(pstate::PsType::Stat).float("pulser.edm.r_open", stat.ratio.open),
            )
            .await;
        send_stat_f32(line_tx, "pulser.pulse_current_a", stat.pulse_current_a).await;
        send_stat_f32(line_tx, "pulser.pulse_dur_us", stat.pulse_dur_us).await;
        send_stat_f32(line_tx, "pulser.max_duty_pct", stat.max_duty_pct).await;
    }

    let (feeding, pos, rate) = {
        let c = core.lock().await;
        (c.wirefeed.feeding(), c.wirefeed.pos_mm(), c.wirefeed.rate())
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
