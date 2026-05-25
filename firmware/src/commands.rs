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

/// Set to 1 while the executor is processing a popped command, 0 otherwise.
/// `cmd_queue.len() + OUTSTANDING` gives `?queue`'s "num" field, which the
/// host treats as a "machine idle" indicator (num == 0 ⇒ idle).
pub static OUTSTANDING: atomic::AtomicUsize = atomic::AtomicUsize::new(0);

/// Rapid feed, also used for homing moves.
const RAPID_SPEED_MM_PER_S: f32 = 10.0;
/// Probe feed.
const PROBE_SPEED_MM_PER_S: f32 = 1.0;

#[allow(clippy::too_many_arguments)]
pub async fn exec(
    cmd: Command,
    cmd_queue: &CmdQueue,
    core: &SharedCore,
    tmc: &settings::SharedTmc,
    homing: &mutex::Mutex<raw::NoopRawMutex, homing::Config>,
    line_tx: &line_tx::LineTx,
    canceler: &canceler::Canceler,
    repo: &mut model::settings::Repo,
    pulser_cfg: &mut pulser::Config,
) {
    // Take a cancel snapshot at exec entry. The new wait-then-dispatch shape
    // (most commands wait for motion idle before issuing) means a cancel landing
    // during the wait would otherwise resurrect motion the operator just stopped.
    // Each dispatch site re-checks `watch.cancelled()` after the prerequisite
    // wait — and, for motion-touching commands, under the dispatch lock so
    // there's no yield between check and issue.
    let watch = canceler.watch();
    match cmd {
        Command::Gcode(gcode::Parsed::Rapid(spec)) => {
            settle_for_non_edm(core).await;
            {
                let mut c = core.lock().await;
                if watch.cancelled() {
                    return;
                }
                let here = c.motors.current();
                let target = c.coord.resolve_move(&spec, here);
                c.motion.start_rapid(target, RAPID_SPEED_MM_PER_S);
            }
            wait_until_idle(core).await;
        }
        Command::Gcode(gcode::Parsed::Feed(spec)) => {
            // Wait until motion can accept this segment (Idle, or EdmMove with a
            // free extension slot); then dispatch in one lock so the mode read
            // and `do_edm` see the same state.
            loop {
                let mut c = core.lock().await;
                if watch.cancelled() {
                    return;
                }
                if !c.motion.ready_for_edm() {
                    drop(c);
                    embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
                    continue;
                }
                let starting_fresh = c.motion.mode() == motion::Mode::Idle;
                let here = c.motors.current();
                let target = c.coord.resolve_move(&spec, here);
                if starting_fresh {
                    // Pulser carve-out: I²C writes hold Core across .await.
                    c.pulser.energize(pulser_cfg).await;
                }
                c.motion.do_edm(target);
                break;
            }
            // Settle: keep this command "outstanding" until either the next
            // command is queued (it'll chain or transition the chain) or motion
            // drains to Idle (chain ended naturally — or cancelled). This
            // preserves the `?queue` num==0 ⇔ machine idle contract.
            while cmd_queue.is_empty() && core.lock().await.motion.mode() != motion::Mode::Idle {
                embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
            }
        }
        Command::Gcode(gcode::Parsed::Probe(spec)) => {
            wait_until_idle(core).await;
            {
                let mut c = core.lock().await;
                if watch.cancelled() {
                    return;
                }
                let here = c.motors.current();
                let target = c.coord.resolve_move(&spec, here);
                // Pulser carve-out: I²C writes hold Core across .await.
                c.pulser.energize(pulser_cfg).await;
                c.motion.start_probe(target, PROBE_SPEED_MM_PER_S);
            }
            wait_until_idle(core).await;
            // Pulser carve-out: I²C write holds Core across .await.
            core.lock().await.pulser.deenergize().await;
        }
        Command::Gcode(gcode::Parsed::Home(target)) => {
            settle_for_non_edm(core).await;
            if watch.cancelled() {
                return;
            }
            exec_home(target, core, homing, canceler).await;
        }
        Command::Gcode(gcode::Parsed::SelectCoordSys(a)) => {
            settle_for_non_edm(core).await;
            if watch.cancelled() {
                return;
            }
            core.lock().await.coord.select(a);
        }
        Command::Gcode(gcode::Parsed::PumpOn) => {
            settle_for_non_edm(core).await;
            if watch.cancelled() {
                return;
            }
            core.lock().await.pump.set_enable(true);
            wait_pump_settled(core).await;
        }
        Command::Gcode(gcode::Parsed::PumpOff) => {
            settle_for_non_edm(core).await;
            if watch.cancelled() {
                return;
            }
            core.lock().await.pump.set_enable(false);
            wait_pump_settled(core).await;
        }
        Command::Gcode(gcode::Parsed::WirefeedStart(rate)) => {
            settle_for_non_edm(core).await;
            if watch.cancelled() {
                return;
            }
            core.lock().await.wirefeed.start(rate);
            // Wait 2 s for wire tension to stabilize.
            embassy_time::Timer::after(embassy_time::Duration::from_millis(2000)).await;
        }
        Command::Gcode(gcode::Parsed::WirefeedStop) => {
            settle_for_non_edm(core).await;
            if watch.cancelled() {
                return;
            }
            core.lock().await.wirefeed.stop();
        }
        Command::Gcode(gcode::Parsed::SetPulse(params)) => {
            let d = pulser::Config::default();
            *pulser_cfg = pulser::Config {
                tool_negative: params.tool_negative,
                pulse_us: params.pulse_us.unwrap_or(d.pulse_us),
                current_a: params.current_a.unwrap_or(d.current_a),
                duty_pct: params.duty_pct.unwrap_or(d.duty_pct),
            };
        }
        Command::Set(key, val) => {
            settle_for_non_edm(core).await;
            if watch.cancelled() {
                return;
            }
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
            settle_for_non_edm(core).await;
            if watch.cancelled() {
                return;
            }
            dump_settings(line_tx, repo).await;
        }
        Command::Stat => {
            settle_for_non_edm(core).await;
            if watch.cancelled() {
                return;
            }
            dump_stat(line_tx, core, tmc).await;
        }
    }
}

/// Poll until motion reaches Idle, on the tick cadence.
async fn wait_until_idle(core: &SharedCore) {
    while core.lock().await.motion.mode() != motion::Mode::Idle {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
    }
}

/// Wait for motion idle and ensure the pulser is de-energized — the precondition
/// for any command that is not part of an EDM chain. Deenergize is lazy so
/// commands that don't follow an EDM chain pay no I²C cost.
async fn settle_for_non_edm(core: &SharedCore) {
    wait_until_idle(core).await;
    let mut c = core.lock().await;
    if c.pulser.energized() {
        // Pulser carve-out: I²C write holds Core across .await.
        c.pulser.deenergize().await;
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
    let stat = core.lock().await.pulser.read_stat();
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
                    .int("pulser.i2c_write", stat.i2c_write as i32),
            )
            .await;
        line_tx
            .send(
                pstate::Line::new(pstate::PsType::Stat)
                    .int("pulser.i2c_write_fail", stat.i2c_write_fail as i32),
            )
            .await;
        line_tx
            .send(
                pstate::Line::new(pstate::PsType::Stat)
                    .int("pulser.i2c_read", stat.i2c_read as i32),
            )
            .await;
        line_tx
            .send(
                pstate::Line::new(pstate::PsType::Stat)
                    .int("pulser.i2c_read_fail", stat.i2c_read_fail as i32),
            )
            .await;
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
