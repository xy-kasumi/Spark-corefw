// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

use core::fmt::Write;
use core::sync::atomic;

use embassy_sync::blocking_mutex::raw;
use embassy_sync::channel;
use model::coords;
use model::gcode;
use model::motion;
use model::pstate;

use model::command::Command;
use model::gcode::MoveSpec;

use crate::board;
use crate::canceler;
use crate::drivers::tmc2209;
use crate::homing;
use crate::outbox;
use crate::pulser;
use crate::settings;
use crate::SharedCore;

pub const CMD_QUEUE_CAP: usize = 64;

pub type CmdQueue = channel::Channel<raw::NoopRawMutex, Command, CMD_QUEUE_CAP>;

/// Each command emits at most one pstate (a response, or an error), sized to
/// a max-shape line plus its trailing LF.
pub const OUTPUT_CAP: usize = pstate::LINE_CAP + 1;

/// Per-command line buffer: `exec` pushes here, `cmd_loop` flushes to the [`outbox::Outbox`].
pub type OutputBuf = outbox::OutputBuf<OUTPUT_CAP>;

/// Set to 1 while the executor is processing a popped command, 0 otherwise.
/// `cmd_queue.len() + OUTSTANDING` gives `?queue`'s "num" field, which the
/// host treats as a "machine idle" indicator (num == 0 ⇒ idle).
pub static OUTSTANDING: atomic::AtomicUsize = atomic::AtomicUsize::new(0);

/// Rapid feed, also used for homing moves.
const RAPID_SPEED_MM_PER_S: f32 = 10.0;
/// Probe feed.
const PROBE_SPEED_MM_PER_S: f32 = 1.0;

/// Hardware fault detected during execution. `cmd_loop` will enter fault
/// state. Cancellation is not an error — it returns `Ok(...)`.
#[derive(Debug)]
pub struct HwFault;

/// Result of one successful `exec`.
pub enum ExecOutcome {
    /// Command is self-contained. Caller of `exec` should [`drain`] after exec. `drain` marks the command completion.
    Done,
    /// Feed was dispatched (expecting chaining).
    /// Caller should check for chain opportunity before calling [`drain`] or declaring command completion.
    FeedDispatched,
}

/// Run one command.
/// Caller must guarantee clean machine state before calling, by using [`drain`].
/// `canceler` is only checked in long-running commands.
pub async fn exec(
    cmd: Command,
    core: &SharedCore,
    tmc: &settings::SharedTmc,
    homing: &mut homing::Config,
    canceler: &canceler::Canceler,
    repo: &mut model::settings::Repo,
    pulser_cfg: &mut pulser::Config,
    out: &mut OutputBuf,
) -> Result<ExecOutcome, HwFault> {
    match cmd {
        Command::Gcode(gcode::Parsed::Rapid(spec)) => {
            let mut c = core.lock().await;
            let here = c.motors.current();
            let target = c.coord.resolve_move(&spec, here);
            c.motion.start_rapid(target, RAPID_SPEED_MM_PER_S);
        }
        Command::Gcode(gcode::Parsed::Feed(spec)) => {
            // Wait until motion can accept this segment (Idle, or EdmMove with
            // a free extension slot); then dispatch in one lock so the mode
            // read and `do_edm` see the same state. This loop yields, so it
            // honors cancellation explicitly.
            let watch = canceler.watch();
            loop {
                let mut c = core.lock().await;
                if watch.cancelled() {
                    return Ok(ExecOutcome::Done);
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
                    c.pulser.request_cut(pulser_cfg);
                }
                c.motion.do_edm(target);
                break;
            }
            return Ok(ExecOutcome::FeedDispatched);
        }
        Command::Gcode(gcode::Parsed::Probe(spec)) => {
            let mut c = core.lock().await;
            let here = c.motors.current();
            let target = c.coord.resolve_move(&spec, here);
            c.pulser.request_probe();
            c.motion.start_probe(target, PROBE_SPEED_MM_PER_S);
        }
        Command::Gcode(gcode::Parsed::Home(target)) => {
            exec_home(target, core, homing, canceler).await;
        }
        Command::Gcode(gcode::Parsed::CalibrateWork { width, depth }) => {
            exec_calibrate_work(width, depth, core, canceler).await;
        }
        Command::Gcode(gcode::Parsed::SelectCoordSys(a)) => {
            core.lock().await.coord.select(a);
        }
        Command::Gcode(gcode::Parsed::PumpOn) => {
            core.lock().await.pump.set_enable(true);
        }
        Command::Gcode(gcode::Parsed::PumpOff) => {
            core.lock().await.pump.set_enable(false);
        }
        Command::Gcode(gcode::Parsed::WirefeedStart(rate)) => {
            core.lock().await.wirefeed.start(rate);
        }
        Command::Gcode(gcode::Parsed::WirefeedStop) => {
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
            if let Err(e) = settings::write(repo, &key, val, core, tmc, homing).await {
                match e {
                    settings::Error::UnknownKey => {
                        out.push_error(format_args!("unknown key {}", key.as_str()));
                    }
                    settings::Error::ApplyFailed => {
                        out.push_error(format_args!(
                            "failed to set {} {}",
                            key.as_str(),
                            val.get()
                        ));
                    }
                }
            }
        }
        Command::Get => {
            dump_settings(out, repo);
        }
        Command::Stat => {
            dump_stat(out, core, tmc).await;
        }
    }
    Ok(ExecOutcome::Done)
}

/// Universal post-command settle: drain motion to idle, ensure the pulser is
/// deenergized, and wait pump/wirefeed countdowns. Called by `cmd_loop` after
/// every [`ExecOutcome::Done`]; also called after a Feed chain ends naturally.
/// Cheap when the machine is already clean — each `settled()` returns
/// immediately. Pulser fault during the wait surfaces as [`HwFault`].
pub async fn drain(core: &SharedCore) -> Result<(), HwFault> {
    while core.lock().await.motion.mode() != motion::Mode::Idle {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
    }
    core.lock().await.pulser.request_deenergize();
    loop {
        {
            let c = core.lock().await;
            if c.pulser.fault() {
                return Err(HwFault);
            }
            if c.pulser.settled() && c.pump.settled() && c.wirefeed.settled() {
                return Ok(());
            }
        }
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
    }
}

/// Home the target axis, or all axes in phase order, by slamming `side*travel`
/// into the hard stop, then re-anchoring the axis to its configured origin. Stall
/// sensing is dead on this board, so the move always stops at target.
async fn exec_home(
    target: gcode::HomeSpec,
    core: &SharedCore,
    homing: &homing::Config,
    canceler: &canceler::Canceler,
) {
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

async fn exec_calibrate_work(
    width: f32,
    depth: f32,
    core: &SharedCore,
    canceler: &canceler::Canceler,
) {
    let watch = canceler.watch();
    // Switch to work coords and drop any prior calibration, then read the start
    // Z (in work coords) for the probe/retract heights.
    let z_safe = {
        let mut c = core.lock().await;
        c.coord.select(coords::CoordSys::Work);
        c.coord.clear_work_y_calibration();
        let here = c.motors.current();
        c.coord.to_active(here).z
    };
    let z_probe = z_safe - depth;

    // Single-side probe, returns machine Y of the contact.
    #[rustfmt::skip]
    async fn probe_single(
        core: &SharedCore,
        y: f32,
        z_safe: f32,
        z_probe: f32,
        canc: &canceler::Canceler,
    ) -> f32 {
        move_to(core, MoveSpec {y: Some(y), z: Some(z_safe), ..Default::default()}, canc).await;
        move_to(core, MoveSpec {z: Some(z_probe), ..Default::default()}, canc).await;
        let p = probe_to(core, MoveSpec { y: Some(0.0), ..Default::default()},canc).await;
        move_to(core, MoveSpec {y: Some(y), ..Default::default()},canc).await;
        move_to(core, MoveSpec {z: Some(z_safe),..Default::default()}, canc).await;
        p.y
    }

    // exec left & right probe.
    let py_left = probe_single(core, width * 0.5, z_safe, z_probe, canceler).await;
    let py_right = probe_single(core, -width * 0.5, z_safe, z_probe, canceler).await;
    // return to center
    move_to(
        core,
        gcode::MoveSpec {
            y: Some(0.0),
            z: Some(z_safe),
            ..Default::default()
        },
        canceler,
    )
    .await;

    // A cancel mid-probe leaves the contact readings bogus; don't calibrate from them.
    if watch.cancelled() {
        return;
    }
    let work_center_machine_y = (py_left + py_right) * 0.5;
    core.lock()
        .await
        .coord
        .calibrate_work_y(work_center_machine_y);
}

/// Moves to dst and wait until completion.
async fn move_to(core: &SharedCore, dst: gcode::MoveSpec, canceler: &canceler::Canceler) {
    let watch = canceler.watch();
    {
        let mut c = core.lock().await;
        let here = c.motors.current();
        let target = c.coord.resolve_move(&dst, here);
        c.motion.start_rapid(target, RAPID_SPEED_MM_PER_S);
    }
    wait_until_idle(core, &watch).await;
}

/// Probes towards dst and wait until completion.
/// Returns final pos in machine coord.
async fn probe_to(
    core: &SharedCore,
    dst: gcode::MoveSpec,
    canceler: &canceler::Canceler,
) -> coords::PosPhys {
    let watch = canceler.watch();
    {
        let mut c = core.lock().await;
        let here = c.motors.current();
        let target = c.coord.resolve_move(&dst, here);
        c.pulser.request_probe();
        c.motion.start_probe(target, PROBE_SPEED_MM_PER_S);
    }
    wait_until_idle(core, &watch).await;
    // De-energize and wait for the pulser to settle before returning, so the
    // caller's next rapid never moves while energized. A pulser fault also ends
    // the wait — the trailing drain will surface it.
    core.lock().await.pulser.request_deenergize();
    loop {
        let c = core.lock().await;
        if c.pulser.settled() || c.pulser.fault() {
            return c.motors.current();
        }
        drop(c);
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
    }
}

/// Block until motion settles to idle, or a cancel forces it there (the tick
/// loop's safe-state sweep cancels motion on its own once a cancel lands).
async fn wait_until_idle(core: &SharedCore, watch: &canceler::Watcher<'_>) {
    while core.lock().await.motion.mode() != motion::Mode::Idle && !watch.cancelled() {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
    }
}

/// Push the `stg` p-state listing every setting.
fn dump_settings(out: &mut OutputBuf, repo: &model::settings::Repo) {
    let mut line = pstate::Line::new(pstate::PsType::Settings);
    for (key, value) in repo.iter() {
        line = line.float(key, value);
    }
    out.push(line);
}

/// Push the `stat` p-state for debugging.
/// Slow — several hundred ms, dominated by the TMC register dump.
async fn dump_stat(out: &mut OutputBuf, core: &SharedCore, tmc: &settings::SharedTmc) {
    let mut line = pstate::Line::new(pstate::PsType::Stat);

    let (mode, steps, calib_work_y) = {
        let c = core.lock().await;
        (
            c.motion.mode(),
            c.motors.step_counts(),
            c.coord.work_offset_y(),
        )
    };
    let mode_name = match mode {
        motion::Mode::Idle => "idle",
        motion::Mode::Rapid => "rapid",
        motion::Mode::EdmMove => "edm",
        motion::Mode::Probing => "probe",
    };
    line = line
        .str_val("motion.mode", mode_name)
        .float("calib.work.y", calib_work_y);
    for (i, &steps_i) in steps.iter().enumerate() {
        let mut key: heapless::String<32> = heapless::String::new();
        let _ = write!(&mut key, "motor.m{}.current_steps", i);
        line = line.int(&key, steps_i);
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
                line = match t[i].read_reg(*addr).await {
                    Ok(v) => line.hex32(&key, v),
                    Err(_) => line.str_val(&key, "error"),
                };
            }
        }
    }

    let stat = core.lock().await.pulser.read_stat();
    line = line
        .bool("pulser.fault", stat.fault)
        .bool("pulser.energized", stat.energized)
        .int("pulser.i2c_write", stat.i2c_write as i32)
        .int("pulser.i2c_write_fail", stat.i2c_write_fail as i32)
        .int("pulser.i2c_read", stat.i2c_read as i32)
        .int("pulser.i2c_read_fail", stat.i2c_read_fail as i32);

    let (feeding, pos, rate) = {
        let c = core.lock().await;
        (c.wirefeed.feeding(), c.wirefeed.pos_mm(), c.wirefeed.rate())
    };
    line = line
        .bool("wirefeed.feeding", feeding)
        .float("wirefeed.pos", pos)
        .float("wirefeed.rate", rate);

    let max_dt_us = crate::TICK_MAX_DT_US.load(atomic::Ordering::Relaxed);
    let slow_count = crate::TICK_SLOW_COUNT.load(atomic::Ordering::Relaxed);
    line = line
        .int("tick.max_dt_us", max_dt_us as i32)
        .int("tick.slow_count", slow_count as i32);

    out.push(line);
}
