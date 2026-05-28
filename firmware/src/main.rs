// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

#![no_std]
#![no_main]

mod board;
mod canceler;
mod commands;
mod drivers;
mod homing;
mod interactive;
mod motor;
mod outbox;
mod panic_diag;
mod pulser;
mod pump;
mod settings;
mod signals;
mod wirefeed;

use core::sync::atomic;

use embassy_futures::join;
use embassy_sync::blocking_mutex::raw;
use embassy_sync::channel;
use embassy_sync::mutex;
use model::command;
use model::coordstate;
use model::linecomm;
use model::motion;
use model::pstate;

use crate::drivers::serial;

/// Serial port baudrate.
const SERIAL_BAUD: u32 = 115200;

/// Main tick loop cadence. (technically configurable, but not at all sure non 1ms works).
const TICK_DT_MS: f32 = 1.0;

/// Max rx bytes to process per tick.
const TICK_RX_BYTES: usize = 32;

/// Max "fset" to process per tick.
/// Must be larger than TICK_RX_BYTES / (min "fset" command size).
const TICK_FS_CAP: usize = 8;

/// Per-tick output-staging buffer (bytes). Sized for ~10 max-shape `?pos`
/// responses, the worst-case single-tick query burst (TICK_RX_BYTES / 3 single-
/// char queries). Overflow drops the over-budget line and latches the buf's
/// overflow flag; signals are queryable so the next tick recovers.
const TICK_OUT_CAP: usize = 2048;

/// Wire-ring capacity (bytes) of the shared outbox.
const OUTBOX_CAP: usize = 4000;

/// Consider 90%+ of 1ms budget as too slow.
pub(crate) const SLOW_TICK_THRESHOLD_US: u32 = 900;

/// Max observed interval between consecutive `tick_loop` wakeups, in microseconds.
/// Exceeding the 1 ms period indicates some sync span on the executor stalled the tick.
pub(crate) static TICK_MAX_DT_US: atomic::AtomicU32 = atomic::AtomicU32::new(0);

/// Count of ticks whose interval exceeded `TICK_LATE_THRESHOLD_US`.
pub(crate) static TICK_SLOW_COUNT: atomic::AtomicU32 = atomic::AtomicU32::new(0);

/// Motor index of wirefeed stepper.
const M_WIREFEED: usize = 6;

/// EDM path-buffer history capacity: 10 mm max retract at 0.001 mm resolution.
pub(crate) const PB_CAPACITY: usize = 10001;

/// All state shared between tick_loop and cmd_loop. One Mutex, no lock order.
///
/// Discipline: do not `.await` while holding the guard — except `pulser.tick`,
/// which does bounded I²C transactions (~150µs–1ms) and is the sole place that
/// hardware-converges the pulser toward its requested state.
pub(crate) struct Core {
    pub motors: motor::Motors,
    pub pulser: board::Pulser,
    pub pump: board::Pump,
    pub wirefeed: wirefeed::Wirefeed,

    pub coord: coordstate::CoordState,
    pub motion: motion::MotionState<PB_CAPACITY>,
}

pub(crate) type SharedCore = mutex::Mutex<raw::NoopRawMutex, Core>;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let board = board::init(&spawner, SERIAL_BAUD);
    let step = board.motors.step;

    let motors = motor::Motors::new(step);
    let start = motors.current();

    // These live in static storage, not as `main`-task locals. Held inline in
    // main's future they make it exceed the embassy task arena (default 4 KiB;
    // Motion alone is ~32 KiB), so spawning main panics ("task arena is full")
    // before any code runs. StaticCell puts them in plain .bss instead.
    static CORE_CELL: static_cell::StaticCell<SharedCore> = static_cell::StaticCell::new();
    let core: &'static SharedCore = CORE_CELL.init(mutex::Mutex::new(Core {
        motion: motion::MotionState::new(start),
        motors,
        coord: coordstate::CoordState::new(),
        pulser: board.pulser,
        pump: board::Pump::new(board.pump),
        wirefeed: wirefeed::Wirefeed::new(),
    }));
    static TMC_CELL: static_cell::StaticCell<settings::SharedTmc> = static_cell::StaticCell::new();
    let tmc: &'static settings::SharedTmc = TMC_CELL.init(mutex::Mutex::new(board.motors.tmc));
    static CMD_QUEUE_CELL: static_cell::StaticCell<commands::CmdQueue> =
        static_cell::StaticCell::new();
    let cmd_queue: &'static commands::CmdQueue = CMD_QUEUE_CELL.init(channel::Channel::new());
    static CANCELER_CELL: static_cell::StaticCell<canceler::Canceler> =
        static_cell::StaticCell::new();
    let canceler: &'static canceler::Canceler = CANCELER_CELL.init(canceler::Canceler::new());
    static OUTBOX_CELL: static_cell::StaticCell<outbox::Outbox<OUTBOX_CAP>> =
        static_cell::StaticCell::new();
    let outbox: &'static outbox::Outbox<OUTBOX_CAP> = OUTBOX_CELL.init(outbox::Outbox::new());

    // init phase
    core.lock().await.pulser.init().await;
    let defaults = model::settings::Repo::defaults();
    let mut homing = homing::Config::default();
    let settings_result = settings::apply_all(&defaults, core, tmc, &mut homing).await;
    let mut init_out: outbox::OutputBuf<256> = outbox::OutputBuf::new();
    if let Err(key) = settings_result {
        init_out.push_error(format_args!("failed to apply setting {}", key));
    }
    init_out.push(pstate::Line::new(pstate::PsType::Sys).str_val("ev", "boot"));
    outbox.flush(&mut init_out).await;

    // Discard any RX buffered before boot (likely stale bytes from previous power cycle).
    let mut drain = [0u8; serial::RX_CAP];
    while !board.serial.rx_get(&mut drain).is_empty() {}

    if core.lock().await.pulser.fault() {
        init_out.push_error(format_args!("fault: pulser"));
        outbox.flush(&mut init_out).await;
        enter_fault(canceler, outbox).await;
    }
    if settings_result.is_err() {
        init_out.push_error(format_args!("fault: settings"));
        outbox.flush(&mut init_out).await;
        enter_fault(canceler, outbox).await;
    }

    join::join(
        tick_loop(board.serial, cmd_queue, core, outbox, canceler),
        cmd_loop(cmd_queue, core, tmc, homing, outbox, canceler),
    )
    .await;
}

/// Latch the fault state and emit `sys ev:"fault"`. Idempotent: re-entries
/// are no-ops after the first. Hardware safe-state is driven by [`tick_loop`]'s
/// cancel sweep: the sticky fault latch keeps `canceler.active()` true forever,
/// so each subsequent tick re-runs the level-triggered cancel block.
pub(crate) async fn enter_fault(
    canceler: &canceler::Canceler,
    outbox: &outbox::Outbox<OUTBOX_CAP>,
) {
    if !canceler.enter_fault() {
        return;
    }
    let mut out: outbox::OutputBuf<64> = outbox::OutputBuf::new();
    out.push(pstate::Line::new(pstate::PsType::Sys).str_val("ev", "fault"));
    outbox.flush(&mut out).await;
}

/// Drives RX framing/dispatch, line-TX draining, and the motion tick at [`TICK_HZ`].
async fn tick_loop(
    serial: &serial::Device,
    cmd_queue: &commands::CmdQueue,
    core: &SharedCore,
    outbox: &outbox::Outbox<OUTBOX_CAP>,
    canceler: &canceler::Canceler,
) {
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_micros(
        (TICK_DT_MS * 1e3) as u64,
    ));
    let mut framer = linecomm::Framer::new();
    let mut tx_state = outbox::DrainState::new();
    let mut edm_ov = EdmOverrides::default();

    loop {
        ticker.next().await; // spent in ohter places & idling.
        let t_begin = embassy_time::Instant::now();

        // Pulser I/O
        core.lock().await.pulser.tick().await;

        // Sync logic
        let mut out: outbox::OutputBuf<TICK_OUT_CAP> = outbox::OutputBuf::new();
        canceler.tick();
        let tx_idle = outbox.is_idle(&tx_state);
        {
            let mut c = core.lock().await;
            let active = c.coord.active();
            let stats = signals::MachineStats {
                pos: c.motors.current(),
                edm: c.motion.edm_state(),
                active,
                offset: c.coord.offset_of(active),
                smooth_pulse_ratio: c.pulser.smoothed_ratio(),
            };
            let rx = handle_rx(
                serial,
                &mut framer,
                tx_idle,
                &mut out,
                cmd_queue,
                canceler,
                &stats,
            );
            for fs in &rx.fastsets {
                match fs {
                    command::FastKey::PumpEn(on) => c.pump.set_override(*on),
                    command::FastKey::EdmRetrThresh(v) => edm_ov.retr_thresh = *v,
                    command::FastKey::EdmAdvThresh(v) => edm_ov.adv_thresh = *v,
                    command::FastKey::EdmRetrSpeed(v) => edm_ov.retr_speed = *v,
                    command::FastKey::EdmAdvSpeed(v) => edm_ov.adv_speed = *v,
                }
            }
            // Level-triggered safe-state sweep. Subsystems' cancel/stop methods
            // are idempotent, so re-running every tick while the canceler is
            // active is a no-op after the first.
            if canceler.active() {
                let here = c.motors.current();
                c.motion.cancel(here);
                c.coord.cancel();
                c.pump.cancel();
                c.wirefeed.stop();
                c.pulser.request_deenergize();
            }
            let r = c.pulser.last_ratio();
            let input = motion::MotionInputs {
                dt: TICK_DT_MS * 1e-3,
                open_rate: r.open,
                short_rate: r.short,
                discharge: c.pulser.has_discharge(),
            };
            let o = c
                .motion
                .tick(input, edm_ov.apply(motion::DEFAULT_CONTROL_PARAMS));
            c.motors.set_target(o.target);
            if let Some(pos_mm) = c.wirefeed.tick() {
                c.motors.set_motor_target(M_WIREFEED, pos_mm);
            }
            c.pump.tick();
        }

        outbox.flush(&mut out).await;
        outbox.drain(serial, &mut tx_state);

        // update tick stat
        let t_end = embassy_time::Instant::now();
        let dt_us = (t_end - t_begin).as_micros().min(u32::MAX as u64) as u32;
        TICK_MAX_DT_US.fetch_max(dt_us, atomic::Ordering::Relaxed);
        if dt_us > SLOW_TICK_THRESHOLD_US {
            TICK_SLOW_COUNT.fetch_add(1, atomic::Ordering::Relaxed);
        }
    }
}

/// Serial-side phase of one tick: echo, frame, parse, immediate-dispatch.
/// Touches `serial`, `cmd_queue`, `canceler`, and the producer-side `out` buf —
/// never `Core`. Anything that needs `Core` (the cancel block, fset applies)
/// is returned via [`RxBatch`] for the caller's Core-touching pass.
fn handle_rx<const N: usize>(
    serial: &serial::Device,
    framer: &mut linecomm::Framer,
    tx_idle: bool,
    out: &mut outbox::OutputBuf<N>,
    cmd_queue: &commands::CmdQueue,
    canceler: &canceler::Canceler,
    stats: &signals::MachineStats,
) -> RxBatch {
    let mut batch = RxBatch {
        cancel_seen: false,
        fastsets: heapless::Vec::new(),
    };
    let mut chunk = [0u8; TICK_RX_BYTES];
    for &b in serial.rx_get(&mut chunk) {
        interactive::echo(b, framer.line_len(), tx_idle, serial);
        let Some(bytes) = framer.feed(b) else {
            continue;
        };
        match command::parse(bytes) {
            command::Parsed::Cancel if !canceler.faulted() => {
                canceler.cancel();
                batch.cancel_seen = true;
            }
            command::Parsed::Query(q) => {
                signals::exec_query(q, stats, cmd_queue, out);
            }
            command::Parsed::FastSet(fs) if !canceler.active() => {
                let _ = batch.fastsets.push(fs);
            }
            command::Parsed::Command(c) if !canceler.active() => {
                if cmd_queue.try_send(c).is_err() {
                    out.push_error(format_args!("queue full"));
                }
            }
            command::Parsed::Error if !canceler.active() => {
                out.push_error(format_args!("syntax error"));
            }
            _ => {}
        }
    }
    if batch.cancel_seen {
        batch.fastsets.clear();
    }
    batch
}

/// Per-tick output of serial RX parsing.
struct RxBatch {
    cancel_seen: bool,
    fastsets: heapless::Vec<command::FastKey, TICK_FS_CAP>,
}

/// `ov.edm.*` values (for fset command). Corresponds to [`motion::EdmControlParams`].
#[derive(Default)]
struct EdmOverrides {
    retr_thresh: Option<f32>,
    adv_thresh: Option<f32>,
    retr_speed: Option<f32>,
    adv_speed: Option<f32>,
}

impl EdmOverrides {
    fn apply(&self, base: motion::EdmControlParams) -> motion::EdmControlParams {
        motion::EdmControlParams {
            retr_thresh: self.retr_thresh.unwrap_or(base.retr_thresh),
            adv_thresh: self.adv_thresh.unwrap_or(base.adv_thresh),
            retr_speed: self.retr_speed.unwrap_or(base.retr_speed),
            adv_speed: self.adv_speed.unwrap_or(base.adv_speed),
        }
    }
}

/// Keep executing `cmd_queue` forever.
async fn cmd_loop(
    cmd_queue: &commands::CmdQueue,
    core: &SharedCore,
    tmc: &settings::SharedTmc,
    mut homing: homing::Config,
    outbox: &outbox::Outbox<OUTBOX_CAP>,
    canceler: &canceler::Canceler,
) {
    let mut repo = model::settings::Repo::defaults();
    let mut pulser_cfg = pulser::Config::default();

    loop {
        let curr = cmd_queue.receive().await;
        if canceler.active() {
            // discard if in cancel state
            continue;
        }

        commands::OUTSTANDING.fetch_add(1, atomic::Ordering::Relaxed);
        let mut out: commands::OutputBuf = outbox::OutputBuf::new();
        let result = commands::exec(
            curr,
            core,
            tmc,
            &mut homing,
            canceler,
            &mut repo,
            &mut pulser_cfg,
            &mut out,
        )
        .await;
        outbox.flush(&mut out).await;

        let settle = match result {
            Ok(commands::ExecOutcome::Done) => commands::drain(core).await,
            Ok(commands::ExecOutcome::FeedDispatched) => {
                // While motion is ongoing, don't give up chaining.
                while cmd_queue.is_empty() && core.lock().await.motion.mode() != motion::Mode::Idle
                {
                    embassy_time::Timer::after(embassy_time::Duration::from_millis(1)).await;
                }

                if core.lock().await.motion.mode() == motion::Mode::Idle {
                    // Chain was not possible.
                    commands::drain(core).await
                } else {
                    // Chained.
                    Ok(())
                }
            }
            Err(e) => Err(e),
        };
        commands::OUTSTANDING.fetch_sub(1, atomic::Ordering::Relaxed);

        if settle.is_err() {
            enter_fault(canceler, outbox).await;
        }
    }
}
