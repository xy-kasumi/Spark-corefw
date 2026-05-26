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
mod line_tx;
mod motor;
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

/// Orchestrator loop tick rate. Slower-cadence work counts ticks; nothing else schedules its own timer.
const TICK_HZ: u32 = 1000;
const TICK_DT_S: f32 = 1.0 / TICK_HZ as f32;

/// Motor index of wirefeed stepper.
const M_WIREFEED: usize = 6;

/// EDM path-buffer history capacity: 10 mm max retract at 0.005 mm resolution.
pub(crate) const PB_CAPACITY: usize = 2001;

/// All state shared between tick_loop and cmd_loop. One Mutex, no lock order.
///
/// Discipline: do not `.await` while holding the guard — except `pulser.*`
/// methods, which do bounded I²C transactions (~150µs–1ms). Tick jitter
/// during those calls matches the pre-fold behavior.
pub(crate) struct Core {
    pub motors: motor::Motors,
    pub pulser: board::Pulser,
    pub pump: board::Pump,
    pub wirefeed: wirefeed::Wirefeed,

    pub coord: coordstate::CoordState,
    pub motion: motion::MotionState<PB_CAPACITY>,
}

pub(crate) type SharedCore = mutex::Mutex<raw::NoopRawMutex, Core>;
type SharedHoming = mutex::Mutex<raw::NoopRawMutex, homing::Config>;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let board = board::init(&spawner, 115200);
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
    static HOMING_CELL: static_cell::StaticCell<SharedHoming> = static_cell::StaticCell::new();
    let homing: &'static SharedHoming =
        HOMING_CELL.init(mutex::Mutex::new(homing::Config::default()));
    static CANCELER_CELL: static_cell::StaticCell<canceler::Canceler> =
        static_cell::StaticCell::new();
    let canceler: &'static canceler::Canceler = CANCELER_CELL.init(canceler::Canceler::new());
    let line_tx = line_tx::LineTx::init();

    // init phase
    core.lock().await.pulser.init().await;
    let defaults = model::settings::Repo::defaults();
    let settings_result = settings::apply_all(&defaults, core, tmc, homing).await;
    if let Err(key) = settings_result {
        line_tx.try_send_error(format_args!("failed to apply setting {}", key));
    }
    let _ = line_tx.try_send(
        pstate::Line::new(pstate::PsType::Sys)
            .str_val("ev", "boot")
            .end(),
    );
    // Discard any RX buffered before boot (likely stale bytes from previous power cycle).
    let mut drain = [0u8; serial::RX_CAP];
    while !board.serial.rx_get(&mut drain).is_empty() {}

    if core.lock().await.pulser.fault() {
        line_tx.try_send_error(format_args!("fault: pulser"));
        enter_fault(core, cmd_queue, canceler, line_tx).await;
    }
    if settings_result.is_err() {
        line_tx.try_send_error(format_args!("fault: settings"));
        enter_fault(core, cmd_queue, canceler, line_tx).await;
    }

    join::join(
        tick_loop(board.serial, cmd_queue, core, line_tx, canceler),
        cmd_loop(cmd_queue, core, tmc, homing, line_tx, canceler),
    )
    .await;
}

/// Drop hardware to its safe defaults: stop motion, hold position, disable pump
/// and wirefeed, de-energize the pulser, and drain queued commands. Shared by
/// the cancel and fault paths. Holds `Core` only across pulser I²C per the
/// documented carve-out.
async fn soft_stop(core: &SharedCore, cmd_queue: &commands::CmdQueue) {
    {
        let mut c = core.lock().await;
        let here = c.motors.current();
        c.motion.cancel(here);
        c.motors.set_target(here);
        c.coord.cancel();
        c.pump.cancel();
        c.wirefeed.stop();
    }
    // Pulser carve-out: I²C write holds Core across .await. Error here is
    // ignored — we may already be entering fault, and re-firing on a stuck
    // I²C bus is pointless. The idempotent fault latch handles re-entry.
    let _ = core.lock().await.pulser.deenergize().await;
    while cmd_queue.try_receive().is_ok() {}
}

/// Latch the fault state, emit `sys ev:"fault"`, and run the safe-stop teardown.
/// Idempotent: re-entries are no-ops after the first.
pub(crate) async fn enter_fault(
    core: &SharedCore,
    cmd_queue: &commands::CmdQueue,
    canceler: &canceler::Canceler,
    line_tx: &line_tx::LineTx,
) {
    if !canceler.enter_fault() {
        return;
    }
    let _ = line_tx.try_send(
        pstate::Line::new(pstate::PsType::Sys)
            .str_val("ev", "fault")
            .end(),
    );
    soft_stop(core, cmd_queue).await;
}

/// Drives RX framing/dispatch, line-TX draining, and the motion tick at [`TICK_HZ`].
async fn tick_loop(
    serial: &serial::Device,
    cmd_queue: &commands::CmdQueue,
    core: &SharedCore,
    line_tx: &line_tx::LineTx,
    canceler: &canceler::Canceler,
) {
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_millis(1));
    let mut framer = linecomm::Framer::new();
    let mut tx_state = line_tx::DrainState::new();
    let mut edm_ov = EdmOverrides::default();

    loop {
        let stats = capture_stats(core).await;

        ticker.next().await;
        canceler.tick();

        let rx = handle_rx(
            serial,
            &mut framer,
            &tx_state,
            line_tx,
            cmd_queue,
            canceler,
            &stats,
        );

        if rx.cancel_seen && !canceler.faulted() {
            soft_stop(core, cmd_queue).await;
        }
        for fs in &rx.fastsets {
            match fs {
                command::FastKey::PumpEn(on) => core.lock().await.pump.set_override(*on),
                command::FastKey::EdmRetrThresh(v) => edm_ov.retr_thresh = *v,
                command::FastKey::EdmAdvThresh(v) => edm_ov.adv_thresh = *v,
                command::FastKey::EdmRetrSpeed(v) => edm_ov.retr_speed = *v,
                command::FastKey::EdmAdvSpeed(v) => edm_ov.adv_speed = *v,
            }
        }

        // Pulser carve-out: refresh I²C holds Core across .await.
        let input = {
            let mut c = core.lock().await;
            c.pulser.tick().await;
            let r = c.pulser.last_ratio();
            motion::MotionInputs {
                dt: TICK_DT_S,
                open_rate: r.open,
                short_rate: r.short,
                discharge: c.pulser.has_discharge(),
            }
        };

        {
            let mut c = core.lock().await;
            let o = c
                .motion
                .tick(input, edm_ov.apply(motion::DEFAULT_CONTROL_PARAMS));
            c.motors.set_target(o.target);
            if let Some(pos_mm) = c.wirefeed.tick() {
                c.motors.set_motor_target(M_WIREFEED, pos_mm);
            }
            c.pump.tick();
        }

        line_tx.drain(serial, &mut tx_state);
    }
}

/// Serial-side phase of one tick: echo, frame, parse, immediate-dispatch.
/// Touches `serial`, `line_tx`, `cmd_queue`, and `canceler` — never `Core`.
/// Anything that needs `Core` (the cancel block, fset applies) is returned via
/// [`RxBatch`] for the caller's Core-touching pass.
fn handle_rx(
    serial: &serial::Device,
    framer: &mut linecomm::Framer,
    tx_state: &line_tx::DrainState,
    line_tx: &line_tx::LineTx,
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
        interactive::echo(b, framer.line_len(), line_tx.is_idle(tx_state), serial);
        let Some(bytes) = framer.feed(b) else {
            continue;
        };
        match command::parse(bytes) {
            command::Parsed::Cancel if !canceler.faulted() => {
                canceler.cancel();
                batch.cancel_seen = true;
            }
            command::Parsed::Query(q) => {
                signals::exec_query(q, stats, cmd_queue, line_tx);
            }
            command::Parsed::FastSet(fs) if !canceler.active() => {
                let _ = batch.fastsets.push(fs);
            }
            command::Parsed::Command(c) if !canceler.active() => {
                if let Err(_dropped) = cmd_queue.try_send(c) {
                    line_tx.try_send_error(format_args!("queue full"));
                }
            }
            command::Parsed::Error if !canceler.active() => {
                line_tx.try_send_error(format_args!("syntax error"));
            }
            _ => {}
        }
    }
    if batch.cancel_seen {
        batch.fastsets.clear();
    }
    batch
}

/// Max rx bytes to process per tick.
const TICK_RX_BYTES: usize = 32;

/// Max "fset" to process per tick.
/// Must be larger than TICK_RX_BYTES / (min "fset" command size).
const TICK_FS_CAP: usize = 8;

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

/// Pops parsed [`Command`]s from the queue and runs each. Feed-chain continuity
/// is decided inside `exec` by observing motion state (`ready_for_edm`) and the
/// queue depth — no peek buffer or `cont_*` flags are tracked here.
async fn cmd_loop(
    cmd_queue: &commands::CmdQueue,
    core: &SharedCore,
    tmc: &settings::SharedTmc,
    homing: &SharedHoming,
    line_tx: &line_tx::LineTx,
    canceler: &canceler::Canceler,
) {
    let mut repo = model::settings::Repo::defaults();
    let mut pulser_cfg = pulser::Config::default();

    loop {
        // OUTSTANDING is bumped only after a successful pop. Single-threaded executor +
        // `await` as the only yield point means the signal reader can't observe a torn count.
        let curr = cmd_queue.receive().await;
        commands::OUTSTANDING.fetch_add(1, atomic::Ordering::Relaxed);
        let result = commands::exec(
            curr,
            cmd_queue,
            core,
            tmc,
            homing,
            line_tx,
            canceler,
            &mut repo,
            &mut pulser_cfg,
        )
        .await;
        commands::OUTSTANDING.fetch_sub(1, atomic::Ordering::Relaxed);
        if result.is_err() {
            enter_fault(core, cmd_queue, canceler, line_tx).await;
        }
    }
}

/// Snapshot the query-visible state under one lock take, reading cached getters only.
async fn capture_stats(core: &SharedCore) -> signals::MachineStats {
    let c = core.lock().await;
    let active = c.coord.active();
    signals::MachineStats {
        pos: c.motors.current(),
        edm: c.motion.edm_state(),
        active,
        offset: c.coord.offset_of(active),
        smooth_pulse_ratio: c.pulser.smoothed_ratio(),
    }
}
