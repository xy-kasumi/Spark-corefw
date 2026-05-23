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
use model::gcode;
use model::linecomm;
use model::motion;
use model::pstate;

use crate::drivers::serial;

/// Orchestrator loop tick rate. Slower-cadence work counts ticks; nothing else schedules its own timer.
const TICK_HZ: u32 = 1000;
const TICK_DT_S: f32 = 1.0 / TICK_HZ as f32;

/// EDM path-buffer history capacity: 10 mm max retract at 0.005 mm resolution.
pub(crate) const PB_CAPACITY: usize = 2001;

pub(crate) type SharedMotion = mutex::Mutex<raw::NoopRawMutex, motion::MotionState<PB_CAPACITY>>;
pub(crate) type SharedMotors = mutex::Mutex<raw::NoopRawMutex, motor::Motors>;
type SharedPulser = mutex::Mutex<raw::NoopRawMutex, board::Pulser>;
type SharedCoord = mutex::Mutex<raw::NoopRawMutex, coordstate::CoordState>;
type SharedPump = mutex::Mutex<raw::NoopRawMutex, board::Pump>;
type SharedWirefeed = mutex::Mutex<raw::NoopRawMutex, wirefeed::Wirefeed>;
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
    static MOTORS_CELL: static_cell::StaticCell<SharedMotors> = static_cell::StaticCell::new();
    let motors: &'static SharedMotors = MOTORS_CELL.init(mutex::Mutex::new(motors));
    static MOTION_CELL: static_cell::StaticCell<SharedMotion> = static_cell::StaticCell::new();
    let motion: &'static SharedMotion =
        MOTION_CELL.init(mutex::Mutex::new(motion::MotionState::new(start)));
    static TMC_CELL: static_cell::StaticCell<settings::SharedTmc> = static_cell::StaticCell::new();
    let tmc: &'static settings::SharedTmc = TMC_CELL.init(mutex::Mutex::new(board.motors.tmc));
    static PULSER_CELL: static_cell::StaticCell<SharedPulser> = static_cell::StaticCell::new();
    let pulser: &'static SharedPulser = PULSER_CELL.init(mutex::Mutex::new(board.pulser));
    static CMD_QUEUE_CELL: static_cell::StaticCell<commands::CmdQueue> =
        static_cell::StaticCell::new();
    let cmd_queue: &'static commands::CmdQueue = CMD_QUEUE_CELL.init(channel::Channel::new());
    static COORD_CELL: static_cell::StaticCell<SharedCoord> = static_cell::StaticCell::new();
    let coord: &'static SharedCoord =
        COORD_CELL.init(mutex::Mutex::new(coordstate::CoordState::new()));
    static PUMP_CELL: static_cell::StaticCell<SharedPump> = static_cell::StaticCell::new();
    let pump: &'static SharedPump = PUMP_CELL.init(mutex::Mutex::new(board::Pump::new(board.pump)));
    static WIREFEED_CELL: static_cell::StaticCell<SharedWirefeed> = static_cell::StaticCell::new();
    let wirefeed: &'static SharedWirefeed =
        WIREFEED_CELL.init(mutex::Mutex::new(wirefeed::Wirefeed::new()));
    static HOMING_CELL: static_cell::StaticCell<SharedHoming> = static_cell::StaticCell::new();
    let homing: &'static SharedHoming =
        HOMING_CELL.init(mutex::Mutex::new(homing::Config::default()));
    let line_tx = line_tx::LineTx::init();

    // init phase
    let _ = line_tx.try_send(pstate::Line::new(pstate::PsType::Init).begin());
    let pulser_ok = pulser.lock().await.init(line_tx).await;
    let settings_ok = settings::apply_all(
        &model::settings::Repo::defaults(),
        motors,
        tmc,
        coord,
        homing,
        line_tx,
    )
    .await;
    let _ = line_tx
        .try_send(pstate::Line::new(pstate::PsType::Init).bool("ok", pulser_ok && settings_ok));
    let _ = line_tx.try_send(pstate::Line::new(pstate::PsType::Init).end());

    join::join(
        tick_loop(
            board.serial,
            cmd_queue,
            motion,
            motors,
            coord,
            pulser,
            pump,
            wirefeed,
            line_tx,
        ),
        cmd_loop(
            cmd_queue, motion, motors, tmc, coord, pulser, pump, wirefeed, homing, line_tx,
        ),
    )
    .await;
}

/// Drives RX framing/dispatch, line-TX draining, and the motion tick at [`TICK_HZ`].
#[allow(clippy::too_many_arguments)]
async fn tick_loop(
    serial: &serial::Device,
    cmd_queue: &commands::CmdQueue,
    motion: &SharedMotion,
    motors: &SharedMotors,
    coord: &SharedCoord,
    pulser: &SharedPulser,
    pump: &SharedPump,
    wirefeed: &SharedWirefeed,
    line_tx: &line_tx::LineTx,
) {
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_millis(1));
    let mut framer = linecomm::Framer::new();
    let mut tx_state = line_tx::DrainState::new();
    // Tick-published query snapshot; seeded so the first query after init is valid.
    let mut stats = capture_stats(motion, motors, coord, pulser).await;

    loop {
        ticker.next().await;
        canceler::CANCELER.tick();

        let mut chunk = [0u8; 32];
        for &b in serial.rx_get(&mut chunk) {
            interactive::echo(b, framer.line_len(), line_tx.is_idle(&tx_state), serial);
            let Some(bytes) = framer.feed(b) else {
                continue;
            };
            match command::parse(bytes) {
                command::Parsed::Cancel => {
                    canceler::CANCELER.cancel();
                    let here = motors.lock().await.current();
                    motion.lock().await.cancel(here);
                    motors.lock().await.set_target(here);
                    coord.lock().await.cancel();
                    pulser.lock().await.deenergize().await;
                    pump.lock().await.cancel();
                    wirefeed.lock().await.stop();
                    while cmd_queue.try_receive().is_ok() {}
                }
                command::Parsed::Query(q) => {
                    signals::exec_query(q, &stats, cmd_queue, line_tx);
                }
                command::Parsed::FastSet(fs) => match fs {
                    command::FastKey::PumpEn(on) => pump.lock().await.set_override(on),
                },
                // Only handle commands received outside cancel window.
                command::Parsed::Command(c) if !canceler::CANCELER.active() => {
                    if let Err(_dropped) = cmd_queue.try_send(c) {
                        let _ = line_tx.try_send(
                            pstate::ErrorLine::new()
                                .msg(format_args!("queue full"))
                                .finish(),
                        );
                    }
                }
                command::Parsed::Error if !canceler::CANCELER.active() => {
                    let _ = line_tx.try_send(
                        pstate::ErrorLine::new()
                            .source(bytes)
                            .msg(format_args!("syntax error"))
                            .finish(),
                    );
                }
                _ => {}
            }
        }

        line_tx.drain(serial, &mut tx_state);

        // Refresh pulser feedback first, then feed the snapshot into the motion
        // tick. Locks are taken sequentially (never nested), so no lock-order
        // constraint with the executor's motion->coord/pulser order.
        let input = {
            let mut p = pulser.lock().await;
            p.tick().await;
            let r = p.pulse_ratio();
            motion::MotionInputs {
                dt: TICK_DT_S,
                open_rate: r.open,
                short_rate: r.short,
                discharge: p.has_discharge(),
            }
        };

        let target = motion.lock().await.tick(input).ok().map(|o| o.target);
        if let Some(t) = target {
            motors.lock().await.set_target(t);
        }

        if let Some(pos_mm) = wirefeed.lock().await.tick() {
            motors.lock().await.set_motor_target(6, pos_mm);
        }

        pump.lock().await.tick();

        stats = capture_stats(motion, motors, coord, pulser).await;
    }
}

/// Pops parsed [`Command`]s from the queue and runs each. Carries a one-slot peek
/// buffer so the executor can see the next command before committing — used to
/// detect Feed-chain continuity (`cont_next`).
#[allow(clippy::too_many_arguments)]
async fn cmd_loop(
    cmd_queue: &commands::CmdQueue,
    motion: &SharedMotion,
    motors: &SharedMotors,
    tmc: &settings::SharedTmc,
    coord: &SharedCoord,
    pulser: &SharedPulser,
    pump: &SharedPump,
    wirefeed: &SharedWirefeed,
    homing: &SharedHoming,
    line_tx: &line_tx::LineTx,
) {
    let mut repo = model::settings::Repo::defaults();
    let mut pulser_cfg = pulser::Config::default();

    let mut peek_buf: Option<commands::Command> = None;
    // Tracks whether the previous command was a feed with a following feed (cont_next).
    let mut last_has_cont = false;
    loop {
        // OUTSTANDING is bumped only after a successful pop. Single-threaded executor +
        // `await` as the only yield point means the signal reader can't observe a torn count.
        let curr = match peek_buf.take() {
            Some(c) => c,
            None => {
                let c = cmd_queue.receive().await;
                commands::OUTSTANDING.fetch_add(1, atomic::Ordering::Relaxed);
                c
            }
        };
        let peek = match cmd_queue.try_receive() {
            Ok(c) => {
                commands::OUTSTANDING.fetch_add(1, atomic::Ordering::Relaxed);
                Some(c)
            }
            Err(_) => None,
        };
        // Chain consecutive feeds: cont_next is set when both this and the peeked
        // command are feeds. cont_prev carries the previous iteration's cont_next.
        let cont_next = is_feed(&curr) && peek.as_ref().map_or(false, is_feed);
        // The lookahead is already pulled out of the channel, so a cancel's queue
        // drain (signals::exec) can't reach it. Watch the canceler and drop the
        // held lookahead ourselves if a cancel landed during this command.
        let watch = canceler::CANCELER.watch();
        commands::exec(
            curr,
            last_has_cont,
            cont_next,
            motion,
            motors,
            tmc,
            pulser,
            coord,
            pump,
            wirefeed,
            homing,
            line_tx,
            &mut repo,
            &mut pulser_cfg,
        )
        .await;
        commands::OUTSTANDING.fetch_sub(1, atomic::Ordering::Relaxed);
        if watch.cancelled() {
            if peek.is_some() {
                commands::OUTSTANDING.fetch_sub(1, atomic::Ordering::Relaxed);
            }
            last_has_cont = false;
            peek_buf = None;
        } else {
            last_has_cont = cont_next;
            peek_buf = peek;
        }
    }
}

/// Snapshot the query-visible state. Locks motors/motion/coord/pulser
/// sequentially (never nested), reading cached getters only.
async fn capture_stats(
    motion: &SharedMotion,
    motors: &SharedMotors,
    coord: &SharedCoord,
    pulser: &SharedPulser,
) -> signals::MachineStats {
    let pos = motors.lock().await.current();
    let edm = motion.lock().await.edm_state();
    let (active, offset) = {
        let c = coord.lock().await;
        (c.active(), c.offset_of(c.active()))
    };
    let (eff_duty, ratio) = {
        let p = pulser.lock().await;
        (p.eff_duty(), p.pulse_ratio())
    };
    signals::MachineStats {
        pos,
        edm,
        active,
        offset,
        eff_duty,
        open_rate: ratio.open,
        short_rate: ratio.short,
    }
}

fn is_feed(cmd: &commands::Command) -> bool {
    matches!(cmd, commands::Command::Gcode(gcode::Parsed::Feed(_)))
}
