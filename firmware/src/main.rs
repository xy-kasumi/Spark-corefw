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
mod motion;
mod motor;
mod panic_diag;
mod pulser;
mod pump;
mod settings;
mod signals;
mod toolsupply;
mod wirefeed;

use core::sync::atomic;

use embassy_futures::join;
use embassy_sync::blocking_mutex::raw;
use embassy_sync::channel;
use embassy_sync::mutex;
use model::comm;
use model::command;
use model::coordstate;
use model::pstate;

use crate::drivers::serial;

/// Orchestrator loop tick rate. Slower-cadence work counts ticks; nothing else schedules its own timer.
const TICK_HZ: u32 = 1000;
const TICK_DT_S: f32 = 1.0 / TICK_HZ as f32;

type SharedMotion = mutex::Mutex<raw::NoopRawMutex, motion::Motion>;
type SharedPulser = mutex::Mutex<raw::NoopRawMutex, board::Pulser>;
type SharedCoord = mutex::Mutex<raw::NoopRawMutex, coordstate::CoordState>;
type SharedPump = mutex::Mutex<raw::NoopRawMutex, board::Pump>;
type SharedWirefeed = mutex::Mutex<raw::NoopRawMutex, wirefeed::Wirefeed>;
type SharedToolSupply = mutex::Mutex<raw::NoopRawMutex, board::ToolSupply>;
type SharedHoming = mutex::Mutex<raw::NoopRawMutex, homing::Config>;

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let board = board::init(&spawner, 115200);
    let step = board.motors.step;

    let motors = motor::Motors {
        x: step[0],
        y: step[1],
        z: step[2],
        c: step[3],
        cal: motor::AxisConfig::default(),
        home_offset: [0; 3],
    };

    // These live in static storage, not as `main`-task locals. Held inline in
    // main's future they make it exceed the embassy task arena (default 4 KiB;
    // Motion alone is ~32 KiB), so spawning main panics ("task arena is full")
    // before any code runs. StaticCell puts them in plain .bss instead.
    static MOTION_CELL: static_cell::StaticCell<SharedMotion> = static_cell::StaticCell::new();
    let motion: &'static SharedMotion =
        MOTION_CELL.init(mutex::Mutex::new(motion::Motion::new(motors)));
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
        WIREFEED_CELL.init(mutex::Mutex::new(wirefeed::Wirefeed::new(step[6])));
    static TOOLSUPPLY_CELL: static_cell::StaticCell<SharedToolSupply> =
        static_cell::StaticCell::new();
    let toolsupply: &'static SharedToolSupply = TOOLSUPPLY_CELL.init(mutex::Mutex::new(
        board::ToolSupply::new(board.toolsupply_pwm),
    ));
    static HOMING_CELL: static_cell::StaticCell<SharedHoming> = static_cell::StaticCell::new();
    let homing: &'static SharedHoming =
        HOMING_CELL.init(mutex::Mutex::new(homing::Config::default()));
    let line_tx = line_tx::LineTx::init();

    // init phase
    let _ = line_tx.try_send(pstate::Line::new(pstate::PsType::Init).begin());
    let pulser_ok = pulser.lock().await.init(line_tx).await;
    toolsupply.lock().await.init();
    let settings_ok = settings::apply_all(
        &model::settings::Repo::defaults(),
        motion,
        tmc,
        coord,
        wirefeed,
        toolsupply,
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
            coord,
            pulser,
            pump,
            wirefeed,
            line_tx,
        ),
        cmd_loop(
            cmd_queue, motion, tmc, coord, pulser, pump, wirefeed, toolsupply, homing, line_tx,
        ),
    )
    .await;
}

/// Drives RX framing/dispatch, line-TX draining, and the motion tick at [`TICK_HZ`].
async fn tick_loop(
    serial: &serial::Device,
    cmd_queue: &commands::CmdQueue,
    motion: &SharedMotion,
    coord: &SharedCoord,
    pulser: &SharedPulser,
    pump: &SharedPump,
    wirefeed: &SharedWirefeed,
    line_tx: &line_tx::LineTx,
) {
    let mut ticker = embassy_time::Ticker::every(embassy_time::Duration::from_millis(1));
    let mut parser = comm::Parser::new();
    let mut tx_state = line_tx::DrainState::new();
    // Tick-published query snapshot; seeded so the first query after init is valid.
    let mut stats = capture_stats(motion, coord, pulser).await;

    loop {
        ticker.next().await;
        canceler::CANCELER.tick();

        let mut chunk = [0u8; 32];
        for &b in serial.rx_get(&mut chunk) {
            interactive::echo(b, parser.line_len(), line_tx.is_idle(&tx_state), serial);
            match parser.feed(b) {
                Some(comm::Parsed::CancelSignal) => {
                    canceler::CANCELER.cancel();
                    // Lock order motion -> coord (matches commands.rs).
                    {
                        let mut m = motion.lock().await;
                        m.cancel();
                    }
                    coord.lock().await.cancel();
                    pulser.lock().await.deenergize().await;
                    pump.lock().await.cancel();
                    wirefeed.lock().await.stop();
                    while cmd_queue.try_receive().is_ok() {}
                }
                Some(comm::Parsed::QuerySignal(q)) => {
                    signals::exec_query(q, &stats, cmd_queue, line_tx);
                }
                // Fast-set: applied immediately like a signal (unqueued), and
                // stays live during the cancel window for the same reason.
                Some(comm::Parsed::FastSet(fs)) => match fs {
                    command::FastSet::PumpEn(on) => pump.lock().await.set_override(on),
                },
                // While the cancel window is open, blackhole incoming commands so a
                // single `!` drains the queue instead of racing host bytes still in
                // flight. Signals stay live so `?` queries and a follow-up `!` work.
                Some(comm::Parsed::Command(c)) if !canceler::CANCELER.active() => {
                    if let Err(_dropped) = cmd_queue.try_send(c) {
                        let _ = line_tx.try_send(
                            pstate::ErrorLine::new()
                                .msg(format_args!("queue full"))
                                .finish(),
                        );
                    }
                }
                Some(comm::Parsed::CommandError(src)) if !canceler::CANCELER.active() => {
                    let _ = line_tx.try_send(
                        pstate::ErrorLine::new()
                            .source(src)
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
        let fb = {
            let mut p = pulser.lock().await;
            p.tick().await;
            motion::PulserFeedback {
                open_rate: p.open_rate(),
                short_rate: p.short_rate(),
                discharge: p.has_discharge(),
            }
        };

        {
            let mut m = motion.lock().await;
            m.tick(TICK_DT_S, fb);
        }

        wirefeed.lock().await.tick();

        stats = capture_stats(motion, coord, pulser).await;
    }
}

/// Pops parsed [`Command`]s from the queue and runs each. Carries a one-slot peek
/// buffer so the executor can see the next command before committing — used to
/// detect G1-chain continuity (`cont_next`).
async fn cmd_loop(
    cmd_queue: &commands::CmdQueue,
    motion: &SharedMotion,
    tmc: &settings::SharedTmc,
    coord: &SharedCoord,
    pulser: &SharedPulser,
    pump: &SharedPump,
    wirefeed: &SharedWirefeed,
    toolsupply: &SharedToolSupply,
    homing: &SharedHoming,
    line_tx: &line_tx::LineTx,
) {
    let mut repo = model::settings::Repo::defaults();
    let mut pulser_cfg = pulser::Config::default();

    let mut peek_buf: Option<commands::Command> = None;
    // Tracks whether the previous command was a G1 with a following G1 (cont_next).
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
        // Chain consecutive G1s: cont_next is set when both this and the peeked
        // command are G1. cont_prev carries the previous iteration's cont_next.
        let cont_next = commands::is_g1(&curr) && peek.as_ref().map_or(false, commands::is_g1);
        // The lookahead is already pulled out of the channel, so a cancel's queue
        // drain (signals::exec) can't reach it. Watch the canceler and drop the
        // held lookahead ourselves if a cancel landed during this command.
        let watch = canceler::CANCELER.watch();
        commands::exec(
            curr,
            last_has_cont,
            cont_next,
            motion,
            tmc,
            pulser,
            coord,
            pump,
            wirefeed,
            toolsupply,
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

/// Snapshot the query-visible state. Locks motion/coord/pulser sequentially
/// (never nested), reading cached getters only, so it carries no lock-order
/// constraint with the executor's motion->coord/pulser order.
async fn capture_stats(
    motion: &mutex::Mutex<raw::NoopRawMutex, motion::Motion>,
    coord: &mutex::Mutex<raw::NoopRawMutex, coordstate::CoordState>,
    pulser: &mutex::Mutex<raw::NoopRawMutex, board::Pulser>,
) -> signals::MachineStats {
    let (pos, edm) = {
        let m = motion.lock().await;
        (m.current_position(), m.edm_state())
    };
    let (active, offset) = {
        let c = coord.lock().await;
        (c.active(), c.offset_of(c.active()))
    };
    let (eff_duty, open_rate, short_rate, temp) = {
        let p = pulser.lock().await;
        (p.eff_duty(), p.open_rate(), p.short_rate(), p.temp())
    };
    signals::MachineStats {
        pos,
        edm,
        active,
        offset,
        eff_duty,
        open_rate,
        short_rate,
        temp,
    }
}
