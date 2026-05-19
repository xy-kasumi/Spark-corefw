#![no_std]
#![no_main]

mod board;
mod commands;
mod drivers;
mod line_tx;
mod motion;
mod motor;
mod settings;
mod signals;

use core::sync::atomic::Ordering;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker};
use model::comm::{Parsed, Parser};
use model::pstate::ErrorLine;
use model::settings::Settings as SettingsCache;
use panic_halt as _;

use crate::commands::{CmdQueue, Command, OUTSTANDING};
use crate::drivers::serial::Serial;
use crate::line_tx::{DrainState, LineTx};
use crate::motion::Motion;
use crate::motor::{MotorAxisConfig, Motors};
use crate::settings::SharedTmc;

/// Orchestrator loop tick rate. Slower-cadence work counts ticks; nothing else schedules its own timer.
const TICK_HZ: u32 = 1000;
const TICK_DT_S: f32 = 1.0 / TICK_HZ as f32;

type SharedMotion = Mutex<NoopRawMutex, Motion>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut board = board::init(&spawner, 115200);

    // Energize XYZ; C (m3) and m4..m6 stay off.
    board.motors.en[0].set_low();
    board.motors.en[1].set_low();
    board.motors.en[2].set_low();

    // Seed Motion's calibration from defaults so apply_all is the sole writer of these numbers.
    let init_settings = SettingsCache::defaults();
    let motors = Motors {
        x: board.motors.step[0],
        y: board.motors.step[1],
        z: board.motors.step[2],
        c: board.motors.step[3],
        cal: MotorAxisConfig {
            steps_per_mm_x: init_settings.motors[0].unitsteps,
            steps_per_mm_y: init_settings.motors[1].unitsteps,
            steps_per_mm_z: init_settings.motors[2].unitsteps,
            steps_per_turn_c: init_settings.motors[3].unitsteps,
        },
    };

    let motion: SharedMotion = Mutex::new(Motion::new(motors));
    let tmc: SharedTmc = Mutex::new(board.motors.tmc);
    let line_tx = LineTx::init();
    let cmd_queue: CmdQueue = Channel::new();

    // Push defaults to hardware; emits the `init` p-state with the result.
    settings::apply_all(&init_settings, &motion, &tmc, line_tx).await;

    join(
        tick_loop(board.console, &cmd_queue, &motion, line_tx),
        cmd_loop(&cmd_queue, &motion, &tmc, line_tx),
    )
    .await;
}

/// Drives RX framing/dispatch, line-TX draining, and the motion tick at [`TICK_HZ`].
async fn tick_loop(serial: &Serial, cmd_queue: &CmdQueue, motion: &SharedMotion, line_tx: &LineTx) {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut parser = Parser::new();
    let mut tx_state = DrainState::new();

    loop {
        ticker.next().await;

        let mut chunk = [0u8; 32];
        for &b in serial.rx_get(&mut chunk) {
            match parser.feed(b) {
                Some(Parsed::Signal(s)) => {
                    signals::exec(s, motion, cmd_queue, line_tx).await;
                }
                Some(Parsed::Command(c)) => {
                    if let Err(_dropped) = cmd_queue.try_send(c) {
                        let _ = line_tx
                            .try_send(ErrorLine::new().msg(format_args!("queue full")).finish());
                    }
                }
                Some(Parsed::CommandError(src, e)) => {
                    let _ = line_tx.try_send(
                        ErrorLine::new()
                            .source(src)
                            .msg(format_args!("{:?}", e))
                            .finish(),
                    );
                }
                None => {}
            }
        }

        line_tx.drain(serial, &mut tx_state);

        {
            let mut m = motion.lock().await;
            m.tick(TICK_DT_S);
        }
    }
}

/// Pops parsed [`Command`]s from the queue and runs each. Carries a one-slot peek
/// buffer so the executor can see the next command before committing — required
/// for upcoming G1-chain continuity (currently unused).
async fn cmd_loop(cmd_queue: &CmdQueue, motion: &SharedMotion, tmc: &SharedTmc, line_tx: &LineTx) {
    let mut settings = SettingsCache::defaults();

    let mut peek_buf: Option<Command> = None;
    loop {
        // OUTSTANDING is bumped only after a successful pop. Single-threaded executor +
        // `await` as the only yield point means the signal reader can't observe a torn count.
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
        commands::exec(curr, peek.as_ref(), motion, tmc, line_tx, &mut settings).await;
        OUTSTANDING.fetch_sub(1, Ordering::Relaxed);
        peek_buf = peek;
    }
}
