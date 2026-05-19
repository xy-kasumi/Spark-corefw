#![no_std]
#![no_main]

mod board;
mod cmd_loop;
mod command;
mod dispatch;
mod drivers;
mod line_tx;
mod motion;
mod motor;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker};
use model::comm;
use model::pstate::{ErrorLine, Line, PsType};
use panic_halt as _;
use static_cell::StaticCell;

use crate::command::CmdQueue;
use crate::drivers::serial::Serial;
use crate::line_tx::LineTx;
use crate::motion::Motion;
use crate::motor::{MotorAxisConfig, Motors};

// Tick rate of the orchestrator loop. Anything that wants a slower cadence
// counts ticks; nothing else schedules its own timer.
const TICK_HZ: u32 = 1000;
const TICK_DT_S: f32 = 1.0 / TICK_HZ as f32;

type SharedMotion = Mutex<NoopRawMutex, Motion>;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut board = board::init(&spawner, 115200);

    // Enable motors 0..=2 (XYZ). Active-low EN. C-axis (m3) and m4..m6 stay off.
    board.motors.en[0].set_low();
    board.motors.en[1].set_low();
    board.motors.en[2].set_low();

    let motors = Motors {
        x: board.motors.step[0],
        y: board.motors.step[1],
        z: board.motors.step[2],
        c: board.motors.step[3],
        cal: MotorAxisConfig {
            steps_per_mm_x: 400.0,
            steps_per_mm_y: 400.0,
            steps_per_mm_z: 400.0,
            steps_per_turn_c: 6400.0,
        },
    };

    static MOTION_CELL: StaticCell<SharedMotion> = StaticCell::new();
    let motion: &'static SharedMotion = MOTION_CELL.init(Mutex::new(Motion::new(motors)));

    let line_tx = LineTx::init(&spawner, board.console);

    static CMD_QUEUE_CELL: StaticCell<CmdQueue> = StaticCell::new();
    let cmd_queue: &'static CmdQueue = CMD_QUEUE_CELL.init(Channel::new());

    spawner.must_spawn(cmd_loop::run(cmd_queue, motion, line_tx));

    let _ = line_tx.try_send(Line::new(PsType::Init).begin().bool("ok", true).end());

    tick_loop(board.console, cmd_queue, motion, line_tx).await;
}

async fn tick_loop(
    serial: &'static Serial,
    cmd_queue: &'static CmdQueue,
    motion: &'static SharedMotion,
    line_tx: &'static LineTx,
) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut framer = comm::Framer::new();
    let mut chunk = [0u8; 32];

    loop {
        ticker.next().await;

        for &b in serial.rx_get(&mut chunk) {
            if let Some(frame) = framer.feed(b) {
                match frame {
                    comm::Frame::Signal(s) => {
                        dispatch::signal(s, motion, cmd_queue, line_tx).await;
                    }
                    comm::Frame::Command(c) => parse_and_enqueue(c, cmd_queue, line_tx),
                }
            }
        }

        {
            let mut m = motion.lock().await;
            m.tick(TICK_DT_S);
        }
    }
}

fn parse_and_enqueue(bytes: &[u8], cmd_queue: &CmdQueue, line_tx: &LineTx) {
    match command::parse(bytes) {
        Ok(cmd) => {
            if let Err(_dropped) = cmd_queue.try_send(cmd) {
                let _ = line_tx.try_send(
                    ErrorLine::new()
                        .source(bytes)
                        .msg(format_args!("queue full"))
                        .finish(),
                );
            }
        }
        Err(e) => {
            let _ = line_tx.try_send(
                ErrorLine::new()
                    .source(bytes)
                    .msg(format_args!("{:?}", e))
                    .finish(),
            );
        }
    }
}
