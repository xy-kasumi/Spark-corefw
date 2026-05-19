#![no_std]
#![no_main]

mod board;
mod comm;
mod dispatch;
mod log;
mod motion;
mod motor;
mod soft_uart;
mod step_gen;
mod tmc2209;

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use heapless::String;
use model::motion::Mode;
use panic_halt as _;

use crate::log::TxMutex;
use crate::motion::{Motion, Shared};
use crate::motor::{Calibration, Motors};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut board = board::init(115200);
    let tx: TxMutex = Mutex::new(board.console_tx);

    // Enable motors 0..=2 (XYZ). Active-low EN. C-axis (m3) and m4..m6 stay off.
    board.motors.en[0].set_low();
    board.motors.en[1].set_low();
    board.motors.en[2].set_low();

    // Phase 4: pull motor-to-axis mapping + calibration from settings.
    // Step is Copy, so indexing board.motors.step[N] doesn't move out of
    // board.motors — the en pins stay alive in scope.
    let motors = Motors {
        x: board.motors.step[0],
        y: board.motors.step[1],
        z: board.motors.step[2],
        c: board.motors.step[3],
        cal: Calibration {
            steps_per_mm_x: 400.0,
            steps_per_mm_y: 400.0,
            steps_per_mm_z: 400.0,
            steps_per_turn_c: 6400.0,
        },
    };

    let motion: Shared = Mutex::new(Motion::new(motors));
    log::log(&tx, b"[spark-rs] booted\r\n").await;

    join3(
        comm::run(board.console_rx, &motion, &tx),
        tick_loop(&motion),
        heartbeat(&motion, &tx),
    )
    .await;
}

async fn tick_loop(motion: &Shared) -> ! {
    let mut last = Instant::now();
    loop {
        Timer::after(Duration::from_millis(1)).await;
        let now = Instant::now();
        let dt = (now - last).as_micros() as f32 / 1_000_000.0;
        last = now;
        let mut m = motion.lock().await;
        m.tick(dt);
    }
}

async fn heartbeat(motion: &Shared, tx: &TxMutex) -> ! {
    loop {
        Timer::after(Duration::from_secs(5)).await;
        let (mode, pos) = {
            let m = motion.lock().await;
            (m.mode(), m.current_position())
        };
        let mode_str = match mode {
            Mode::Idle => "idle",
            Mode::Rapid => "rapid",
        };
        let mut line: String<96> = String::new();
        let _ = write!(
            &mut line,
            "hb {} x={:.3} y={:.3} z={:.3} c={:.4}\r\n",
            mode_str, pos.x, pos.y, pos.z, pos.c,
        );
        log::log(tx, line.as_bytes()).await;
    }
}
