#![no_std]
#![no_main]

mod board;
mod comm;
mod dispatch;
mod motion;
mod motor;
mod serial;
mod soft_uart;
mod step_gen;
mod tmc2209;

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};
use heapless::String;
use model::motion::Mode;
use panic_halt as _;

use crate::comm::LineBuf;
use crate::motion::Motion;
use crate::motor::{Calibration, Motors};
use crate::serial::Serial;

// Tick rate of the single orchestrator loop. Anything that wants a slower
// cadence counts ticks; nothing else schedules its own timer.
const TICK_HZ: u32 = 1000;
const TICK_DT_S: f32 = 1.0 / TICK_HZ as f32;
const HEARTBEAT_EVERY: u32 = 5 * TICK_HZ;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut board = board::init(&spawner, 115200);

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
    let motion = Motion::new(motors);

    board.console.tx_push(b"[spark-rs] booted\r\n");
    orchestrate(motion, board.console).await;
}

async fn orchestrate(mut motion: Motion, serial: &Serial) -> ! {
    let mut ticker = Ticker::every(Duration::from_millis(1));
    let mut count: u32 = 0;
    let mut line = LineBuf::new();
    let mut chunk = [0u8; 32];

    loop {
        ticker.next().await;
        count = count.wrapping_add(1);

        for &b in serial.rx_get(&mut chunk) {
            comm::handle_byte(b, &mut line, &mut motion, serial);
        }

        motion.tick(TICK_DT_S);

        if count % HEARTBEAT_EVERY == 0 {
            emit_heartbeat(&motion, serial);
        }
    }
}

fn emit_heartbeat(motion: &Motion, serial: &Serial) {
    let pos = motion.current_position();
    let mode_str = match motion.mode() {
        Mode::Idle => "idle",
        Mode::Rapid => "rapid",
    };
    let mut line: String<96> = String::new();
    let _ = write!(
        &mut line,
        "hb {} x={:.3} y={:.3} z={:.3} c={:.4}\r\n",
        mode_str, pos.x, pos.y, pos.z, pos.c,
    );
    serial.tx_push(line.as_bytes());
}
