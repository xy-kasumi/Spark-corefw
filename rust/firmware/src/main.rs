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
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use heapless::String;
use model::motion::Mode;
use panic_halt as _;

use crate::board::init_motors;
use crate::log::TxMutex;
use crate::motion::{Motion, Shared};
use crate::motor::{Calibration, Motors};

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut cfg = UartConfig::default();
    cfg.baudrate = 115200;
    let uart = Uart::new(
        p.USART2, p.PD6, p.PD5, Irqs, p.DMA1_CH0, p.DMA1_CH1, cfg,
    )
    .unwrap();
    let (tx, rx) = uart.split();
    let tx: TxMutex = Mutex::new(tx);

    let mut bm = init_motors(
        p.TIM7,
        p.TIM6,
        (p.PC4,  p.PF13, p.PF12, p.PF14),
        (p.PD11, p.PG0,  p.PG1,  p.PF15),
        (p.PC6,  p.PF11, p.PG3,  p.PG5),
        (p.PC7,  p.PG4,  p.PC1,  p.PA0),
        (p.PF2,  p.PF9,  p.PF10, p.PG2),
        (p.PE4,  p.PC13, p.PF0,  p.PF1),
        (p.PE1,  p.PE2,  p.PE3,  p.PD4),
    );

    // Enable motors 0..=2 (XYZ). Active-low EN. C-axis (m3) and m4..m6 stay off.
    bm.en[0].set_low();
    bm.en[1].set_low();
    bm.en[2].set_low();

    // Phase 4: pull motor-to-axis mapping + calibration from settings.
    // Step is Copy, so this doesn't move out of bm — bm.en pins stay alive in scope.
    let motors = Motors {
        x: bm.step[0],
        y: bm.step[1],
        z: bm.step[2],
        c: bm.step[3],
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
        comm::run(rx, &motion, &tx),
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
