#![no_std]
#![no_main]

mod board;
mod soft_uart;
mod step_gen;
mod tmc2209;

use core::fmt::Write as _;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_stm32::usart::{Config as UartConfig, Uart, UartRx, UartTx};
use embassy_stm32::{bind_interrupts, mode, peripherals, usart};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Timer};
use heapless::String;
use panic_halt as _;

use crate::board::{init_motors, MOTOR_NAMES, NUM_MOTORS};
use crate::soft_uart::SoftUartHandle;
use crate::step_gen::StepGenHandle;
use crate::tmc2209::{Error as TmcError, Tmc2209};

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

type Tx = UartTx<'static, mode::Async>;
type TxMutex = Mutex<NoopRawMutex, Tx>;
type Tmc = Tmc2209<SoftUartHandle<NUM_MOTORS>>;
type Step = StepGenHandle<NUM_MOTORS>;

static MOTION_ON: AtomicBool = AtomicBool::new(false);

// 400 µsteps/mm × 10 mm.
const MOVE_USTEPS: i32 = 4000;
// Which motor to wiggle. m0 (PF13 step / PC4 uart) is the safest default.
const MOVE_MOTOR: usize = 0;
// One-way travel time. Cycle = 2 × this.
const MOVE_MS: u64 = 1000;

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

    let mut motors = init_motors(
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

    log(&tx, b"\r\n[spark-corefw-rs] step + EMI test (space = toggle motion)\r\n").await;

    for m in motors.tmc.iter_mut() {
        let mut line: String<64> = String::new();
        match setup_motor(m).await {
            Ok(()) => {
                let _ = write!(&mut line, "setup {}: ok\r\n", m.name);
            }
            Err(e) => {
                let _ = write!(&mut line, "setup {}: err {:?}\r\n", m.name, e);
            }
        }
        log(&tx, line.as_bytes()).await;
    }

    // Keep the moving motor energized continuously so back-and-forth is
    // instantaneous (no de-energize gap between direction flips).
    motors.step[MOVE_MOTOR].set_always_energized(true);

    let step_handle = motors.step[MOVE_MOTOR];
    join3(
        rx_task(rx, &tx),
        motion_task(step_handle, &tx),
        verify_task(motors.tmc, &tx),
    )
    .await;
}

async fn log(tx: &TxMutex, s: &[u8]) {
    let _ = tx.lock().await.write(s).await;
}

async fn setup_motor(m: &mut Tmc) -> Result<(), TmcError<soft_uart::Error>> {
    m.init().await?;
    m.set_microstep(32).await?;
    m.set_current(30, 30).await?;
    m.set_tcoolthrs(750_000).await?;
    m.set_stallguard_threshold(2).await?;
    Ok(())
}

async fn rx_task(mut rx: UartRx<'static, mode::Async>, tx: &TxMutex) {
    let mut buf = [0u8; 16];
    loop {
        if let Ok(n) = rx.read_until_idle(&mut buf).await {
            for &b in &buf[..n] {
                if b == b' ' {
                    let new = !MOTION_ON.load(Ordering::Relaxed);
                    MOTION_ON.store(new, Ordering::Relaxed);
                    log(tx, if new { b"[motion ON]\r\n" } else { b"[motion OFF]\r\n" }).await;
                }
            }
        }
    }
}

async fn motion_task(step: Step, tx: &TxMutex) {
    let mut going_up = true;
    loop {
        while !MOTION_ON.load(Ordering::Relaxed) {
            Timer::after(Duration::from_millis(50)).await;
        }
        let from = step.current();
        let to = if going_up { MOVE_USTEPS } else { 0 };
        if !ramp_to(step, from, to, MOVE_MS).await {
            log(tx, b"[motion halted mid-move]\r\n").await;
        }
        going_up = !going_up;
    }
}

// Linearly advance target from `from` to `to` over `duration_ms`. Updates
// target every 1ms so the step_gen ISR sees a continuously moving setpoint.
// Returns false if MOTION_ON went low mid-move (target is halted at current).
async fn ramp_to(step: Step, from: i32, to: i32, duration_ms: u64) -> bool {
    let start = Instant::now();
    let delta = (to - from) as i64;
    let dur = duration_ms as i64;
    loop {
        if !MOTION_ON.load(Ordering::Relaxed) {
            step.set_target(step.current());
            return false;
        }
        let elapsed_ms = start.elapsed().as_millis() as i64;
        if elapsed_ms >= dur {
            break;
        }
        let intermediate = from as i64 + (delta * elapsed_ms / dur);
        step.set_target(intermediate as i32);
        Timer::after(Duration::from_millis(1)).await;
    }
    step.set_target(to);
    true
}

// Periodically re-applies the default-path register setup on each motor.
// Each cycle exercises ~12 UART transactions per motor (with IFCNT verify),
// so transient EMI from a stepping motor surfaces here as failures.
async fn verify_task(mut tmc: [Tmc; NUM_MOTORS], tx: &TxMutex) {
    let mut succ = [0u32; NUM_MOTORS];
    let mut fail = [0u32; NUM_MOTORS];
    let mut iter: u32 = 0;
    loop {
        for (i, m) in tmc.iter_mut().enumerate() {
            let ok = m.set_microstep(32).await.is_ok()
                && m.set_current(30, 30).await.is_ok()
                && m.set_tcoolthrs(750_000).await.is_ok()
                && m.set_stallguard_threshold(2).await.is_ok();
            if ok {
                succ[i] += 1;
            } else {
                fail[i] += 1;
            }
        }
        iter += 1;
        let mut line: String<160> = String::new();
        let _ = write!(&mut line, "[{:>4}]", iter);
        for i in 0..NUM_MOTORS {
            let _ = write!(&mut line, " {}:{}/{}", MOTOR_NAMES[i], succ[i], fail[i]);
        }
        let _ = write!(&mut line, "\r\n");
        log(tx, line.as_bytes()).await;
        Timer::after(Duration::from_millis(20)).await;
    }
}
