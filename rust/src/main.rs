#![no_std]
#![no_main]

mod board;
mod soft_uart;
mod tmc2209;

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_time::{Duration, Timer};
use heapless::String;
use panic_halt as _;

use crate::board::{init_motors, MOTOR_NAMES, NUM_MOTORS};
use crate::soft_uart::SoftUartHandle;
use crate::tmc2209::{Tmc2209, REG_GCONF, REG_IFCNT};

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut cfg = UartConfig::default();
    cfg.baudrate = 115200;

    let uart = Uart::new(
        p.USART2,
        p.PD6,
        p.PD5,
        Irqs,
        p.DMA1_CH0,
        p.DMA1_CH1,
        cfg,
    )
    .unwrap();
    let (mut tx, _rx) = uart.split();

    let mut motors = init_motors(
        p.TIM7, p.PC4, p.PD11, p.PC6, p.PC7, p.PF2, p.PE4, p.PE1,
    );

    let _ = tx.write(b"\r\n[spark-corefw-rs] tmc2209 soft-uart bringup\r\n").await;

    for m in motors.iter_mut() {
        let mut line: String<64> = String::new();
        match m.init().await {
            Ok(()) => {
                let _ = write!(&mut line, "init {}: ok\r\n", m.name);
            }
            Err(e) => {
                let _ = write!(&mut line, "init {}: err {:?}\r\n", m.name, e);
            }
        }
        let _ = tx.write(line.as_bytes()).await;
    }

    let mut successes = [0u32; NUM_MOTORS];
    let mut failures = [0u32; NUM_MOTORS];
    let mut iter: u32 = 0;

    loop {
        for (i, m) in motors.iter_mut().enumerate() {
            if round_trip(m).await {
                successes[i] += 1;
            } else {
                failures[i] += 1;
            }
        }
        iter += 1;
        if iter % 10 == 0 {
            let mut line: String<160> = String::new();
            let _ = write!(&mut line, "[{:>5}]", iter);
            for i in 0..NUM_MOTORS {
                let _ = write!(&mut line, " {}:{}/{}", MOTOR_NAMES[i], successes[i], failures[i]);
            }
            let _ = write!(&mut line, "\r\n");
            let _ = tx.write(line.as_bytes()).await;
        }
        Timer::after(Duration::from_millis(20)).await;
    }
}

async fn round_trip(m: &mut Tmc2209<SoftUartHandle<NUM_MOTORS>>) -> bool {
    if m.read_reg(REG_IFCNT).await.is_err() {
        return false;
    }
    let Ok(gconf) = m.read_reg(REG_GCONF).await else {
        return false;
    };
    let flipped = gconf ^ 1;
    if m.write_reg(REG_GCONF, flipped).await.is_err() {
        return false;
    }
    let Ok(readback) = m.read_reg(REG_GCONF).await else {
        return false;
    };
    if readback != flipped {
        return false;
    }
    m.write_reg(REG_GCONF, gconf).await.is_ok()
}
