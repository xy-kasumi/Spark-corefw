#![no_std]
#![no_main]

use core::fmt::Write as _;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Timer};
use heapless::String;
use panic_halt as _;

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
    let (tx, mut rx) = uart.split();
    let tx: Mutex<NoopRawMutex, _> = Mutex::new(tx);

    let writer = async {
        let mut n: u32 = 0;
        loop {
            let mut line: String<32> = String::new();
            let _ = write!(&mut line, "hello {}\r\n", n);
            let _ = tx.lock().await.write(line.as_bytes()).await;
            n = n.wrapping_add(1);
            Timer::after(Duration::from_secs(1)).await;
        }
    };

    let echoer = async {
        let mut buf = [0u8; 16];
        loop {
            match rx.read_until_idle(&mut buf).await {
                Ok(n) if n > 0 => {
                    let _ = tx.lock().await.write(&buf[..n]).await;
                }
                _ => {}
            }
        }
    };

    join(writer, echoer).await;
}
