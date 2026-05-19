//! Console serial: symmetric TX/RX byte rings + pumper tasks.
//!
//! Logic code uses only the sync free functions [`tx_push`] and [`rx_get`].
//! The pumper tasks ([`pump_tx`], [`pump_rx`]) do nothing but move bytes
//! between the static pipes and the UART DMA — no logic, no formatting.
//!
//! Pipe sizes are picked relative to the 1 ms tick budget at 115200 baud
//! (~12 B/tick): TX = ~22 ticks (covers a max-line reply + heartbeat + jitter),
//! RX = ~5 ticks (covers tick jitter).

use embassy_stm32::mode;
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;

pub const TX_CAP: usize = 256;
pub const RX_CAP: usize = 64;

static TX_PIPE: Pipe<CriticalSectionRawMutex, TX_CAP> = Pipe::new();
static RX_PIPE: Pipe<CriticalSectionRawMutex, RX_CAP> = Pipe::new();

/// Push bytes into the TX ring. Best-effort.
///
/// FIXME: TX overflow handling is unspecified at the protocol level. When the
/// ring is full, `try_write` writes whatever fits and drops the rest — output
/// gets mangled mid-line, which is the visible "something's wrong" signal.
/// A protocol-conformant host should never trigger this; when we firm up
/// protocol-side rules for TX overflow, distinguish full vs partial and
/// surface a drop count.
pub fn tx_push(bytes: &[u8]) {
    let _ = TX_PIPE.try_write(bytes);
}

/// Drain whatever bytes the RX ring currently holds. Returns the filled
/// prefix of the caller-provided buffer (`&[]` if nothing is available).
/// Non-blocking.
pub fn rx_get<'a>(buf: &'a mut [u8]) -> &'a [u8] {
    match RX_PIPE.try_read(buf) {
        Ok(n) => &buf[..n],
        Err(_) => &[],
    }
}

#[embassy_executor::task]
pub async fn pump_tx(mut tx: UartTx<'static, mode::Async>) {
    let mut buf = [0u8; 32];
    loop {
        let n = TX_PIPE.read(&mut buf).await;
        let _ = tx.write(&buf[..n]).await;
    }
}

#[embassy_executor::task]
pub async fn pump_rx(mut rx: RingBufferedUartRx<'static>) {
    let mut buf = [0u8; 32];
    loop {
        // RX errors (overrun, framing) restart background DMA on next read().
        if let Ok(n) = rx.read(&mut buf).await {
            // Drop overflow silently — same FIXME story as TX.
            let _ = RX_PIPE.try_write(&buf[..n]);
        }
    }
}
