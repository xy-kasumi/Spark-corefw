//! Console serial: capability-style handle wrapping the TX/RX byte rings.
//!
//! `Serial::init` is called once during boot; it spawns the two pumper tasks
//! that shuttle bytes between the static Pipes and the UART DMA, and hands
//! back a `&'static Serial` capability. Logic code calls `tx_push` / `rx_get`
//! on the handle — no ambient functions, no public statics, no way to drive
//! the console without holding the capability.
//!
//! Pipe sizes are picked relative to the 1 ms tick budget at 115200 baud
//! (~12 B/tick): TX = ~22 ticks (covers a max-line reply + heartbeat + jitter),
//! RX = ~5 ticks (covers tick jitter).

use embassy_executor::Spawner;
use embassy_stm32::mode;
use embassy_stm32::usart::{RingBufferedUartRx, Uart, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use static_cell::StaticCell;

pub const TX_CAP: usize = 256;
pub const RX_CAP: usize = 64;

// Hardware-side DMA ring for RX. Sized for ~5 ticks of bandwidth at 115200
// baud to absorb tick jitter; pump_rx forwards into the software RX_PIPE.
const RX_DMA_CAP: usize = 64;

pub struct Serial {
    tx_pipe: Pipe<CriticalSectionRawMutex, TX_CAP>,
    rx_pipe: Pipe<CriticalSectionRawMutex, RX_CAP>,
}

impl Serial {
    /// Build the console subsystem and spawn its pumper tasks. Panics if
    /// called more than once.
    pub fn init(spawner: &Spawner, uart: Uart<'static, mode::Async>) -> &'static Self {
        let (tx, rx) = uart.split();
        let rx_buf: &'static mut [u8; RX_DMA_CAP] =
            cortex_m::singleton!(: [u8; RX_DMA_CAP] = [0; RX_DMA_CAP]).unwrap();
        let rx_ring = rx.into_ring_buffered(rx_buf);

        static CELL: StaticCell<Serial> = StaticCell::new();
        let me = CELL.init(Serial {
            tx_pipe: Pipe::new(),
            rx_pipe: Pipe::new(),
        });
        spawner.must_spawn(pump_tx(me, tx));
        spawner.must_spawn(pump_rx(me, rx_ring));
        me
    }

    /// Push bytes into the TX ring. Best-effort.
    ///
    /// FIXME: TX overflow handling is unspecified at the protocol level. When the
    /// ring is full, `try_write` writes whatever fits and drops the rest — output
    /// gets mangled mid-line, which is the visible "something's wrong" signal.
    /// A protocol-conformant host should never trigger this; when we firm up
    /// protocol-side rules for TX overflow, distinguish full vs partial and
    /// surface a drop count.
    pub fn tx_push(&self, bytes: &[u8]) {
        let _ = self.tx_pipe.try_write(bytes);
    }

    /// Drain whatever bytes the RX ring currently holds. Returns the filled
    /// prefix of the caller-provided buffer (`&[]` if nothing is available).
    /// Non-blocking.
    pub fn rx_get<'a>(&self, buf: &'a mut [u8]) -> &'a [u8] {
        match self.rx_pipe.try_read(buf) {
            Ok(n) => &buf[..n],
            Err(_) => &[],
        }
    }
}

#[embassy_executor::task]
async fn pump_tx(serial: &'static Serial, mut tx: UartTx<'static, mode::Async>) {
    let mut buf = [0u8; 32];
    loop {
        let n = serial.tx_pipe.read(&mut buf).await;
        let _ = tx.write(&buf[..n]).await;
    }
}

#[embassy_executor::task]
async fn pump_rx(serial: &'static Serial, mut rx: RingBufferedUartRx<'static>) {
    let mut buf = [0u8; 32];
    loop {
        // RX errors (overrun, framing) restart background DMA on next read().
        if let Ok(n) = rx.read(&mut buf).await {
            // Drop overflow silently — same FIXME story as tx_push.
            let _ = serial.rx_pipe.try_write(&buf[..n]);
        }
    }
}
