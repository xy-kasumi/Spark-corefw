//! UART TX helper. Shares the TX half of USART2 across boot/comm/heartbeat.

use embassy_stm32::mode;
use embassy_stm32::usart::UartTx;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;

pub type Tx = UartTx<'static, mode::Async>;
pub type TxMutex = Mutex<NoopRawMutex, Tx>;

pub async fn log(tx: &TxMutex, msg: &[u8]) {
    let _ = tx.lock().await.write(msg).await;
}

/// Write three slices back-to-back under a single lock; convenient for
/// `prefix + variant + suffix` log lines without heap formatting.
pub async fn log3(tx: &TxMutex, a: &[u8], b: &[u8], c: &[u8]) {
    let mut t = tx.lock().await;
    let _ = t.write(a).await;
    let _ = t.write(b).await;
    let _ = t.write(c).await;
}
