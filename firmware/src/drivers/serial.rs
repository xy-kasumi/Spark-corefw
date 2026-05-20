// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! UART-based serial bytestring RX/TX.
//!
//! Pipe sizes are picked relative to the 1 ms tick x 115200 baud (~12 B) x "just-in-case" jitter buffer (x5).

use embassy_executor::Spawner;
use embassy_stm32::mode;
use embassy_stm32::usart::{RingBufferedUartRx, Uart, UartTx};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use static_cell::StaticCell;

pub const TX_CAP: usize = 64;
pub const RX_CAP: usize = 64;

const RX_DMA_CAP: usize = 64;

pub struct Serial {
    tx_pipe: Pipe<CriticalSectionRawMutex, TX_CAP>,
    rx_pipe: Pipe<CriticalSectionRawMutex, RX_CAP>,
}

impl Serial {
    /// Setup serial by spawning tasks. Only one init() call in the program allowed.
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

    /// Push bytes into the TX buffer.
    /// Returns the number of bytes actually written (0 if buffer is full).
    pub fn tx_push(&self, bytes: &[u8]) -> usize {
        let mut written = 0;
        while written < bytes.len() {
            match self.tx_pipe.try_write(&bytes[written..]) {
                Ok(n) => written += n,
                Err(_) => break,
            }
        }
        written
    }

    /// Drain bytes from the RX buffer.
    /// Returns the filled prefix of the caller-provided buffer (`&[]` if nothing is available).
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
            // Need to call twice when write is past the ring's wrap point.
            let mut remaining = &buf[..n];
            while !remaining.is_empty() {
                match serial.rx_pipe.try_write(remaining) {
                    Ok(k) => remaining = &remaining[k..],
                    Err(_) => break,
                }
            }
        }
    }
}
