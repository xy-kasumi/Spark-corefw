//! Comm task: read UART bytes, frame into lines, parse + dispatch, reply ok/err.

use embassy_stm32::{mode, usart::UartRx};
use heapless::Vec;
use model::gcode::{self, Command, ParseError};

use crate::dispatch;
use crate::log::{self, TxMutex};
use crate::motion::Shared;

const LINE_CAP: usize = 128;

pub async fn run(mut rx: UartRx<'static, mode::Async>, motion: &Shared, tx: &TxMutex) -> ! {
    let mut line: Vec<u8, LINE_CAP> = Vec::new();
    let mut buf = [0u8; 32];
    loop {
        let n = match rx.read_until_idle(&mut buf).await {
            Ok(n) => n,
            Err(_) => {
                log::log(tx, b"err rx\r\n").await;
                continue;
            }
        };
        for &b in &buf[..n] {
            match b {
                b'!' => {
                    motion.lock().await.cancel();
                    line.clear();
                    log::log(tx, b"cancelled\r\n").await;
                }
                b'\n' | b'\r' => {
                    if !line.is_empty() {
                        match handle_line(&line, motion).await {
                            Ok(()) => log::log(tx, b"ok\r\n").await,
                            Err(e) => log::log3(tx, b"err ", err_name(e), b"\r\n").await,
                        }
                        line.clear();
                    }
                }
                _ => {
                    let _ = line.push(b); // Phase 4: surface overflow as protocol error
                }
            }
        }
    }
}

async fn handle_line(line: &[u8], motion: &Shared) -> Result<(), ParseError> {
    let cmd: Command = gcode::parse(line)?;
    let mut m = motion.lock().await;
    dispatch::exec(cmd, &mut m);
    Ok(())
}

fn err_name(e: ParseError) -> &'static [u8] {
    match e {
        ParseError::Empty => b"empty",
        ParseError::UnknownCommand => b"unknown",
        ParseError::BadAxis => b"bad-axis",
        ParseError::BadNumber => b"bad-number",
        ParseError::ExpectedSeparator => b"missing-sep",
        ParseError::TrailingGarbage => b"trailing",
    }
}
