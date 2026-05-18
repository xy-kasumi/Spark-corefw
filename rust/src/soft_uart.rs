// Bit-banged single-wire half-duplex UART for STM32H7, multiplexed across up
// to NUM_PINS GPIOs via a 30µs hardware-timer ISR. Mirrors the C
// drivers/motor/uart1wire.c.
//
// TIM7 (basic timer, not contended by embassy-time) ticks every 30µs. Each
// bit takes 3 ticks (90µs) → ~11.1 kbps. Frame = 1 START low + 8 data LSB-
// first + 1 STOP high.

#![allow(dead_code)]

use core::cell::RefCell;

use embassy_stm32::gpio::{Flex, Pull, Speed};
use embassy_stm32::interrupt;
use embassy_stm32::interrupt::InterruptExt as _;
use embassy_stm32::peripherals::TIM7;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::Timer as LlTimer;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::mutex::Mutex as AsyncMutex;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration};

use crate::tmc2209::TmcTransport;

pub const NUM_PINS: usize = 7;
pub const MAX_FRAME: usize = 8;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    Timeout,
    BufferTooLarge,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Idle,
    Send,
    Receive,       // waiting for falling edge (start bit)
    ReceiveSynced, // edge seen; phase counter active
}

struct EngineInner {
    state: State,
    phase: u8,
    buffer: [u8; MAX_FRAME],
    buffer_size: usize,
    byte_idx: usize,
    bit_idx: u8, // 0=START, 1..=8=DATA, 9=STOP
    current_pin: usize,
    timer: Option<LlTimer<'static, TIM7>>,
    pins: [Option<Flex<'static>>; NUM_PINS],
}

impl EngineInner {
    const fn new() -> Self {
        Self {
            state: State::Idle,
            phase: 0,
            buffer: [0; MAX_FRAME],
            buffer_size: 0,
            byte_idx: 0,
            bit_idx: 0,
            current_pin: 0,
            timer: None,
            pins: [None, None, None, None, None, None, None],
        }
    }
}

pub struct SoftUart {
    inner: BlockingMutex<CriticalSectionRawMutex, RefCell<EngineInner>>,
    signal: Signal<CriticalSectionRawMutex, ()>,
    bus: AsyncMutex<CriticalSectionRawMutex, ()>,
}

static SOFT_UART: SoftUart = SoftUart {
    inner: BlockingMutex::new(RefCell::new(EngineInner::new())),
    signal: Signal::new(),
    bus: AsyncMutex::new(()),
};

impl SoftUart {
    // Configure TIM7 for 30µs periodic interrupt, install the 7 pins as
    // open-drain + pull-up (idle high), enable the NVIC, and return one
    // handle per pin slot.
    pub fn init(
        tim: TIM7,
        mut pins: [Flex<'static>; NUM_PINS],
    ) -> [SoftUartHandle; NUM_PINS] {
        for pin in pins.iter_mut() {
            pin.set_as_input_output_pull(Speed::Low, Pull::Up);
            pin.set_high();
        }

        let timer = LlTimer::new(tim);
        timer.set_frequency(Hertz(33_333)); // ≈30.0003µs
        timer.enable_update_interrupt(true);
        timer.start();

        SOFT_UART.inner.lock(|cell| {
            let mut e = cell.borrow_mut();
            e.timer = Some(timer);
            for (i, p) in pins.into_iter().enumerate() {
                e.pins[i] = Some(p);
            }
        });

        unsafe {
            interrupt::Interrupt::TIM7.enable();
        }

        core::array::from_fn(|i| SoftUartHandle { engine: &SOFT_UART, pin_idx: i })
    }

    async fn transact_write(&'static self, pin_idx: usize, data: &[u8]) -> Result<(), Error> {
        if data.len() > MAX_FRAME {
            return Err(Error::BufferTooLarge);
        }
        let _guard = self.bus.lock().await;
        self.do_tx(pin_idx, data).await
    }

    async fn transact_write_then_read(
        &'static self,
        pin_idx: usize,
        tx: &[u8],
        rx: &mut [u8],
    ) -> Result<(), Error> {
        if tx.len() > MAX_FRAME || rx.len() > MAX_FRAME {
            return Err(Error::BufferTooLarge);
        }
        let _guard = self.bus.lock().await;
        self.do_tx(pin_idx, tx).await?;
        self.do_rx(pin_idx, rx).await
    }

    async fn do_tx(&'static self, pin_idx: usize, data: &[u8]) -> Result<(), Error> {
        self.signal.reset();
        self.inner.lock(|cell| {
            let mut e = cell.borrow_mut();
            e.buffer[..data.len()].copy_from_slice(data);
            e.buffer_size = data.len();
            e.byte_idx = 0;
            e.bit_idx = 0;
            e.phase = 0;
            e.current_pin = pin_idx;
            e.state = State::Send;
        });
        match with_timeout(Duration::from_millis(15), self.signal.wait()).await {
            Ok(()) => Ok(()),
            Err(_) => {
                self.inner.lock(|cell| cell.borrow_mut().state = State::Idle);
                Err(Error::Timeout)
            }
        }
    }

    async fn do_rx(&'static self, pin_idx: usize, out: &mut [u8]) -> Result<(), Error> {
        self.signal.reset();
        self.inner.lock(|cell| {
            let mut e = cell.borrow_mut();
            for b in e.buffer.iter_mut().take(out.len()) {
                *b = 0;
            }
            e.buffer_size = out.len();
            e.byte_idx = 0;
            e.bit_idx = 0;
            e.phase = 0;
            e.current_pin = pin_idx;
            e.state = State::Receive;
        });
        match with_timeout(Duration::from_millis(15), self.signal.wait()).await {
            Ok(()) => {
                self.inner.lock(|cell| {
                    let e = cell.borrow();
                    out.copy_from_slice(&e.buffer[..out.len()]);
                });
                Ok(())
            }
            Err(_) => {
                self.inner.lock(|cell| cell.borrow_mut().state = State::Idle);
                Err(Error::Timeout)
            }
        }
    }
}

#[derive(Copy, Clone)]
pub struct SoftUartHandle {
    engine: &'static SoftUart,
    pin_idx: usize,
}

impl TmcTransport for SoftUartHandle {
    type Error = Error;
    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.engine.transact_write(self.pin_idx, data).await
    }
    async fn write_then_read(
        &mut self,
        tx: &[u8],
        rx: &mut [u8],
    ) -> Result<(), Self::Error> {
        self.engine.transact_write_then_read(self.pin_idx, tx, rx).await
    }
}

// --- ISR: 30µs tick (ports drivers/motor/uart1wire.c::tick) -----------------

#[interrupt]
fn TIM7() {
    let mut done = false;
    SOFT_UART.inner.lock(|cell| {
        let mut e_ref = cell.borrow_mut();
        let e = &mut *e_ref;

        if let Some(t) = e.timer.as_ref() {
            t.clear_update_interrupt();
        }
        if e.state == State::Idle {
            return;
        }
        let pin = match e.pins[e.current_pin].as_mut() {
            Some(p) => p,
            None => return,
        };

        match e.state {
            State::Idle => {}
            State::Send => {
                if e.phase == 0 {
                    let bit_high = match e.bit_idx {
                        0 => false, // START
                        1..=8 => {
                            let data_bit = e.bit_idx - 1;
                            (e.buffer[e.byte_idx] >> data_bit) & 1 != 0
                        }
                        _ => true, // STOP
                    };
                    if bit_high {
                        pin.set_high();
                    } else {
                        pin.set_low();
                    }
                    e.bit_idx += 1;
                    if e.bit_idx >= 10 {
                        e.bit_idx = 0;
                        e.byte_idx += 1;
                        if e.byte_idx >= e.buffer_size {
                            e.state = State::Idle;
                            done = true;
                        }
                    }
                }
                e.phase = (e.phase + 1) % 3;
            }
            State::Receive => {
                if !pin.is_high() {
                    e.state = State::ReceiveSynced;
                    e.phase = 1;
                    e.bit_idx = 0;
                }
            }
            State::ReceiveSynced => {
                if e.phase == 1 {
                    let high = pin.is_high();
                    if (1..=8).contains(&e.bit_idx) {
                        let data_bit = e.bit_idx - 1;
                        let byte_idx = e.byte_idx;
                        if high {
                            e.buffer[byte_idx] |= 1 << data_bit;
                        }
                    }
                    e.bit_idx += 1;
                    if e.bit_idx >= 10 {
                        e.state = State::Receive;
                        e.byte_idx += 1;
                        if e.byte_idx >= e.buffer_size {
                            e.state = State::Idle;
                            done = true;
                        }
                    }
                }
                e.phase = (e.phase + 1) % 3;
            }
        }
    });

    if done {
        SOFT_UART.signal.signal(());
    }
}
