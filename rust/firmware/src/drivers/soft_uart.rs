// Bit-banged single-wire half-duplex UART, multiplexed across N GPIOs via a
// 30µs hardware-timer ISR.
//
// One bit = 3 ticks = 90µs → ~11.1 kbps. Frame = 1 START low + 8 data LSB-
// first + 1 STOP high.
//
// The owning module is responsible for declaring `static SOFT_UART:
// SoftUart<T, N>` and an `#[interrupt] fn <Timer>()` that calls
// `SOFT_UART.tick()`.

#![allow(dead_code)]

use core::cell::RefCell;

use embassy_stm32::gpio::{Flex, Level, Pull, Speed};
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::Timer as LlTimer;
use embassy_stm32::timer::CoreInstance;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;
use embassy_sync::mutex::Mutex as AsyncMutex;
use embassy_sync::signal::Signal;
use embassy_time::{with_timeout, Duration};

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

struct EngineInner<T: CoreInstance, const N: usize> {
    state: State,
    phase: u8,      // 0..3, sub-bit position within the 90µs bit cell
    bit_pos: usize, // global bit index into the frame (0..buffer_size*10)
    buffer: [u8; MAX_FRAME],
    buffer_size: usize,
    current_pin: usize,
    timer: Option<LlTimer<'static, T>>,
    pins: [Option<Flex<'static>>; N],
}

impl<T: CoreInstance, const N: usize> EngineInner<T, N> {
    const fn new() -> Self {
        Self {
            state: State::Idle,
            phase: 0,
            bit_pos: 0,
            buffer: [0; MAX_FRAME],
            buffer_size: 0,
            current_pin: 0,
            timer: None,
            pins: [const { None }; N],
        }
    }
}

fn frame_bit_at(buf: &[u8], pos: usize) -> Option<Level> {
    let byte_idx = pos / 10;
    if byte_idx >= buf.len() {
        return None;
    }
    Some(match pos % 10 {
        0 => Level::Low,  // START
        9 => Level::High, // STOP
        n => {
            if (buf[byte_idx] >> (n - 1)) & 1 != 0 {
                Level::High
            } else {
                Level::Low
            }
        }
    })
}

fn store_rx_bit(buf: &mut [u8], pos: usize, level: Level) {
    let n = pos % 10;
    if (1..=8).contains(&n) && level == Level::High {
        buf[pos / 10] |= 1 << (n - 1);
    }
}

pub struct SoftUart<T: CoreInstance, const N: usize> {
    inner: BlockingMutex<CriticalSectionRawMutex, RefCell<EngineInner<T, N>>>,
    signal: Signal<CriticalSectionRawMutex, ()>,
    bus: AsyncMutex<CriticalSectionRawMutex, ()>,
}

impl<T: CoreInstance, const N: usize> SoftUart<T, N> {
    pub const fn new() -> Self {
        Self {
            inner: BlockingMutex::new(RefCell::new(EngineInner::new())),
            signal: Signal::new(),
            bus: AsyncMutex::new(()),
        }
    }

    // Configure the timer for 30µs periodic interrupt, install the N pins as
    // open-drain + pull-up (idle high), enable the NVIC, and return one
    // handle per pin slot. Caller is responsible for installing the ISR.
    pub fn init(&'static self, tim: T, mut pins: [Flex<'static>; N]) -> [SoftUartHandle<T, N>; N] {
        for pin in pins.iter_mut() {
            pin.set_as_input_output_pull(Speed::Low, Pull::Up);
            pin.set_high();
        }

        let timer = LlTimer::new(tim);
        timer.set_frequency(Hertz(33_333)); // ≈30.0003µs
        timer.enable_update_interrupt(true);
        timer.start();

        self.inner.lock(|cell| {
            let mut e = cell.borrow_mut();
            e.timer = Some(timer);
            for (i, p) in pins.into_iter().enumerate() {
                e.pins[i] = Some(p);
            }
        });

        unsafe {
            T::UpdateInterrupt::enable();
        }

        core::array::from_fn(|i| SoftUartHandle {
            engine: self,
            pin_idx: i,
        })
    }

    // Drive one 30µs ISR tick. Must be called from the timer interrupt handler.
    pub fn tick(&self) {
        let mut done = false;
        self.inner.lock(|cell| {
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
                        match frame_bit_at(&e.buffer[..e.buffer_size], e.bit_pos) {
                            Some(level) => {
                                pin.set_level(level);
                                e.bit_pos += 1;
                            }
                            None => {
                                e.state = State::Idle;
                                done = true;
                            }
                        }
                    }
                    e.phase = (e.phase + 1) % 3;
                }
                State::Receive => {
                    if pin.is_low() {
                        e.state = State::ReceiveSynced;
                        e.phase = 1;
                    }
                }
                State::ReceiveSynced => {
                    if e.phase == 1 {
                        let level = pin.get_level();
                        let size = e.buffer_size;
                        store_rx_bit(&mut e.buffer[..size], e.bit_pos, level);
                        e.bit_pos += 1;
                        // Each byte is its own UART frame; resync per byte by
                        // returning to Receive to hunt for the next start bit.
                        if e.bit_pos % 10 == 0 {
                            if e.bit_pos >= size * 10 {
                                e.state = State::Idle;
                                done = true;
                            } else {
                                e.state = State::Receive;
                            }
                        }
                    }
                    e.phase = (e.phase + 1) % 3;
                }
            }
        });

        if done {
            self.signal.signal(());
        }
    }

    async fn transact_write(&self, pin_idx: usize, data: &[u8]) -> Result<(), Error> {
        if data.len() > MAX_FRAME {
            return Err(Error::BufferTooLarge);
        }
        let _guard = self.bus.lock().await;
        self.do_tx(pin_idx, data).await
    }

    async fn transact_write_then_read(
        &self,
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

    async fn do_tx(&self, pin_idx: usize, data: &[u8]) -> Result<(), Error> {
        self.signal.reset();
        self.inner.lock(|cell| {
            let mut e = cell.borrow_mut();
            e.buffer[..data.len()].copy_from_slice(data);
            e.buffer_size = data.len();
            e.bit_pos = 0;
            e.phase = 0;
            e.current_pin = pin_idx;
            e.state = State::Send;
        });
        match with_timeout(Duration::from_millis(15), self.signal.wait()).await {
            Ok(()) => Ok(()),
            Err(_) => {
                self.inner
                    .lock(|cell| cell.borrow_mut().state = State::Idle);
                Err(Error::Timeout)
            }
        }
    }

    async fn do_rx(&self, pin_idx: usize, out: &mut [u8]) -> Result<(), Error> {
        self.signal.reset();
        self.inner.lock(|cell| {
            let mut e = cell.borrow_mut();
            for b in e.buffer.iter_mut().take(out.len()) {
                *b = 0;
            }
            e.buffer_size = out.len();
            e.bit_pos = 0;
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
                self.inner
                    .lock(|cell| cell.borrow_mut().state = State::Idle);
                Err(Error::Timeout)
            }
        }
    }
}

pub struct SoftUartHandle<T: CoreInstance + 'static, const N: usize> {
    engine: &'static SoftUart<T, N>,
    pin_idx: usize,
}

impl<T: CoreInstance, const N: usize> Clone for SoftUartHandle<T, N> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T: CoreInstance, const N: usize> Copy for SoftUartHandle<T, N> {}

impl<T: CoreInstance, const N: usize> SoftUartHandle<T, N> {
    pub async fn write(&self, data: &[u8]) -> Result<(), Error> {
        self.engine.transact_write(self.pin_idx, data).await
    }
    pub async fn write_then_read(&self, tx: &[u8], rx: &mut [u8]) -> Result<(), Error> {
        self.engine
            .transact_write_then_read(self.pin_idx, tx, rx)
            .await
    }
}
