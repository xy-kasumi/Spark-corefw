// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Board: BTT Octopus Pro v1.1 with TMC2209 stepper drivers.
//! Official pinouts: https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-Pro/blob/master/Hardware/BIGTREETECH%20Octopus%20Pro%20-%20PIN.pdf
//!
//! Spark machine connections: see `docs/board_pins.png`.

use embassy_stm32::gpio;
use embassy_stm32::i2c;
use embassy_stm32::interrupt;
use embassy_stm32::mode;
use embassy_stm32::time;
use embassy_stm32::timer;
use embassy_stm32::{bind_interrupts, peripherals, usart};

use crate::drivers::digital_out;
use crate::drivers::pulser::{self, Bus};
use crate::drivers::serial;
use crate::drivers::soft_uart;
use crate::drivers::step_gen;
use crate::drivers::tmc2209::{self, TmcTransport};

pub const NUM_MOTORS: usize = 7;

type SoftUartTim = peripherals::TIM7;
type StepGenTim = peripherals::TIM6;

pub type TmcBus = soft_uart::SoftUartHandle<SoftUartTim, NUM_MOTORS>;
pub type MotorStepping = step_gen::StepGenHandle<StepGenTim, NUM_MOTORS>;
pub type MotorConfig = tmc2209::Device<TmcBus>;
pub type PulserBusImpl = i2c::I2c<'static, mode::Async>;
pub type Pulser = crate::pulser::Device<PulserBusImpl>;
pub type Pump = crate::pump::Pump<gpio::Output<'static>>;

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[interrupt]
fn TIM7() {
    SOFT_UART.tick();
}

#[interrupt]
fn TIM6_DAC() {
    STEP_GEN.tick();
}

pub struct Board {
    pub serial: &'static serial::Device,
    pub motors: Motors,
    pub pulser: Pulser,
    /// Pump gate (PA3, active-high).
    pub pump: gpio::Output<'static>,
}

pub struct Motors {
    pub tmc: [MotorConfig; NUM_MOTORS],
    pub step: [MotorStepping; NUM_MOTORS],
    /// EN lines (active-low), held low for the program's lifetime so every motor
    /// stays energized. Never read after construction; owned here only to keep
    /// the pins driven.
    #[allow(dead_code)]
    en: [gpio::Output<'static>; NUM_MOTORS],
}

static SOFT_UART: soft_uart::Uart<SoftUartTim, NUM_MOTORS> = soft_uart::Uart::new();
static STEP_GEN: step_gen::Gen<StepGenTim, NUM_MOTORS> = step_gen::Gen::new();

pub fn init(spawner: &embassy_executor::Spawner, console_baud: u32) -> Board {
    let p = embassy_stm32::init(Default::default());

    // Host serial.
    let mut cfg = usart::Config::default();
    cfg.baudrate = console_baud;
    let uart = usart::Uart::new(p.USART2, p.PD6, p.PD5, Irqs, p.DMA1_CH0, p.DMA1_CH1, cfg).unwrap();
    let serial = serial::Device::init(spawner, uart);

    // Motors. note: motor7 (PA14 DIR, also SWCLK) is omitted because it breaks SWD flashing.
    let uart_handles = SOFT_UART.init(
        p.TIM7,
        [
            gpio::Flex::new(p.PC4),
            gpio::Flex::new(p.PD11),
            gpio::Flex::new(p.PC6),
            gpio::Flex::new(p.PC7),
            gpio::Flex::new(p.PF2),
            gpio::Flex::new(p.PE4),
            gpio::Flex::new(p.PE1),
        ],
    );
    let step_handles = STEP_GEN.init(
        p.TIM6,
        [
            (out(p.PF13), out(p.PF12)),
            (out(p.PG0), out(p.PG1)),
            (out(p.PF11), out(p.PG3)),
            (out(p.PG4), out(p.PC1)),
            (out(p.PF9), out(p.PF10)),
            (out(p.PC13), out(p.PF0)),
            (out(p.PE2), out(p.PE3)),
        ],
    );
    // EN is active-low and we want to enable them; so it works w/o explicit set(low).
    let en = [
        out(p.PF14),
        out(p.PF15),
        out(p.PG5),
        out(p.PA0),
        out(p.PG2),
        out(p.PF1),
        out(p.PD4),
    ];
    let tmc: [MotorConfig; NUM_MOTORS] =
        core::array::from_fn(|i| tmc2209::Device::new(uart_handles[i]));
    let motors = Motors {
        tmc,
        step: step_handles,
        en,
    };

    // Pulser board on I2C1: PB8 (SCL) / PB9 (SDA), 400 kHz fast mode.
    let i2c = i2c::I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH2,
        p.DMA1_CH3,
        time::Hertz(400_000),
        Default::default(),
    );
    let pulser = Pulser::new(pulser::Device::new(i2c));

    // Misc. peripherals.
    let pump = out(p.PA3); // default off (b/c active high)

    Board {
        serial,
        motors,
        pulser,
        pump,
    }
}

fn out(pin: impl gpio::Pin) -> gpio::Output<'static> {
    gpio::Output::new(pin, gpio::Level::Low, gpio::Speed::Low)
}

impl Bus for PulserBusImpl {
    type Error = i2c::Error;
    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Self::Error> {
        i2c::I2c::write(self, addr, data).await
    }
    async fn write_read(&mut self, addr: u8, tx: &[u8], rx: &mut [u8]) -> Result<(), Self::Error> {
        i2c::I2c::write_read(self, addr, tx, rx).await
    }
}

impl digital_out::Pin for gpio::Output<'static> {
    fn set(&mut self, high: bool) {
        self.set_level(high.into());
    }
}

impl<T: timer::CoreInstance, const N: usize> TmcTransport for soft_uart::SoftUartHandle<T, N> {
    type Error = soft_uart::Error;
    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        soft_uart::SoftUartHandle::write(self, data).await
    }
    async fn write_then_read(&mut self, tx: &[u8], rx: &mut [u8]) -> Result<(), Self::Error> {
        soft_uart::SoftUartHandle::write_then_read(self, tx, rx).await
    }
}
