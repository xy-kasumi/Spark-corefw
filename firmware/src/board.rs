// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Board: BTT Octopus Pro v1.1 with TMC2209 stepper drivers.
//! Physical connections: see `docs/board_pins.png` at the repo root.
//! motor7 (PA14 DIR) is omitted because PA14 doubles as SWCLK.
//!
//! |  m# | step | dir  | en   | uart | diag |
//! | --- | ---- | ---- | ---- | ---- | ---- |
//! |  m0 | PF13 | PF12 | PF14 | PC4  | PG6  |
//! |  m1 | PG0  | PG1  | PF15 | PD11 | PG9  |
//! |  m2 | PF11 | PG3  | PG5  | PC6  | PG10 |
//! |  m3 | PG4  | PC1  | PA0  | PC7  | PG11 |
//! |  m4 | PF9  | PF10 | PG2  | PF2  | PG12 |
//! |  m5 | PC13 | PF0  | PF1  | PE4  | PG13 |
//! |  m6 | PE2  | PE3  | PD4  | PE1  | PG14 |
//!
//! Console UART: USART2 on PD5 (TX) / PD6 (RX), DMA1_CH0 / DMA1_CH1.

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Flex, Level, Output, OutputType, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::interrupt;
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{
    PA0, PC1, PC13, PC4, PC6, PC7, PD11, PD4, PE1, PE2, PE3, PE4, PF0, PF1, PF10, PF11, PF12, PF13,
    PF14, PF15, PF2, PF9, PG0, PG1, PG2, PG3, PG4, PG5, TIM1, TIM6, TIM7,
};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CountingMode;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::CoreInstance;
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::{bind_interrupts, peripherals, usart};

use crate::drivers::digital_output::DigitalOutput;
use crate::drivers::pulser::{PulserBus, PulserDevice};
use crate::drivers::pwm_output::PwmOutput;
use crate::drivers::serial::Serial;
use crate::drivers::soft_uart::{self, SoftUart, SoftUartHandle};
use crate::drivers::step_gen::{StepGen, StepGenHandle};
use crate::drivers::tmc2209::{Tmc2209, TmcTransport};

pub const NUM_MOTORS: usize = 7;
pub const MOTOR_NAMES: [&str; NUM_MOTORS] = ["m0", "m1", "m2", "m3", "m4", "m5", "m6"];

type SoftUartTim = TIM7;
type StepGenTim = TIM6;

pub type TmcBus = SoftUartHandle<SoftUartTim, NUM_MOTORS>;
pub type MotorStepping = StepGenHandle<StepGenTim, NUM_MOTORS>;
pub type MotorConfig = Tmc2209<TmcBus>;

/// Concrete I2C bus backing the pulser. Pin/peripheral choice is local to `init`.
pub type PulserBusImpl = I2c<'static, Async>;
pub type Pulser = crate::pulser::Pulser<PulserBusImpl>;

/// Pump gate output (PA3, active-high).
pub type Pump = crate::pump::Pump<Output<'static>>;

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

impl PulserBus for PulserBusImpl {
    type Error = i2c::Error;
    async fn write(&mut self, addr: u8, data: &[u8]) -> Result<(), Self::Error> {
        I2c::write(self, addr, data).await
    }
    async fn write_read(&mut self, addr: u8, tx: &[u8], rx: &mut [u8]) -> Result<(), Self::Error> {
        I2c::write_read(self, addr, tx, rx).await
    }
}

impl DigitalOutput for Output<'static> {
    fn set(&mut self, high: bool) {
        self.set_level(high.into());
    }
}

impl PwmOutput for ToolSupplyPwm {
    fn init(&mut self, period_ms: f32) {
        self.set_frequency(Hertz((1000.0 / period_ms) as u32));
        self.ch1().enable();
    }
    fn set(&mut self, duty: f32) {
        let mut ch = self.ch1();
        let max = ch.max_duty_cycle();
        ch.set_duty_cycle((duty * max as f32) as u16);
    }
}

static SOFT_UART: SoftUart<SoftUartTim, NUM_MOTORS> = SoftUart::new();
static STEP_GEN: StepGen<StepGenTim, NUM_MOTORS> = StepGen::new();

#[interrupt]
fn TIM7() {
    SOFT_UART.tick();
}

#[interrupt]
fn TIM6_DAC() {
    STEP_GEN.tick();
}

impl<T: CoreInstance, const N: usize> TmcTransport for SoftUartHandle<T, N> {
    type Error = soft_uart::Error;
    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        SoftUartHandle::write(self, data).await
    }
    async fn write_then_read(&mut self, tx: &[u8], rx: &mut [u8]) -> Result<(), Self::Error> {
        SoftUartHandle::write_then_read(self, tx, rx).await
    }
}

pub struct Motors {
    pub tmc: [MotorConfig; NUM_MOTORS],
    pub step: [MotorStepping; NUM_MOTORS],
}

/// Tool supply servo PWM (TIM1 channel 1 on PE9).
pub type ToolSupplyPwm = SimplePwm<'static, TIM1>;
pub type ToolSupply = crate::toolsupply::ToolSupply<ToolSupplyPwm>;

pub struct Board {
    pub console: &'static Serial,
    pub motors: Motors,
    pub pulser: Pulser,
    /// Pump gate (PA3, active-high).
    pub pump: Output<'static>,
    pub toolsupply_pwm: ToolSupplyPwm,
}

pub fn init(spawner: &Spawner, console_baud: u32) -> Board {
    let p = embassy_stm32::init(Default::default());

    let mut cfg = UartConfig::default();
    cfg.baudrate = console_baud;
    let uart = Uart::new(p.USART2, p.PD6, p.PD5, Irqs, p.DMA1_CH0, p.DMA1_CH1, cfg).unwrap();
    let console = Serial::init(spawner, uart);

    let motors = init_motors(
        p.TIM7,
        p.TIM6,
        (p.PC4, p.PF13, p.PF12, p.PF14),
        (p.PD11, p.PG0, p.PG1, p.PF15),
        (p.PC6, p.PF11, p.PG3, p.PG5),
        (p.PC7, p.PG4, p.PC1, p.PA0),
        (p.PF2, p.PF9, p.PF10, p.PG2),
        (p.PE4, p.PC13, p.PF0, p.PF1),
        (p.PE1, p.PE2, p.PE3, p.PD4),
    );

    // Pulser board on I2C1: PB8 (SCL) / PB9 (SDA), 400 kHz fast mode.
    let i2c = I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH2,
        p.DMA1_CH3,
        Hertz(400_000),
        Default::default(),
    );
    let pulser = Pulser::new(PulserDevice::new(i2c));

    // Pump gate on PA3 (active-high), starts off.
    let pump = Output::new(p.PA3, Level::Low, Speed::Low);

    // Tool supply servo: TIM1 channel 1 on PE9, 50 Hz carrier.
    let toolsupply_pwm = SimplePwm::new(
        p.TIM1,
        Some(PwmPin::new_ch1(p.PE9, OutputType::PushPull)),
        None,
        None,
        None,
        Hertz(50),
        CountingMode::EdgeAlignedUp,
    );

    Board {
        console,
        motors,
        pulser,
        pump,
        toolsupply_pwm,
    }
}

/// Pin tuple per motor: (uart, step, dir, enable).
fn init_motors(
    tim_uart: TIM7,
    tim_step: TIM6,
    m0: (PC4, PF13, PF12, PF14),
    m1: (PD11, PG0, PG1, PF15),
    m2: (PC6, PF11, PG3, PG5),
    m3: (PC7, PG4, PC1, PA0),
    m4: (PF2, PF9, PF10, PG2),
    m5: (PE4, PC13, PF0, PF1),
    m6: (PE1, PE2, PE3, PD4),
) -> Motors {
    let uart_handles = SOFT_UART.init(
        tim_uart,
        [
            Flex::new(m0.0),
            Flex::new(m1.0),
            Flex::new(m2.0),
            Flex::new(m3.0),
            Flex::new(m4.0),
            Flex::new(m5.0),
            Flex::new(m6.0),
        ],
    );

    // EN pins are active-low and start High = de-energized; step_gen energizes on demand.
    let step_handles = STEP_GEN.init(
        tim_step,
        [
            (
                Output::new(m0.1, Level::Low, Speed::Low),
                Output::new(m0.2, Level::Low, Speed::Low),
                Output::new(m0.3, Level::High, Speed::Low),
            ),
            (
                Output::new(m1.1, Level::Low, Speed::Low),
                Output::new(m1.2, Level::Low, Speed::Low),
                Output::new(m1.3, Level::High, Speed::Low),
            ),
            (
                Output::new(m2.1, Level::Low, Speed::Low),
                Output::new(m2.2, Level::Low, Speed::Low),
                Output::new(m2.3, Level::High, Speed::Low),
            ),
            (
                Output::new(m3.1, Level::Low, Speed::Low),
                Output::new(m3.2, Level::Low, Speed::Low),
                Output::new(m3.3, Level::High, Speed::Low),
            ),
            (
                Output::new(m4.1, Level::Low, Speed::Low),
                Output::new(m4.2, Level::Low, Speed::Low),
                Output::new(m4.3, Level::High, Speed::Low),
            ),
            (
                Output::new(m5.1, Level::Low, Speed::Low),
                Output::new(m5.2, Level::Low, Speed::Low),
                Output::new(m5.3, Level::High, Speed::Low),
            ),
            (
                Output::new(m6.1, Level::Low, Speed::Low),
                Output::new(m6.2, Level::Low, Speed::Low),
                Output::new(m6.3, Level::High, Speed::Low),
            ),
        ],
    );

    let tmc: [MotorConfig; NUM_MOTORS] =
        core::array::from_fn(|i| Tmc2209::new(uart_handles[i], MOTOR_NAMES[i]));

    Motors {
        tmc,
        step: step_handles,
    }
}
