// SPDX-FileCopyrightText: 夕月霞
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

use embassy_stm32::gpio;
use embassy_stm32::i2c;
use embassy_stm32::interrupt;
use embassy_stm32::mode;
use embassy_stm32::time;
use embassy_stm32::timer;
use embassy_stm32::timer::low_level;
use embassy_stm32::timer::simple_pwm;
use embassy_stm32::{bind_interrupts, peripherals, usart};

use crate::drivers::digital_out;
use crate::drivers::pulser::{self, Bus};
use crate::drivers::pwm_out;
use crate::drivers::serial;
use crate::drivers::soft_uart;
use crate::drivers::step_gen;
use crate::drivers::tmc2209::{self, TmcTransport};

pub const NUM_MOTORS: usize = 7;
pub const MOTOR_NAMES: [&str; NUM_MOTORS] = ["m0", "m1", "m2", "m3", "m4", "m5", "m6"];

type SoftUartTim = peripherals::TIM7;
type StepGenTim = peripherals::TIM6;

pub type TmcBus = soft_uart::SoftUartHandle<SoftUartTim, NUM_MOTORS>;
pub type MotorStepping = step_gen::StepGenHandle<StepGenTim, NUM_MOTORS>;
pub type MotorConfig = tmc2209::Device<TmcBus>;

/// Concrete I2C bus backing the pulser. Pin/peripheral choice is local to `init`.
pub type PulserBusImpl = i2c::I2c<'static, mode::Async>;
pub type Pulser = crate::pulser::Device<PulserBusImpl>;

/// Pump gate output (PA3, active-high).
pub type Pump = crate::pump::Pump<gpio::Output<'static>>;

bind_interrupts!(struct Irqs {
    USART2 => usart::InterruptHandler<peripherals::USART2>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

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

impl pwm_out::Pin for ToolSupplyPwm {
    fn init(&mut self, period_ms: f32) {
        self.set_frequency(time::Hertz((1000.0 / period_ms) as u32));
        self.ch1().enable();
    }
    fn set(&mut self, duty: f32) {
        let mut ch = self.ch1();
        let max = ch.max_duty_cycle();
        ch.set_duty_cycle((duty * max as f32) as u16);
    }
}

static SOFT_UART: soft_uart::Uart<SoftUartTim, NUM_MOTORS> = soft_uart::Uart::new();
static STEP_GEN: step_gen::Gen<StepGenTim, NUM_MOTORS> = step_gen::Gen::new();

#[interrupt]
fn TIM7() {
    SOFT_UART.tick();
}

#[interrupt]
fn TIM6_DAC() {
    STEP_GEN.tick();
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

pub struct Motors {
    pub tmc: [MotorConfig; NUM_MOTORS],
    pub step: [MotorStepping; NUM_MOTORS],
    /// EN lines (active-low), held low for the program's lifetime so every motor
    /// stays energized. Never read after construction; owned here only to keep
    /// the pins driven.
    #[allow(dead_code)]
    en: [gpio::Output<'static>; NUM_MOTORS],
}

/// Tool supply servo PWM (TIM1 channel 1 on PE9).
pub type ToolSupplyPwm = simple_pwm::SimplePwm<'static, peripherals::TIM1>;
pub type ToolSupply = crate::toolsupply::ToolSupply<ToolSupplyPwm>;

pub struct Board {
    pub console: &'static serial::Device,
    pub motors: Motors,
    pub pulser: Pulser,
    /// Pump gate (PA3, active-high).
    pub pump: gpio::Output<'static>,
    pub toolsupply_pwm: ToolSupplyPwm,
}

pub fn init(spawner: &embassy_executor::Spawner, console_baud: u32) -> Board {
    let p = embassy_stm32::init(Default::default());

    let mut cfg = usart::Config::default();
    cfg.baudrate = console_baud;
    let uart = usart::Uart::new(p.USART2, p.PD6, p.PD5, Irqs, p.DMA1_CH0, p.DMA1_CH1, cfg).unwrap();
    let console = serial::Device::init(spawner, uart);

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

    // Pump gate on PA3 (active-high), starts off.
    let pump = gpio::Output::new(p.PA3, gpio::Level::Low, gpio::Speed::Low);

    // Tool supply servo: TIM1 channel 1 on PE9, 50 Hz carrier.
    let toolsupply_pwm = simple_pwm::SimplePwm::new(
        p.TIM1,
        Some(simple_pwm::PwmPin::new_ch1(p.PE9, gpio::OutputType::PushPull)),
        None,
        None,
        None,
        time::Hertz(50),
        low_level::CountingMode::EdgeAlignedUp,
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
    tim_uart: peripherals::TIM7,
    tim_step: peripherals::TIM6,
    m0: (peripherals::PC4, peripherals::PF13, peripherals::PF12, peripherals::PF14),
    m1: (peripherals::PD11, peripherals::PG0, peripherals::PG1, peripherals::PF15),
    m2: (peripherals::PC6, peripherals::PF11, peripherals::PG3, peripherals::PG5),
    m3: (peripherals::PC7, peripherals::PG4, peripherals::PC1, peripherals::PA0),
    m4: (peripherals::PF2, peripherals::PF9, peripherals::PF10, peripherals::PG2),
    m5: (peripherals::PE4, peripherals::PC13, peripherals::PF0, peripherals::PF1),
    m6: (peripherals::PE1, peripherals::PE2, peripherals::PE3, peripherals::PD4),
) -> Motors {
    let uart_handles = SOFT_UART.init(
        tim_uart,
        [
            gpio::Flex::new(m0.0),
            gpio::Flex::new(m1.0),
            gpio::Flex::new(m2.0),
            gpio::Flex::new(m3.0),
            gpio::Flex::new(m4.0),
            gpio::Flex::new(m5.0),
            gpio::Flex::new(m6.0),
        ],
    );

    // EN pins are active-low. Motors are always energized.
    let en = [
        gpio::Output::new(m0.3, gpio::Level::Low, gpio::Speed::Low),
        gpio::Output::new(m1.3, gpio::Level::Low, gpio::Speed::Low),
        gpio::Output::new(m2.3, gpio::Level::Low, gpio::Speed::Low),
        gpio::Output::new(m3.3, gpio::Level::Low, gpio::Speed::Low),
        gpio::Output::new(m4.3, gpio::Level::Low, gpio::Speed::Low),
        gpio::Output::new(m5.3, gpio::Level::Low, gpio::Speed::Low),
        gpio::Output::new(m6.3, gpio::Level::Low, gpio::Speed::Low),
    ];

    let step_handles = STEP_GEN.init(
        tim_step,
        [
            (
                gpio::Output::new(m0.1, gpio::Level::Low, gpio::Speed::Low),
                gpio::Output::new(m0.2, gpio::Level::Low, gpio::Speed::Low),
            ),
            (
                gpio::Output::new(m1.1, gpio::Level::Low, gpio::Speed::Low),
                gpio::Output::new(m1.2, gpio::Level::Low, gpio::Speed::Low),
            ),
            (
                gpio::Output::new(m2.1, gpio::Level::Low, gpio::Speed::Low),
                gpio::Output::new(m2.2, gpio::Level::Low, gpio::Speed::Low),
            ),
            (
                gpio::Output::new(m3.1, gpio::Level::Low, gpio::Speed::Low),
                gpio::Output::new(m3.2, gpio::Level::Low, gpio::Speed::Low),
            ),
            (
                gpio::Output::new(m4.1, gpio::Level::Low, gpio::Speed::Low),
                gpio::Output::new(m4.2, gpio::Level::Low, gpio::Speed::Low),
            ),
            (
                gpio::Output::new(m5.1, gpio::Level::Low, gpio::Speed::Low),
                gpio::Output::new(m5.2, gpio::Level::Low, gpio::Speed::Low),
            ),
            (
                gpio::Output::new(m6.1, gpio::Level::Low, gpio::Speed::Low),
                gpio::Output::new(m6.2, gpio::Level::Low, gpio::Speed::Low),
            ),
        ],
    );

    let tmc: [MotorConfig; NUM_MOTORS] =
        core::array::from_fn(|i| tmc2209::Device::new(uart_handles[i], MOTOR_NAMES[i]));

    Motors {
        tmc,
        step: step_handles,
        en,
    }
}
