// Board: BTT Octopus Pro v1.1 with TMC2209 stepper drivers.
// motor7 (PA14 DIR) is omitted because PA14 doubles as SWCLK.
//
// |  m# | step | dir  | en   | uart | diag |
// | --- | ---- | ---- | ---- | ---- | ---- |
// |  m0 | PF13 | PF12 | PF14 | PC4  | PG6  |
// |  m1 | PG0  | PG1  | PF15 | PD11 | PG9  |
// |  m2 | PF11 | PG3  | PG5  | PC6  | PG10 |
// |  m3 | PG4  | PC1  | PA0  | PC7  | PG11 |
// |  m4 | PF9  | PF10 | PG2  | PF2  | PG12 |
// |  m5 | PC13 | PF0  | PF1  | PE4  | PG13 |
// |  m6 | PE2  | PE3  | PD4  | PE1  | PG14 |

use embassy_stm32::gpio::Flex;
use embassy_stm32::interrupt;
use embassy_stm32::peripherals::{PC4, PC6, PC7, PD11, PE1, PE4, PF2, TIM7};

use crate::soft_uart::{self, SoftUart, SoftUartHandle};
use crate::tmc2209::{Tmc2209, TmcTransport};

pub const NUM_MOTORS: usize = 7;
pub const MOTOR_NAMES: [&str; NUM_MOTORS] = ["m0", "m1", "m2", "m3", "m4", "m5", "m6"];

static SOFT_UART: SoftUart<NUM_MOTORS> = SoftUart::new();

#[interrupt]
fn TIM7() {
    SOFT_UART.tick();
}

impl<const N: usize> TmcTransport for SoftUartHandle<N> {
    type Error = soft_uart::Error;
    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        SoftUartHandle::write(self, data).await
    }
    async fn write_then_read(
        &mut self,
        tx: &[u8],
        rx: &mut [u8],
    ) -> Result<(), Self::Error> {
        SoftUartHandle::write_then_read(self, tx, rx).await
    }
}

pub fn init_motors(
    tim7: TIM7,
    uart0: PC4,
    uart1: PD11,
    uart2: PC6,
    uart3: PC7,
    uart4: PF2,
    uart5: PE4,
    uart6: PE1,
) -> [Tmc2209<SoftUartHandle<NUM_MOTORS>>; NUM_MOTORS] {
    let handles = SOFT_UART.init(
        tim7,
        [
            Flex::new(uart0),
            Flex::new(uart1),
            Flex::new(uart2),
            Flex::new(uart3),
            Flex::new(uart4),
            Flex::new(uart5),
            Flex::new(uart6),
        ],
    );
    core::array::from_fn(|i| Tmc2209::new(handles[i], MOTOR_NAMES[i]))
}
