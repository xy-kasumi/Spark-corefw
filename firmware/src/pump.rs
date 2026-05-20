//! Coolant/dielectric pump: a single active-high GPIO with settle delays.

use embassy_stm32::gpio::Output;
use embassy_time::{Duration, Timer};

pub struct Pump {
    gpio: Output<'static>,
}

impl Pump {
    pub fn new(gpio: Output<'static>) -> Self {
        Self { gpio }
    }

    /// Drive the pump on or off, then wait for it to settle: 1 s after starting,
    /// 100 ms after stopping (blocking).
    pub async fn set_enable(&mut self, enable: bool) {
        if enable {
            self.gpio.set_high();
            Timer::after(Duration::from_millis(1000)).await;
        } else {
            self.gpio.set_low();
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}
