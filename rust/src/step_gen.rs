// Step pulse generator. A 30µs hardware-timer ISR drives a per-motor 3-state
// machine (Idle → PulseHigh → PulseLow → Idle). Mirrors app/src/motor.c.
//
// One step = 3 ticks = 90µs → max ≈11.1k steps/sec. Each motor independently
// advances toward its target_steps; while at target, it de-energizes after
// an idle timeout (unless always_energized).
//
// The owning module declares `static STEP_GEN: StepGen<N>` and an
// `#[interrupt] fn TIM6_DAC()` that calls `STEP_GEN.tick()`.

#![allow(dead_code)]

use core::cell::RefCell;

use embassy_stm32::gpio::Output;
use embassy_stm32::interrupt::InterruptExt as _;
use embassy_stm32::peripherals::TIM6;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::Timer as LlTimer;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;

const TICK_PERIOD_US: u32 = 30;
const DEFAULT_IDLE_MS: u32 = 200;

#[derive(Clone, Copy, PartialEq, Eq)]
enum StepState {
    Idle,
    PulseHigh,
    PulseLow,
}

struct MotorState {
    target: i32,
    current: i32,
    step_state: StepState,
    direction: bool, // true = +, mirrors what's been written to dir_pin
    energized: bool,
    always_energized: bool,
    idle_timeout_ticks: u32,
    idle_ticks: u32,
    step_pin: Option<Output<'static>>,
    dir_pin: Option<Output<'static>>,
    en_pin: Option<Output<'static>>, // active-low: low = energized
}

const fn motor_state_init() -> MotorState {
    MotorState {
        target: 0,
        current: 0,
        step_state: StepState::Idle,
        direction: false,
        energized: false,
        always_energized: false,
        idle_timeout_ticks: (DEFAULT_IDLE_MS * 1000) / TICK_PERIOD_US,
        idle_ticks: 0,
        step_pin: None,
        dir_pin: None,
        en_pin: None,
    }
}

struct EngineInner<const N: usize> {
    timer: Option<LlTimer<'static, TIM6>>,
    motors: [MotorState; N],
}

impl<const N: usize> EngineInner<N> {
    const fn new() -> Self {
        Self {
            timer: None,
            motors: [const { motor_state_init() }; N],
        }
    }
}

pub struct StepGen<const N: usize> {
    inner: BlockingMutex<CriticalSectionRawMutex, RefCell<EngineInner<N>>>,
}

impl<const N: usize> StepGen<N> {
    pub const fn new() -> Self {
        Self {
            inner: BlockingMutex::new(RefCell::new(EngineInner::new())),
        }
    }

    // Install timer + per-motor (step, dir, en) pin triples. Caller is
    // responsible for installing the ISR that calls `tick()`.
    pub fn init(
        &'static self,
        tim: TIM6,
        pins: [(Output<'static>, Output<'static>, Output<'static>); N],
    ) -> [StepGenHandle<N>; N] {
        let timer = LlTimer::new(tim);
        timer.set_frequency(Hertz(33_333));
        timer.enable_update_interrupt(true);
        timer.start();

        self.inner.lock(|cell| {
            let mut e = cell.borrow_mut();
            e.timer = Some(timer);
            for (i, (step, dir, en)) in pins.into_iter().enumerate() {
                e.motors[i].step_pin = Some(step);
                e.motors[i].dir_pin = Some(dir);
                e.motors[i].en_pin = Some(en);
            }
        });

        unsafe {
            embassy_stm32::interrupt::TIM6_DAC.enable();
        }

        core::array::from_fn(|i| StepGenHandle { engine: self, idx: i })
    }

    // Called from the TIM6_DAC interrupt handler.
    pub fn tick(&self) {
        self.inner.lock(|cell| {
            let mut e_ref = cell.borrow_mut();
            let e = &mut *e_ref;
            if let Some(t) = e.timer.as_ref() {
                t.clear_update_interrupt();
            }
            for m in e.motors.iter_mut() {
                process_motor(m);
            }
        });
    }
}

fn process_motor(m: &mut MotorState) {
    match m.step_state {
        StepState::Idle => {
            if m.current != m.target {
                m.idle_ticks = 0;
                if !m.energized {
                    if let Some(en) = m.en_pin.as_mut() {
                        en.set_low(); // active-low: enable
                    }
                    m.energized = true;
                }
                let dir = m.target > m.current;
                if dir != m.direction {
                    m.direction = dir;
                    if let Some(d) = m.dir_pin.as_mut() {
                        if dir {
                            d.set_high();
                        } else {
                            d.set_low();
                        }
                    }
                }
                if let Some(s) = m.step_pin.as_mut() {
                    s.set_high();
                }
                m.step_state = StepState::PulseHigh;
            } else if !m.always_energized {
                if m.idle_ticks < m.idle_timeout_ticks {
                    m.idle_ticks += 1;
                } else if m.energized {
                    if let Some(en) = m.en_pin.as_mut() {
                        en.set_high(); // active-low: disable
                    }
                    m.energized = false;
                }
            }
        }
        StepState::PulseHigh => {
            if let Some(s) = m.step_pin.as_mut() {
                s.set_low();
            }
            m.step_state = StepState::PulseLow;
            if m.target > m.current {
                m.current += 1;
            } else {
                m.current -= 1;
            }
        }
        StepState::PulseLow => {
            m.step_state = StepState::Idle;
        }
    }
}

pub struct StepGenHandle<const N: usize> {
    engine: &'static StepGen<N>,
    idx: usize,
}

impl<const N: usize> Clone for StepGenHandle<N> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<const N: usize> Copy for StepGenHandle<N> {}

impl<const N: usize> StepGenHandle<N> {
    pub fn set_target(&self, target: i32) {
        self.engine.inner.lock(|cell| {
            cell.borrow_mut().motors[self.idx].target = target;
        });
    }

    pub fn current(&self) -> i32 {
        self.engine.inner.lock(|cell| cell.borrow().motors[self.idx].current)
    }

    pub fn target(&self) -> i32 {
        self.engine.inner.lock(|cell| cell.borrow().motors[self.idx].target)
    }

    pub fn set_idle_timeout_ms(&self, ms: u32) {
        let ticks = (ms * 1000) / TICK_PERIOD_US;
        self.engine.inner.lock(|cell| {
            cell.borrow_mut().motors[self.idx].idle_timeout_ticks = ticks;
        });
    }

    pub fn set_always_energized(&self, always: bool) {
        self.engine.inner.lock(|cell| {
            cell.borrow_mut().motors[self.idx].always_energized = always;
        });
    }
}
