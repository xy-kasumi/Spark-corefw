// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Step pulse generator, multiplexed across N motors (STEP + DIR pin pairs)
//! using hardware-timer ISR.
//!
//! Per-motor 3-state machine (Idle → PulseHigh → PulseLow → Idle), 30µs per phase.
//! One step = 90µs → ~11.1k steps/sec max.
use core::cell::RefCell;

use embassy_stm32::gpio::{Level, Output};
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::Timer as LlTimer;
use embassy_stm32::timer::CoreInstance;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;

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
    // Cached last write to dir_pin; lets us skip GPIO writes on unchanged direction.
    direction: bool,
    step_pin: Option<Output<'static>>,
    dir_pin: Option<Output<'static>>,
}

const fn motor_state_init() -> MotorState {
    MotorState {
        target: 0,
        current: 0,
        step_state: StepState::Idle,
        direction: false,
        step_pin: None,
        dir_pin: None,
    }
}

struct EngineInner<T: CoreInstance, const N: usize> {
    timer: Option<LlTimer<'static, T>>,
    motors: [MotorState; N],
}

impl<T: CoreInstance, const N: usize> EngineInner<T, N> {
    const fn new() -> Self {
        Self {
            timer: None,
            motors: [const { motor_state_init() }; N],
        }
    }
}

pub struct StepGen<T: CoreInstance, const N: usize> {
    inner: BlockingMutex<CriticalSectionRawMutex, RefCell<EngineInner<T, N>>>,
}

impl<T: CoreInstance, const N: usize> StepGen<T, N> {
    pub const fn new() -> Self {
        Self {
            inner: BlockingMutex::new(RefCell::new(EngineInner::new())),
        }
    }

    /// Create StepGen using given timer & per-motor (step, dir) pins.
    /// Configures timer to 30us, but caller is responsible for calling tick() from the timer's ISR.
    pub fn init(
        &'static self,
        tim: T,
        pins: [(Output<'static>, Output<'static>); N],
    ) -> [StepGenHandle<T, N>; N] {
        let timer = LlTimer::new(tim);
        timer.set_frequency(Hertz(33_333));
        timer.enable_update_interrupt(true);
        timer.start();

        self.inner.lock(|cell| {
            let mut e = cell.borrow_mut();
            e.timer = Some(timer);
            for (i, (step, dir)) in pins.into_iter().enumerate() {
                e.motors[i].step_pin = Some(step);
                e.motors[i].dir_pin = Some(dir);
            }
        });

        unsafe {
            T::UpdateInterrupt::enable();
        }

        core::array::from_fn(|i| StepGenHandle {
            engine: self,
            idx: i,
        })
    }

    pub fn tick(&self) {
        self.inner.lock(|cell| {
            let mut e_ref = cell.borrow_mut();
            let e = &mut *e_ref;
            if let Some(t) = e.timer.as_ref() {
                t.clear_update_interrupt();
            }
            for m in e.motors.iter_mut() {
                m.process();
            }
        });
    }
}

impl MotorState {
    fn write_step(&mut self, level: Level) {
        if let Some(p) = self.step_pin.as_mut() {
            p.set_level(level);
        }
    }
    fn write_dir(&mut self, level: Level) {
        if let Some(p) = self.dir_pin.as_mut() {
            p.set_level(level);
        }
    }

    fn process(&mut self) {
        match self.step_state {
            StepState::Idle => {
                if self.current != self.target {
                    let dir = self.target > self.current;
                    if dir != self.direction {
                        self.direction = dir;
                        self.write_dir(Level::from(dir));
                    }
                    self.write_step(Level::High);
                    self.step_state = StepState::PulseHigh;
                }
            }
            StepState::PulseHigh => {
                self.write_step(Level::Low);
                self.step_state = StepState::PulseLow;
                self.current += if self.direction { 1 } else { -1 };
            }
            StepState::PulseLow => {
                self.step_state = StepState::Idle;
            }
        }
    }
}

pub struct StepGenHandle<T: CoreInstance + 'static, const N: usize> {
    engine: &'static StepGen<T, N>,
    idx: usize,
}

impl<T: CoreInstance, const N: usize> Clone for StepGenHandle<T, N> {
    fn clone(&self) -> Self {
        *self
    }
}
impl<T: CoreInstance, const N: usize> Copy for StepGenHandle<T, N> {}

impl<T: CoreInstance, const N: usize> StepGenHandle<T, N> {
    pub fn set_target(&self, target: i32) {
        self.engine.inner.lock(|cell| {
            cell.borrow_mut().motors[self.idx].target = target;
        });
    }

    pub fn current(&self) -> i32 {
        self.engine
            .inner
            .lock(|cell| cell.borrow().motors[self.idx].current)
    }
}
