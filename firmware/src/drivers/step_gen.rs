// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Step pulse generator, multiplexed across N motors (STEP + DIR pin pairs)
//! using hardware-timer ISR.
//!
//! Per-motor 3-state machine (Idle → PulseHigh → PulseLow → Idle), 30µs per phase.
//! One step = 90µs → ~11.1k steps/sec max.
//!
//! EN is driven here too: a motor energizes in the same tick it begins stepping
//! and de-energizes after an idle timeout. Energize must be co-timed with the step
//! decision, so it lives in the ISR alongside position counting rather than in a
//! slower caller loop (which would step a still-disabled driver and drop steps).

use core::cell::RefCell;

use embassy_stm32::gpio::{Level, Output};
use embassy_stm32::interrupt::typelevel::Interrupt;
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::Timer as LlTimer;
use embassy_stm32::timer::CoreInstance;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex as BlockingMutex;

/// ISR period; one phase of the step state machine.
const STEP_ISR_PERIOD_US: u32 = 30;
/// Idle timeout used before settings are applied (200 ms, matching the C default).
const DEFAULT_IDLE_TIMEOUT_TICKS: u32 = (200 * 1000) / STEP_ISR_PERIOD_US;

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
    /// Active-low: Low energizes, High disables.
    en_pin: Option<Output<'static>>,
    // Energization policy.
    always_energized: bool,
    idle_timeout_ticks: u32,
    energized: bool,
    idle_ticks: u32,
}

const fn motor_state_init() -> MotorState {
    MotorState {
        target: 0,
        current: 0,
        step_state: StepState::Idle,
        direction: false,
        step_pin: None,
        dir_pin: None,
        en_pin: None,
        always_energized: false,
        idle_timeout_ticks: DEFAULT_IDLE_TIMEOUT_TICKS,
        energized: false,
        idle_ticks: 0,
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

    /// Create StepGen using given timer & per-motor (step, dir, en) pins.
    /// Configures timer to 30us, but caller is responsible for calling tick() from the timer's ISR.
    /// EN pins are active-low and start de-energized; energization is driven on demand.
    pub fn init(
        &'static self,
        tim: T,
        pins: [(Output<'static>, Output<'static>, Output<'static>); N],
    ) -> [StepGenHandle<T, N>; N] {
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

    fn ensure_energized(&mut self, energize: bool) {
        if self.energized == energize {
            return;
        }
        if let Some(p) = self.en_pin.as_mut() {
            p.set_level(if energize { Level::Low } else { Level::High });
        }
        self.energized = energize;
    }

    fn process(&mut self) {
        match self.step_state {
            StepState::Idle => {
                if self.current != self.target {
                    // Energize in the same tick the step begins, before the pulse.
                    self.idle_ticks = 0;
                    self.ensure_energized(true);
                    let dir = self.target > self.current;
                    if dir != self.direction {
                        self.direction = dir;
                        self.write_dir(Level::from(dir));
                    }
                    self.write_step(Level::High);
                    self.step_state = StepState::PulseHigh;
                } else if !self.always_energized {
                    if self.idle_ticks < self.idle_timeout_ticks {
                        self.idle_ticks += 1;
                    } else {
                        self.ensure_energized(false);
                    }
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

    /// Set the idle de-energize timeout. Negative `timeout_ms` keeps the motor
    /// always energized.
    pub fn set_deenergize_after(&self, timeout_ms: i32) {
        self.engine.inner.lock(|cell| {
            let m = &mut cell.borrow_mut().motors[self.idx];
            if timeout_ms < 0 {
                m.always_energized = true;
                m.idle_timeout_ticks = 0;
            } else {
                m.always_energized = false;
                m.idle_timeout_ticks = (timeout_ms as u32 * 1000) / STEP_ISR_PERIOD_US;
            }
        });
    }
}
