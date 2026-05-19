//! Motion state machine: passive controller pumped on each tick by the firmware.
//!
//! Phase 3 scaffolding — implements G0 (rapid) only. EDM/probe/home land in Phase 4.

use crate::coords::PosPhys;
use crate::path_buffer::{MoveError, PathBuffer};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotionOutputs {
    /// Target axis position in machine mm to be commanded to motors this tick.
    pub target: PosPhys,
    /// True once the controller reaches the end of the written path.
    pub at_end: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotionInputs {
    /// Elapsed time since the previous tick, in seconds.
    pub dt: f32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Idle,
    /// G0: open-loop linear motion at a fixed feed.
    Rapid,
}

pub struct MotionState<const N: usize> {
    mode: Mode,
    path: PathBuffer<N>,
    feed_mm_per_s: f32,
}

impl<const N: usize> MotionState<N> {
    pub const fn new(start: PosPhys) -> Self {
        Self {
            mode: Mode::Idle,
            path: PathBuffer::new(start, start),
            feed_mm_per_s: 0.0,
        }
    }

    /// Begin a rapid move from the current path-buffer position to `target`.
    pub fn start_rapid(&mut self, target: PosPhys, feed_mm_per_s: f32) {
        let here = self.path.position();
        self.path = PathBuffer::new(here, target);
        self.feed_mm_per_s = feed_mm_per_s;
        self.mode = Mode::Rapid;
    }

    /// Abort current motion. Resets the path to a degenerate segment anchored at
    /// the caller-provided physical position, so subsequent ticks hold motors there.
    pub fn cancel(&mut self, here: PosPhys) {
        self.path = PathBuffer::new(here, here);
        self.feed_mm_per_s = 0.0;
        self.mode = Mode::Idle;
    }

    /// Advance the controller one tick.
    pub fn tick(&mut self, input: MotionInputs) -> Result<MotionOutputs, MoveError> {
        if self.mode == Mode::Rapid {
            let advance = self.feed_mm_per_s * input.dt;
            self.path.move_by(advance)?;
            if self.path.at_end() {
                self.mode = Mode::Idle;
            }
        }
        Ok(MotionOutputs {
            target: self.path.position(),
            at_end: self.path.at_end(),
        })
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }
}
