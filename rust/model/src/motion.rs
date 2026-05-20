//! Motion state machine: passive controller pumped on each tick by the firmware.
//!
//! Implements G0 (rapid), G1 (EDM control), and G38.3 (probe). Homing reuses the
//! rapid primitive; its position re-anchor lives in the firmware controller.

use crate::coords::PosPhys;
use crate::path_buffer::{MoveError, PathBuffer};

/// EDM control thresholds and per-tick step sizes (mirror C `motion_tick_handler`).
const EDM_OPEN_RATE_THRESH: u8 = 200;
const EDM_SHORT_RATE_THRESH: u8 = 127;
const EDM_ADVANCE_MM: f32 = 1e-3; // +1 µm/tick when too far (open)
const EDM_RETRACT_MM: f32 = -5e-3; // -5 µm/tick when too close (short)

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
    /// Pulser open rate (0-255); high means the gap is too wide (advance).
    pub open_rate: u8,
    /// Pulser short rate (0-255); high means the gap is too narrow (retract).
    pub short_rate: u8,
    /// True when the pulser detects a discharge — the probe contact signal.
    pub discharge: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Idle,
    /// G0: open-loop linear motion at a fixed feed.
    Rapid,
    /// G1: EDM-controlled move, advancing/retracting by pulser feedback.
    EdmMove,
    /// G38.3: constant-feed move that stops on discharge or target.
    Probing,
}

pub struct MotionState<const N: usize> {
    mode: Mode,
    path: PathBuffer<N>,
    feed_mm_per_s: f32,
    /// EDM move only: stop when the path end is reached. False while more
    /// segments may still be chained (continuation).
    edm_stop_at_target: bool,
}

impl<const N: usize> MotionState<N> {
    pub const fn new(start: PosPhys) -> Self {
        Self {
            mode: Mode::Idle,
            path: PathBuffer::new(start, start),
            feed_mm_per_s: 0.0,
            edm_stop_at_target: true,
        }
    }

    /// Begin a rapid move from the current path-buffer position to `target`.
    pub fn start_rapid(&mut self, target: PosPhys, feed_mm_per_s: f32) {
        let here = self.path.position();
        self.path = PathBuffer::new(here, target);
        self.feed_mm_per_s = feed_mm_per_s;
        self.mode = Mode::Rapid;
    }

    /// Begin a probe move (constant feed) toward `target`, stopping on discharge
    /// or path end.
    pub fn start_probe(&mut self, target: PosPhys, feed_mm_per_s: f32) {
        let here = self.path.position();
        self.path = PathBuffer::new(here, target);
        self.feed_mm_per_s = feed_mm_per_s;
        self.mode = Mode::Probing;
    }

    /// Begin an EDM-controlled move toward `target`. When `has_cont`, the move
    /// does not stop at the path end — a following segment is expected.
    pub fn start_edm(&mut self, target: PosPhys, has_cont: bool) {
        let here = self.path.position();
        self.path = PathBuffer::new(here, target);
        self.feed_mm_per_s = 0.0;
        self.edm_stop_at_target = !has_cont;
        self.mode = Mode::EdmMove;
    }

    /// Append the next EDM segment endpoint to the running move. When `has_cont`
    /// is false this is the last segment, so the move stops once it is reached.
    pub fn enqueue_edm(&mut self, target: PosPhys, has_cont: bool) {
        self.path.extend(target);
        if !has_cont {
            self.edm_stop_at_target = true;
        }
    }

    /// True when a further EDM segment can be appended without overwriting one
    /// already queued.
    pub fn can_enqueue(&self) -> bool {
        self.path.can_extend()
    }

    /// Reset the controller to hold at `here` (degenerate path, Idle). Used for
    /// cancel and for the homing position re-anchor.
    pub fn set_position(&mut self, here: PosPhys) {
        self.path = PathBuffer::new(here, here);
        self.feed_mm_per_s = 0.0;
        self.mode = Mode::Idle;
    }

    /// Abort current motion, holding at the caller-provided physical position.
    pub fn cancel(&mut self, here: PosPhys) {
        self.set_position(here);
    }

    /// Advance the controller one tick.
    pub fn tick(&mut self, input: MotionInputs) -> Result<MotionOutputs, MoveError> {
        match self.mode {
            Mode::Rapid => {
                self.path.move_by(self.feed_mm_per_s * input.dt)?;
                if self.path.at_end() {
                    self.mode = Mode::Idle;
                }
            }
            Mode::Probing => {
                if input.discharge {
                    self.mode = Mode::Idle;
                } else {
                    self.path.move_by(self.feed_mm_per_s * input.dt)?;
                    if self.path.at_end() {
                        self.mode = Mode::Idle;
                    }
                }
            }
            Mode::EdmMove => {
                // Retraction can hit the history limit; clamp and continue (as C does).
                if input.open_rate > EDM_OPEN_RATE_THRESH {
                    let _ = self.path.move_by(EDM_ADVANCE_MM);
                } else if input.short_rate > EDM_SHORT_RATE_THRESH {
                    let _ = self.path.move_by(EDM_RETRACT_MM);
                }
                if self.edm_stop_at_target && self.path.at_end() {
                    self.mode = Mode::Idle;
                }
            }
            Mode::Idle => {}
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
