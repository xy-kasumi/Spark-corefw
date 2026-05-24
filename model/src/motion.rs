// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

use crate::coords;
use crate::path;

/// EDM control thresholds and per-tick step sizes.
const EDM_OPEN_RATE_THRESH: f32 = 0.78;
const EDM_SHORT_RATE_THRESH: f32 = 0.5;
const EDM_ADVANCE_MM: f32 = 1e-3; // +1 µm/tick when too far (open)
const EDM_RETRACT_MM: f32 = -5e-3; // -5 µm/tick when too close (short)

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotionOutputs {
    /// Target axis position in machine mm to be commanded to motors this tick.
    pub target: coords::PosPhys,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotionInputs {
    /// Elapsed time since the previous tick, in seconds.
    pub dt: f32,
    /// Pulser open rate [0, 1]; high means the gap is too wide (advance).
    pub open_rate: f32,
    /// Pulser short rate [0, 1]; high means the gap is too narrow (retract).
    pub short_rate: f32,
    /// True when the pulser detects a discharge — the probe contact signal.
    pub discharge: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    Idle,
    /// Constant-velocity motion.
    Rapid,
    /// EDM-controlled move, advancing/retracting by pulser feedback.
    EdmMove,
    /// Constant-velocity move that stops by pulser feedback.
    Probing,
}

/// Snapshot of motion state backing the `?edm` query. `has_edm_data` is set
/// only during an EDM-controlled move; `is_moving` covers any active move.
#[derive(Clone, Copy, Debug, Default)]
pub struct EdmState {
    pub has_edm_data: bool,
    pub is_moving: bool,
    pub forward_buffer: f32,
    pub backward_buffer: f32,
    pub distance: f32,
    pub distance_max: f32,
}

pub struct MotionState<const N: usize> {
    mode: Mode,
    path: path::PathBuffer<N>,
    feed_mm_per_s: f32,
    /// EDM move only: stop when the path end is reached. False while more
    /// segments may still be chained (continuation).
    edm_stop_at_target: bool,
    /// Furthest net distance reached since the current move began. Reset at each
    /// move start, updated every moving tick. Reported by `?edm`.
    distance_max: f32,
}

impl<const N: usize> MotionState<N> {
    pub const fn new(start: coords::PosPhys) -> Self {
        Self {
            mode: Mode::Idle,
            path: path::PathBuffer::new(start, start),
            feed_mm_per_s: 0.0,
            edm_stop_at_target: true,
            distance_max: 0.0,
        }
    }

    /// Begin a rapid move from the current path-buffer position to `target`.
    pub fn start_rapid(&mut self, target: coords::PosPhys, feed_mm_per_s: f32) {
        let here = self.path.position();
        self.path = path::PathBuffer::new(here, target);
        self.feed_mm_per_s = feed_mm_per_s;
        self.distance_max = 0.0;
        self.mode = Mode::Rapid;
    }

    /// Begin a probe move (constant feed) toward `target`, stopping on discharge
    /// or path end.
    pub fn start_probe(&mut self, target: coords::PosPhys, feed_mm_per_s: f32) {
        let here = self.path.position();
        self.path = path::PathBuffer::new(here, target);
        self.feed_mm_per_s = feed_mm_per_s;
        self.distance_max = 0.0;
        self.mode = Mode::Probing;
    }

    /// Begin an EDM-controlled move toward `target`. When `has_cont`, the move
    /// does not stop at the path end — a following segment is expected.
    pub fn start_edm(&mut self, target: coords::PosPhys, has_cont: bool) {
        let here = self.path.position();
        self.path = path::PathBuffer::new(here, target);
        self.feed_mm_per_s = 0.0;
        self.edm_stop_at_target = !has_cont;
        self.distance_max = 0.0;
        self.mode = Mode::EdmMove;
    }

    /// Append the next EDM segment endpoint to the running move. When `has_cont`
    /// is false this is the last segment, so the move stops once it is reached.
    pub fn enqueue_edm(&mut self, target: coords::PosPhys, has_cont: bool) {
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
    pub fn set_position(&mut self, here: coords::PosPhys) {
        self.path = path::PathBuffer::new(here, here);
        self.feed_mm_per_s = 0.0;
        self.mode = Mode::Idle;
    }

    /// Abort current motion, holding at the caller-provided physical position.
    pub fn cancel(&mut self, here: coords::PosPhys) {
        self.set_position(here);
    }

    /// Advance the controller one tick.
    pub fn tick(&mut self, input: MotionInputs) -> MotionOutputs {
        match self.mode {
            Mode::Rapid => {
                self.path.move_by(self.feed_mm_per_s * input.dt);
                if self.path.at_end() {
                    self.mode = Mode::Idle;
                }
            }
            Mode::Probing => {
                if input.discharge {
                    self.mode = Mode::Idle;
                } else {
                    self.path.move_by(self.feed_mm_per_s * input.dt);
                    if self.path.at_end() {
                        self.mode = Mode::Idle;
                    }
                }
            }
            Mode::EdmMove => {
                if input.open_rate > EDM_OPEN_RATE_THRESH {
                    self.path.move_by(EDM_ADVANCE_MM);
                } else if input.short_rate > EDM_SHORT_RATE_THRESH {
                    self.path.move_by(EDM_RETRACT_MM);
                }
                if self.edm_stop_at_target && self.path.at_end() {
                    self.mode = Mode::Idle;
                }
            }
            Mode::Idle => {}
        }
        // Track the furthest distance reached while moving; only update while the
        // move is still in progress.
        if self.mode != Mode::Idle {
            self.distance_max = self.distance_max.max(self.path.distance());
        }
        MotionOutputs {
            target: self.path.position(),
        }
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Forward path distance left before the written path end. (`?edm` pb_f)
    pub fn forward_buffer(&self) -> f32 {
        self.path.forward_buffer()
    }

    /// Backward path distance left before the retraction limit. (`?edm` pb_b)
    pub fn backward_buffer(&self) -> f32 {
        self.path.backward_buffer()
    }

    /// Net distance traveled from the move's start point. (`?edm` dist)
    pub fn distance(&self) -> f32 {
        self.path.distance()
    }

    /// Furthest net distance reached since the move began. (`?edm` dist_max)
    pub fn distance_max(&self) -> f32 {
        self.distance_max
    }

    /// Snapshot for the `?edm` query.
    pub fn edm_state(&self) -> EdmState {
        EdmState {
            has_edm_data: self.mode == Mode::EdmMove,
            is_moving: self.mode != Mode::Idle,
            forward_buffer: self.forward_buffer(),
            backward_buffer: self.backward_buffer(),
            distance: self.distance(),
            distance_max: self.distance_max(),
        }
    }
}
