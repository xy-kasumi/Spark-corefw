// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

use crate::coords;
use crate::path;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EdmControlParams {
    /// Retract when `short_rate > retr_thresh`. [0, 1].
    pub retr_thresh: f32,
    /// Advance when `open_rate > adv_thresh`. [0, 1].
    pub adv_thresh: f32,
    /// Retraction speed (>0), mm/s.
    pub retr_speed: f32,
    /// Advance speed (>0), mm/s.
    pub adv_speed: f32,
}

pub const DEFAULT_CONTROL_PARAMS: EdmControlParams = EdmControlParams {
    retr_thresh: 0.5,
    adv_thresh: 0.78,
    retr_speed: 5.0,
    adv_speed: 1.0,
};

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
    pub retract_remaining: f32,
    pub distance: f32,
    pub distance_max: f32,
}

pub struct MotionState<const N: usize> {
    mode: Mode,
    path: path::PathBuffer<N>,
    feed_mm_per_s: f32,
    /// Furthest net distance reached since the current move began. Reset at each
    /// move start, updated every moving tick. Reported by `?edm`.
    distance_max: f32,
}

impl<const N: usize> MotionState<N> {
    pub fn new(start: coords::PosPhys) -> Self {
        Self {
            mode: Mode::Idle,
            path: path::PathBuffer::new(start, start),
            feed_mm_per_s: 0.0,
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

    /// Dispatch an EDM segment endpoint. Starts a fresh chain when [`Idle`], or
    /// extends the running chain when [`EdmMove`]. The caller must ensure
    /// [`ready_for_edm`](Self::ready_for_edm); calling from any other state is
    /// a contract violation.
    ///
    /// The chain ends naturally when no further segment is appended before the
    /// current one finishes; motion then transitions to [`Idle`] and the next
    /// `do_edm` resets `distance_max`.
    ///
    /// [`Idle`]: Mode::Idle
    /// [`EdmMove`]: Mode::EdmMove
    pub fn do_edm(&mut self, target: coords::PosPhys) {
        match self.mode {
            Mode::Idle => {
                let here = self.path.position();
                self.path = path::PathBuffer::new(here, target);
                self.feed_mm_per_s = 0.0;
                self.distance_max = 0.0;
                self.mode = Mode::EdmMove;
            }
            Mode::EdmMove => {
                self.path.extend(target);
            }
            _ => panic!("do_edm requires Idle or EdmMove"),
        }
    }

    /// True when [`do_edm`](Self::do_edm) can be called: either Idle (starts a
    /// fresh chain) or EdmMove with a free extension slot.
    pub fn ready_for_edm(&self) -> bool {
        match self.mode {
            Mode::Idle => true,
            Mode::EdmMove => self.path.can_extend(),
            _ => false,
        }
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
    pub fn tick(&mut self, input: MotionInputs, params: EdmControlParams) -> MotionOutputs {
        match self.mode {
            Mode::Rapid => {
                self.path.move_by(self.feed_mm_per_s * input.dt);
                if self.path.at_dst() {
                    self.mode = Mode::Idle;
                }
            }
            Mode::Probing => {
                if input.discharge {
                    self.mode = Mode::Idle;
                } else {
                    self.path.move_by(self.feed_mm_per_s * input.dt);
                    if self.path.at_dst() {
                        self.mode = Mode::Idle;
                    }
                }
            }
            Mode::EdmMove => {
                if input.open_rate > params.adv_thresh {
                    self.path.move_by(params.adv_speed * input.dt);
                } else if input.short_rate > params.retr_thresh {
                    self.path.move_by(params.retr_speed * input.dt);
                }
                if self.path.at_dst() {
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

    /// Distance available to retract before hitting the history limit. (`?edm` retr_rem)
    pub fn retract_remaining(&self) -> f32 {
        self.path.retract_remaining()
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
            retract_remaining: self.retract_remaining(),
            distance: self.distance(),
            distance_max: self.distance_max(),
        }
    }
}
