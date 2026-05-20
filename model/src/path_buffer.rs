// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

use crate::coords::PosPhys;

/// Positional resolution of EDM control in mm. Path positions are notch-aligned.
pub const RESOLUTION_MM: f32 = 0.005;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MoveError {
    /// Retraction limit hit. Position is clipped to the furthest retract point.
    RetractLimitExceeded,
}

/// Streamable line-segment path with retractable current position, notch-aligned at
/// `RESOLUTION_MM`.
///
/// `N` is the history capacity (in notches). Maximum retraction distance is
/// `(N - 1) * RESOLUTION_MM`.
///
/// The path is extended via [`extend`](Self::extend); the cursor is moved via
/// [`move_by`](Self::move_by). The cursor cannot escape the written path or the
/// retraction limit.
pub struct PathBuffer<const N: usize> {
    pos_history: [PosPhys; N],
    ix_history: usize,
    num_history: usize,

    notches_retract: i32,

    curr_seg_d: f32,
    curr_seg_src: PosPhys,
    curr_seg_dst: PosPhys,

    next_seg_avail: bool,
    next_pos: PosPhys,

    fraction: f32,

    cum_notches: i32,
}

impl<const N: usize> PathBuffer<N> {
    pub const fn new(src: PosPhys, dst: PosPhys) -> Self {
        Self {
            pos_history: [src; N],
            ix_history: 0,
            num_history: 1,
            notches_retract: 0,
            curr_seg_d: 0.0,
            curr_seg_src: src,
            curr_seg_dst: dst,
            next_seg_avail: false,
            next_pos: src,
            fraction: 0.0,
            cum_notches: 0,
        }
    }

    /// Current (notch-aligned) position.
    pub fn position(&self) -> PosPhys {
        let ix = (self.ix_history + N - self.notches_retract as usize) % N;
        self.pos_history[ix]
    }

    /// True iff the furthest-reached position is at the end of the written path.
    /// Returns false while retracted, even if the furthest point was the end.
    pub fn at_end(&self) -> bool {
        if self.notches_retract > 0 {
            return false;
        }
        let curr = self.pos_history[self.ix_history];
        let end = if self.next_seg_avail {
            self.next_pos
        } else {
            self.curr_seg_dst
        };
        curr.distance_to(&end) <= RESOLUTION_MM
    }

    /// True if [`extend`](Self::extend) can be called without overwriting the queued segment.
    pub fn can_extend(&self) -> bool {
        !self.next_seg_avail
    }

    /// Append the next path segment endpoint. If called when not [`can_extend`](Self::can_extend),
    /// the previously queued endpoint is overwritten.
    pub fn extend(&mut self, next: PosPhys) {
        self.next_pos = next;
        self.next_seg_avail = true;
    }

    /// Move the cursor by `d` mm. Negative `d` retracts.
    ///
    /// Sub-notch motion accumulates internally. Forward motion past the written path is
    /// clipped silently. Retraction past the history limit returns
    /// [`MoveError::RetractLimitExceeded`] with the cursor clipped to the furthest retractable
    /// point.
    pub fn move_by(&mut self, d: f32) -> Result<(), MoveError> {
        self.fraction += d;
        let mut d_notches = libm::truncf(self.fraction * (1.0 / RESOLUTION_MM)) as i32;
        if d_notches == 0 {
            return Ok(());
        }
        self.fraction -= d_notches as f32 * RESOLUTION_MM;

        if d_notches < 0 {
            let available = self.num_history as i32 - self.notches_retract - 1;
            if d_notches < -available {
                self.notches_retract += available;
                self.cum_notches -= available;
                return Err(MoveError::RetractLimitExceeded);
            } else {
                self.notches_retract += -d_notches;
                self.cum_notches -= -d_notches;
                return Ok(());
            }
        } else if self.notches_retract > d_notches {
            self.notches_retract -= d_notches;
            self.cum_notches += d_notches;
            return Ok(());
        } else {
            d_notches -= self.notches_retract;
            self.cum_notches += self.notches_retract;
            self.notches_retract = 0;
        }

        // d_notches > 0: advance forward along the path.
        for _ in 0..d_notches {
            let mut clipped = false;
            let seg_len = self.curr_seg_src.distance_to(&self.curr_seg_dst);

            self.curr_seg_d += RESOLUTION_MM;
            if self.curr_seg_d >= seg_len {
                if !self.next_seg_avail {
                    self.curr_seg_d = seg_len;
                    clipped = true;
                } else {
                    self.curr_seg_d -= seg_len;
                    self.curr_seg_src = self.curr_seg_dst;
                    self.curr_seg_dst = self.next_pos;
                    self.next_seg_avail = false;
                }
            }

            let pos = if seg_len < RESOLUTION_MM {
                self.curr_seg_src
            } else {
                self.curr_seg_src
                    .interp(&self.curr_seg_dst, self.curr_seg_d / seg_len)
            };
            self.push_history(pos);
            self.cum_notches += 1;

            if clipped {
                break;
            }
        }
        Ok(())
    }

    fn push_history(&mut self, pos: PosPhys) {
        self.ix_history = (self.ix_history + 1) % N;
        self.pos_history[self.ix_history] = pos;
        if self.num_history < N {
            self.num_history += 1;
        }
    }

    /// Forward distance available before hitting the end of the written path.
    pub fn forward_buffer(&self) -> f32 {
        let mut d_segs_in_buf = self.curr_seg_src.distance_to(&self.curr_seg_dst);
        if self.next_seg_avail {
            d_segs_in_buf += self.curr_seg_dst.distance_to(&self.next_pos);
        }
        if self.notches_retract == 0 {
            d_segs_in_buf - self.curr_seg_d
        } else {
            d_segs_in_buf + self.notches_retract as f32 * RESOLUTION_MM
        }
    }

    /// Backward distance available before hitting the retraction limit.
    pub fn backward_buffer(&self) -> f32 {
        ((self.num_history as i32 - 1) - self.notches_retract) as f32 * RESOLUTION_MM
    }

    /// Net cumulative distance traveled from init point (forward minus backward).
    pub fn distance(&self) -> f32 {
        self.cum_notches as f32 * RESOLUTION_MM + self.fraction
    }
}

#[cfg(test)]
mod tests {
    //! Test `N` (201) is a deliberately small history cap; production firmware uses a
    //! larger one. The history-cap-dependent tests (`pb_get_buffers_added`,
    //! `pb_get_distance`) only make sense at this smaller size.

    use super::*;

    /// History size matching the era of the C tests.
    const N: usize = 201;

    fn p3(x: f32, y: f32, z: f32) -> PosPhys {
        PosPhys { x, y, z, c: 0.0 }
    }

    fn within(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    /// Position tolerance matches the C `EDM_RESOLUTION_MM + 1e-4f` convention.
    const POS_TOL: f32 = RESOLUTION_MM + 1e-4;

    #[test]
    fn pb_init_basic() {
        let pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(10.0, 0.0, 0.0));
        assert!(within(pb.position().x, 0.0, 1e-4));
        assert!(pb.can_extend(), "buffer available after construction");
        assert!(!pb.at_end(), "initial pos is not end");
    }

    #[test]
    fn pb_move_forward_simple() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.move_by(0.5).unwrap();
        assert!(within(pb.position().x, 0.5, POS_TOL));
    }

    #[test]
    fn pb_move_backward() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.move_by(0.5).unwrap();
        pb.move_by(-0.2).unwrap();
        assert!(within(pb.position().x, 0.3, POS_TOL));
    }

    #[test]
    fn pb_move_retraction_limit() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(10.0, 0.0, 0.0));
        pb.move_by(5.0).unwrap();
        assert!(
            pb.move_by(-10.0).is_err(),
            "retraction beyond history limit should fail"
        );
    }

    #[test]
    fn pb_move_to_end() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(0.5, 0.0, 0.0));
        pb.move_by(1.0).unwrap();
        assert!(pb.at_end(), "should be at end after overshooting");
        assert!(within(pb.position().x, 0.5, POS_TOL));
    }

    #[test]
    fn pb_write_and_traverse() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        assert!(pb.can_extend());
        pb.extend(p3(1.0, 1.0, 0.0));
        pb.move_by(1.5).unwrap();
        let pos = pb.position();
        assert!(within(pos.x, 1.0, POS_TOL));
        assert!(within(pos.y, 0.5, POS_TOL));
        assert!(!pb.at_end(), "not ended yet (1.5 of 2.0)");
        pb.move_by(0.5).unwrap();
        assert!(pb.at_end(), "must be ended (2.0 of 2.0)");
    }

    #[test]
    fn pb_write_check_middle() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        assert!(pb.can_extend());
        pb.extend(p3(1.0, 1.0, 0.0));
        pb.move_by(1.0).unwrap();
        let pos = pb.position();
        assert!(within(pos.x, 1.0, POS_TOL));
        assert!(within(pos.y, 0.0, POS_TOL));
        assert!(!pb.at_end(), "not ended yet (1.0 of 2.0)");
    }

    #[test]
    fn pb_write_buffer_full() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.extend(p3(2.0, 0.0, 0.0));
        assert!(!pb.can_extend(), "buffer should be full");
        pb.move_by(1.1).unwrap();
        assert!(pb.can_extend(), "first segment must be consumed");
    }

    #[test]
    fn pb_tiny_movements() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        let before = pb.position();
        pb.move_by(RESOLUTION_MM * 0.5).unwrap();
        let after = pb.position();
        assert!(within(before.x, after.x, 1e-4));
    }

    #[test]
    fn pb_zero_length_segment() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(5.0, 5.0, 5.0), p3(5.0, 5.0, 5.0));
        pb.move_by(1.0).unwrap();
        assert!(pb.at_end(), "zero-length segment should be at end");
        assert!(within(pb.position().x, 5.0, 1e-4));
    }

    #[test]
    fn pb_accumulated_tiny_movements() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        let tiny = RESOLUTION_MM * 0.3;
        pb.move_by(tiny).unwrap();
        pb.move_by(tiny).unwrap();
        pb.move_by(tiny).unwrap();
        pb.move_by(tiny).unwrap();
        assert!(pb.position().x >= RESOLUTION_MM - 1e-4);
    }

    #[test]
    fn pb_get_buffers() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        assert!(within(pb.forward_buffer(), 1.0, RESOLUTION_MM));
        assert!(within(pb.backward_buffer(), 0.0, RESOLUTION_MM));
        pb.move_by(0.25).unwrap();
        assert!(within(pb.forward_buffer(), 0.75, RESOLUTION_MM));
        assert!(within(pb.backward_buffer(), 0.25, RESOLUTION_MM));
        pb.move_by(0.75).unwrap();
        assert!(within(pb.forward_buffer(), 0.0, RESOLUTION_MM));
        assert!(within(pb.backward_buffer(), 1.0, RESOLUTION_MM));
    }

    #[test]
    fn pb_get_buffers_added() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.extend(p3(1.0, 1.0, 0.0));
        assert!(within(pb.forward_buffer(), 2.0, RESOLUTION_MM));
        assert!(within(pb.backward_buffer(), 0.0, RESOLUTION_MM));
        pb.move_by(1.5).unwrap();
        assert!(within(pb.forward_buffer(), 0.5, RESOLUTION_MM));
        // History capped at N - 1 = 200 notches = 1.0 mm.
        assert!(within(
            pb.backward_buffer(),
            (N as f32 - 1.0) * RESOLUTION_MM,
            RESOLUTION_MM
        ));
    }

    #[test]
    fn pb_get_distance() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        assert!(within(pb.distance(), 0.0, RESOLUTION_MM));
        pb.move_by(0.5).unwrap();
        assert!(within(pb.distance(), 0.5, RESOLUTION_MM));
        pb.move_by(-0.25).unwrap();
        assert!(within(pb.distance(), 0.25, RESOLUTION_MM));
        pb.extend(p3(1.0, 1.0, 0.0));
        pb.move_by(1.0).unwrap();
        assert!(within(pb.distance(), 1.25, RESOLUTION_MM));
        let _ = pb.move_by(100.0); // clipped at end
        assert!(within(pb.distance(), 2.0, RESOLUTION_MM));
        let _ = pb.move_by(-50.0); // hits retract limit
        assert!(within(
            pb.distance(),
            2.0 - (N as f32 - 1.0) * RESOLUTION_MM,
            RESOLUTION_MM
        ));
    }
}
