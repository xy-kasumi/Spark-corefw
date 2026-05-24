// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

use crate::coords;

/// Positional resolution of EDM control in mm. Path positions are notch-aligned.
pub const RESOLUTION_MM: f32 = 0.005;

/// Streamable line-segment path with retractable current position, notch-aligned at
/// `RESOLUTION_MM`.
///
/// `N` is the retract buffer capacity (in notches). Maximum retraction distance is
/// `(N - 1) * RESOLUTION_MM`.
pub struct PathBuffer<const N: usize> {
    pos_history: [coords::PosPhys; N],
    ix_history: usize,
    num_history: usize,

    notches_retract: i32,

    curr_seg_d: f32,
    curr_seg_src: coords::PosPhys,
    curr_seg_dst: coords::PosPhys,

    next_pos: Option<coords::PosPhys>,

    fraction: f32,

    cum_notches: i32,
}

impl<const N: usize> PathBuffer<N> {
    pub const fn new(src: coords::PosPhys, dst: coords::PosPhys) -> Self {
        Self {
            pos_history: [src; N],
            ix_history: 0,
            num_history: 1,
            notches_retract: 0,
            curr_seg_d: 0.0,
            curr_seg_src: src,
            curr_seg_dst: dst,
            next_pos: None,
            fraction: 0.0,
            cum_notches: 0,
        }
    }

    /// Current (notch-aligned) cursor position.
    pub fn position(&self) -> coords::PosPhys {
        let ix = (self.ix_history + N - self.notches_retract as usize) % N;
        self.pos_history[ix]
    }

    /// Returns true if cursor is at the destination.
    pub fn at_dst(&self) -> bool {
        if self.notches_retract > 0 {
            return false;
        }
        let curr = self.pos_history[self.ix_history];
        let end = self.next_pos.unwrap_or(self.curr_seg_dst);
        curr.distance_to(&end) <= RESOLUTION_MM
    }

    /// Returns whether [`extend`](Self::extend) can be called.
    pub fn can_extend(&self) -> bool {
        self.next_pos.is_none()
    }

    /// Extend the path by one more line segment.
    /// [`can_extend`](Self::can_extend) must be true.
    pub fn extend(&mut self, dst: coords::PosPhys) {
        assert!(self.next_pos.is_none(), "next segment already queued");
        self.next_pos = Some(dst);
    }

    /// Distance available to retract before hitting the limit.
    pub fn retract_remaining(&self) -> f32 {
        ((self.num_history as i32 - 1) - self.notches_retract) as f32 * RESOLUTION_MM
    }

    /// Cursor distance from init point along the path.
    pub fn distance(&self) -> f32 {
        self.cum_notches as f32 * RESOLUTION_MM + self.fraction
    }

    /// Advances by `d` mm. Negative `d` retracts. Over-runs in either
    /// direction clip silently to the limit; observe via [`at_end`](Self::at_end)
    /// or [`retract_remaining`](Self::retract_remaining).
    pub fn move_by(&mut self, d: f32) {
        self.fraction += d;
        let mut d_notches = libm::truncf(self.fraction * (1.0 / RESOLUTION_MM)) as i32;
        if d_notches == 0 {
            return;
        }
        self.fraction -= d_notches as f32 * RESOLUTION_MM;

        if d_notches < 0 {
            let want_retract = -d_notches;
            let available = self.num_history as i32 - self.notches_retract - 1;
            let actual_retract = want_retract.min(available);
            self.notches_retract += actual_retract;
            self.cum_notches -= actual_retract;
            return;
        }

        // d_notches > 0: consume any retract debt first, then advance along the path.
        let consume_retract = d_notches.min(self.notches_retract);
        self.notches_retract -= consume_retract;
        self.cum_notches += consume_retract;
        d_notches -= consume_retract;

        for _ in 0..d_notches {
            let seg_len = self.curr_seg_src.distance_to(&self.curr_seg_dst);
            self.curr_seg_d += RESOLUTION_MM;

            let clipped = if self.curr_seg_d >= seg_len {
                match self.next_pos.take() {
                    Some(next) => {
                        self.curr_seg_d -= seg_len;
                        self.curr_seg_src = self.curr_seg_dst;
                        self.curr_seg_dst = next;
                        false
                    }
                    None => {
                        self.curr_seg_d = seg_len;
                        true
                    }
                }
            } else {
                false
            };

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
    }

    fn push_history(&mut self, pos: coords::PosPhys) {
        self.ix_history = (self.ix_history + 1) % N;
        self.pos_history[self.ix_history] = pos;
        if self.num_history < N {
            self.num_history += 1;
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    /// History size for the tests below (see module note).
    const N: usize = 201;
    const RETRACT_LIMIT: f32 = ((N - 1) as f32) * RESOLUTION_MM;

    fn p3(x: f32, y: f32, z: f32) -> coords::PosPhys {
        coords::PosPhys { x, y, z, c: 0.0 }
    }

    fn within(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    /// Position tolerance: one resolution step plus a small epsilon.
    const POS_TOL: f32 = RESOLUTION_MM + 1e-4;

    #[test]
    fn pb_init_basic() {
        let pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(10.0, 0.0, 0.0));
        assert!(within(pb.position().x, 0.0, 1e-4));
        assert!(pb.can_extend(), "buffer available after construction");
        assert!(!pb.at_dst(), "initial pos is not end");
    }

    #[test]
    fn pb_move_forward_simple() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.move_by(0.5);
        assert!(within(pb.position().x, 0.5, POS_TOL));
    }

    #[test]
    fn pb_move_backward() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.move_by(0.5);
        pb.move_by(-0.2);
        assert!(within(pb.position().x, 0.3, POS_TOL));
    }

    #[test]
    fn pb_move_retraction_limit() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(10.0, 0.0, 0.0));
        pb.move_by(5.0);
        pb.move_by(-10.0);
        assert!(within(pb.position().x, 5.0 - RETRACT_LIMIT, POS_TOL));
    }

    #[test]
    fn pb_move_to_end() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(0.5, 0.0, 0.0));
        pb.move_by(1.0);
        assert!(pb.at_dst(), "should be at end after overshooting");
        assert!(within(pb.position().x, 0.5, POS_TOL));
    }

    #[test]
    fn pb_write_and_traverse() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        assert!(pb.can_extend());
        pb.extend(p3(1.0, 1.0, 0.0));
        pb.move_by(1.5);
        let pos = pb.position();
        assert!(within(pos.x, 1.0, POS_TOL));
        assert!(within(pos.y, 0.5, POS_TOL));
        assert!(!pb.at_dst(), "not ended yet (1.5 of 2.0)");
        pb.move_by(0.5);
        assert!(pb.at_dst(), "must be ended (2.0 of 2.0)");
    }

    #[test]
    fn pb_write_check_middle() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        assert!(pb.can_extend());
        pb.extend(p3(1.0, 1.0, 0.0));
        pb.move_by(1.0);
        let pos = pb.position();
        assert!(within(pos.x, 1.0, POS_TOL));
        assert!(within(pos.y, 0.0, POS_TOL));
        assert!(!pb.at_dst(), "not ended yet (1.0 of 2.0)");
    }

    #[test]
    fn pb_write_buffer_full() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.extend(p3(2.0, 0.0, 0.0));
        assert!(!pb.can_extend(), "buffer should be full");
        pb.move_by(1.1);
        assert!(pb.can_extend(), "first segment must be consumed");
    }

    #[test]
    fn pb_tiny_movements() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        let before = pb.position();
        pb.move_by(RESOLUTION_MM * 0.5);
        let after = pb.position();
        assert!(within(before.x, after.x, 1e-4));
    }

    #[test]
    fn pb_zero_length_segment() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(5.0, 5.0, 5.0), p3(5.0, 5.0, 5.0));
        pb.move_by(1.0);
        assert!(pb.at_dst(), "zero-length segment should be at end");
        assert!(within(pb.position().x, 5.0, 1e-4));
    }

    #[test]
    fn pb_accumulated_tiny_movements() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        let tiny = RESOLUTION_MM * 0.3;
        pb.move_by(tiny);
        pb.move_by(tiny);
        pb.move_by(tiny);
        pb.move_by(tiny);
        assert!(pb.position().x >= RESOLUTION_MM - 1e-4);
    }

    #[test]
    fn pb_get_buffer() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        assert!(within(pb.retract_remaining(), 0.0, RESOLUTION_MM));
        pb.move_by(0.25);
        assert!(within(pb.retract_remaining(), 0.25, RESOLUTION_MM));
        pb.move_by(0.75);
        assert!(within(pb.retract_remaining(), 1.0, RESOLUTION_MM));
    }

    #[test]
    fn pb_get_distance() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        assert!(within(pb.distance(), 0.0, RESOLUTION_MM));
        pb.move_by(0.5);
        assert!(within(pb.distance(), 0.5, RESOLUTION_MM));
        pb.move_by(-0.25);
        assert!(within(pb.distance(), 0.25, RESOLUTION_MM));
        pb.extend(p3(1.0, 1.0, 0.0));
        pb.move_by(1.0);
        assert!(within(pb.distance(), 1.25, RESOLUTION_MM));
        pb.move_by(100.0); // clipped at end
        assert!(within(pb.distance(), 2.0, RESOLUTION_MM));
        pb.move_by(-50.0); // hits retract limit
        assert!(within(
            pb.distance(),
            2.0 - (N as f32 - 1.0) * RESOLUTION_MM,
            RESOLUTION_MM
        ));
    }
}
