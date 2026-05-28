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
    /// cursor pos = notch pos + cursor_offset. subnotch is in (-RESOLUTON_MM, RESOLUTION_MM).
    cursor_offset: f32,

    /// discretized history + current pos in forward buffer.
    forward: SegmentBuf,
    history: heapless::HistoryBuffer<coords::PosPhys, N>,
    /// 0=traversed frontier.
    notches_retract: usize,

    cum_notches: usize,
}

impl<const N: usize> PathBuffer<N> {
    pub fn new(src: coords::PosPhys, dst: coords::PosPhys) -> Self {
        let mut history = heapless::HistoryBuffer::new();
        history.write(src);
        Self {
            cursor_offset: 0.0,
            forward: SegmentBuf::new(src, dst),
            history,
            notches_retract: 0,
            cum_notches: 0,
        }
    }

    /// Current (notch-aligned) cursor position.
    pub fn position(&self) -> coords::PosPhys {
        self.back(self.notches_retract as usize)
    }

    /// Returns true if cursor is at the destination.
    pub fn at_dst(&self) -> bool {
        if self.notches_retract > 0 {
            return false;
        }
        self.back(0).distance_to(&self.forward.final_dst()) <= RESOLUTION_MM
    }

    /// Returns whether [`extend`](Self::extend) can be called.
    pub fn can_extend(&self) -> bool {
        self.forward.can_extend()
    }

    /// Extend the path by one more line segment.
    /// [`can_extend`](Self::can_extend) must be true.
    pub fn extend(&mut self, dst: coords::PosPhys) {
        assert!(self.forward.can_extend(), "next segment already queued");
        self.forward.extend(dst);
    }

    /// Distance available to retract before hitting the limit.
    pub fn retract_remaining(&self) -> f32 {
        (self.history.len() - 1 - self.notches_retract) as f32 * RESOLUTION_MM
    }

    /// Cursor distance from init point along the path.
    pub fn distance(&self) -> f32 {
        self.cum_notches as f32 * RESOLUTION_MM + self.cursor_offset
    }

    /// Advances by `d` mm. Negative `d` retracts. Over-runs in either
    /// direction clip silently to the limit; observe via [`at_end`](Self::at_end)
    /// or [`retract_remaining`](Self::retract_remaining).
    /// Takes O(|d|/RESOLUTION_MM) time in worst case.
    pub fn move_by(&mut self, d: f32) {
        self.cursor_offset += d;
        let d_notches = libm::truncf(self.cursor_offset * (1.0 / RESOLUTION_MM)) as i32;
        if d_notches == 0 {
            return;
        }
        self.cursor_offset -= d_notches as f32 * RESOLUTION_MM;

        if d_notches < 0 {
            self.cum_notches -= self.retract_by_notches((-d_notches) as usize);
        } else {
            self.cum_notches += self.advance_by_notches(d_notches as usize);
        }
    }

    /// Walk the cursor back by up to `n` notches, clipped at the retract limit.
    /// Returns the count actually retracted.
    fn retract_by_notches(&mut self, n: usize) -> usize {
        let available = self.history.len() - self.notches_retract - 1;
        let actual = n.min(available);
        self.notches_retract += actual;
        actual
    }

    /// Walk the cursor forward by up to `n`` notches.
    /// Consume retract history first, then drive the segment generator.
    /// Returns the count actually advanced (may be less than `n` if the path ends).
    fn advance_by_notches(&mut self, n: usize) -> usize {
        let n_history = n.min(self.notches_retract);
        self.notches_retract -= n_history;

        let mut advanced_by_fwd = 0;
        for _ in 0..(n - n_history) {
            match self.forward.try_step(RESOLUTION_MM) {
                Some(pos) => {
                    self.history.write(pos);
                    advanced_by_fwd += 1;
                }
                None => break,
            }
        }
        n_history + advanced_by_fwd
    }

    /// `n`-th most recent history entry (n=0 is the latest write).
    fn back(&self, n: usize) -> coords::PosPhys {
        let (older, newer) = self.history.as_slices();
        debug_assert!(n < older.len() + newer.len());
        if n < newer.len() {
            newer[newer.len() - 1 - n]
        } else {
            older[older.len() - 1 - (n - newer.len())]
        }
    }
}

/// Represents 1 (src-dst) or 2 (src-dst-next) line segments, and cursor on it.
struct SegmentBuf {
    src: coords::PosPhys,
    dst: coords::PosPhys,
    next: Option<coords::PosPhys>,
    /// Distance from src in src-dst segment. [0, |dst-src|].
    cursor: f32,
}

impl SegmentBuf {
    fn new(src: coords::PosPhys, dst: coords::PosPhys) -> Self {
        Self {
            src,
            dst,
            next: None,
            cursor: 0.0,
        }
    }

    fn can_extend(&self) -> bool {
        self.next.is_none()
    }

    fn extend(&mut self, dst: coords::PosPhys) {
        debug_assert!(self.next.is_none());
        self.next = Some(dst);
    }

    /// Ultimate path endpoint.
    fn final_dst(&self) -> coords::PosPhys {
        self.next.unwrap_or(self.dst)
    }

    /// Advance cursor by exactly `d` (>=0) if possible.
    /// Returns new position, or None if not possible (won't fit in the segment).
    fn try_step(&mut self, d: f32) -> Option<coords::PosPhys> {
        let seg_len = self.src.distance_to(&self.dst) - self.cursor;
        if d <= seg_len {
            self.cursor += d;
            return Some(self.pos());
        }
        let next = self.next?;
        let next_d = d - seg_len;
        let next_seg_len = self.dst.distance_to(&next);
        if next_d <= next_seg_len {
            (self.src, self.dst, self.next) = (self.dst, next, None);
            self.cursor = next_d;
            return Some(self.pos());
        }
        None
    }

    /// Current cursor position.
    fn pos(&self) -> coords::PosPhys {
        let seg_len = self.src.distance_to(&self.dst);
        // avoid divide-by-zero (caused by tiny line segment)
        if seg_len > 0.0 {
            self.src.interp(&self.dst, self.cursor / seg_len)
        } else {
            self.src
        }
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    /// History size for the tests.
    const N: usize = 201;
    const RETRACT_LIMIT: f32 = ((N - 1) as f32) * RESOLUTION_MM;

    fn p3(x: f32, y: f32, z: f32) -> coords::PosPhys {
        coords::PosPhys { x, y, z, c: 0.0 }
    }

    #[track_caller]
    fn assert_within(observed: f32, expected: f32, tol: f32) {
        let diff = (observed - expected).abs();
        assert!(
            diff < tol,
            "expected={expected} observed={observed} tol={tol} diff={diff}"
        );
    }

    /// Position tolerance: one resolution step plus a small epsilon.
    const POS_TOL: f32 = RESOLUTION_MM + 1e-4;

    #[test]
    fn pb_init_basic() {
        let pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(10.0, 0.0, 0.0));
        assert_within(pb.position().x, 0.0, 1e-4);
        assert!(pb.can_extend(), "buffer available after construction");
        assert!(!pb.at_dst(), "initial pos is not end");
    }

    #[test]
    fn pb_move_forward_simple() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.move_by(0.5);
        assert_within(pb.position().x, 0.5, POS_TOL);
    }

    #[test]
    fn pb_move_backward() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.move_by(0.5);
        pb.move_by(-0.2);
        assert_within(pb.position().x, 0.3, POS_TOL);
    }

    #[test]
    fn pb_move_retraction_limit() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(10.0, 0.0, 0.0));
        pb.move_by(5.0);
        pb.move_by(-10.0);
        assert_within(pb.position().x, 5.0 - RETRACT_LIMIT, POS_TOL);
    }

    #[test]
    fn pb_move_to_end() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(0.5, 0.0, 0.0));
        pb.move_by(1.0);
        assert!(pb.at_dst(), "should be at end after overshooting");
        assert_within(pb.position().x, 0.5, POS_TOL);
    }

    #[test]
    fn pb_write_and_traverse() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        assert!(pb.can_extend());
        pb.extend(p3(1.0, 1.0, 0.0));
        pb.move_by(1.5);
        let pos = pb.position();
        assert_within(pos.x, 1.0, POS_TOL);
        assert_within(pos.y, 0.5, POS_TOL);
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
        assert_within(pos.x, 1.0, POS_TOL);
        assert_within(pos.y, 0.0, POS_TOL);
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
        assert_within(after.x, before.x, 1e-4);
    }

    #[test]
    fn pb_zero_length_segment() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(5.0, 5.0, 5.0), p3(5.0, 5.0, 5.0));
        pb.move_by(1.0);
        assert!(pb.at_dst(), "zero-length segment should be at end");
        assert_within(pb.position().x, 5.0, 1e-4);
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
    fn pb_retract_remaining() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(RETRACT_LIMIT, 0.0, 0.0));
        assert_within(pb.retract_remaining(), 0.0, RESOLUTION_MM);
        pb.move_by(RETRACT_LIMIT * 0.25);
        assert_within(pb.retract_remaining(), RETRACT_LIMIT * 0.25, RESOLUTION_MM);
        pb.move_by(RETRACT_LIMIT * 0.75);
        assert_within(pb.retract_remaining(), RETRACT_LIMIT * 1.0, RESOLUTION_MM);
    }

    #[test]
    fn pb_get_distance() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(RETRACT_LIMIT, 0.0, 0.0));
        assert_within(pb.distance(), 0.0, RESOLUTION_MM);
        pb.move_by(RETRACT_LIMIT * 0.5);
        assert_within(pb.distance(), RETRACT_LIMIT * 0.5, RESOLUTION_MM);
        pb.move_by(RETRACT_LIMIT * -0.25);
        assert_within(pb.distance(), RETRACT_LIMIT * 0.25, RESOLUTION_MM);
        pb.extend(p3(RETRACT_LIMIT, RETRACT_LIMIT, 0.0));
        pb.move_by(RETRACT_LIMIT * 1.0);
        assert_within(pb.distance(), RETRACT_LIMIT * 1.25, RESOLUTION_MM);
        pb.move_by(RETRACT_LIMIT * 10.0); // clipped at end
        assert_within(pb.distance(), RETRACT_LIMIT * 2.0, RESOLUTION_MM);
        pb.move_by(RETRACT_LIMIT * -20.0); // hits retract limit
        assert_within(pb.distance(), RETRACT_LIMIT, RESOLUTION_MM);
    }
}
