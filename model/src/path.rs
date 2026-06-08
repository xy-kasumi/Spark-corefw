// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

use crate::coords;

/// Endpoint-snapping tolerance, in mm.
const EPS_MM: f32 = 1e-4;

/// Streamable line-segment path with a retractable cursor.
/// Holds up to N points (N-1 line segments).
pub struct PathBuffer<const N: usize> {
    /// `front()` is the retract limit; `back()` is the destination.
    points: heapless::Deque<coords::PosPhys, N>,
    /// Segment the cursor lies on. Segent I means [point(seg), point(seg + 1)].
    seg: usize,
    /// Distance from `point(seg)` along the current segment, in `[0, seg_len]`.
    cursor: f32,
    /// Net signed distance traveled from the move's start point.
    traveled: f32,
    /// Path length retained behind the cursor (i.e. `retract_remaining`).
    behind: f32,
}

impl<const N: usize> PathBuffer<N> {
    pub fn new(src: coords::PosPhys, dst: coords::PosPhys) -> Self {
        let mut points = heapless::Deque::new();
        points.push_back(src).ok();
        points.push_back(dst).ok();
        Self {
            points,
            seg: 0,
            cursor: 0.0,
            traveled: 0.0,
            behind: 0.0,
        }
    }

    /// Current cursor position.
    pub fn position(&self) -> coords::PosPhys {
        let a = self.point(self.seg);
        let b = self.point(self.seg + 1);
        let len = a.distance_to(&b);
        if len > 0.0 {
            a.interp(&b, self.cursor / len)
        } else {
            a
        }
    }

    /// Returns true if cursor is at the destination.
    pub fn at_dst(&self) -> bool {
        self.seg + 2 == self.points.len() && self.cursor + EPS_MM >= self.seg_len(self.seg)
    }

    /// Returns whether [`extend`](Self::extend) can be called.
    pub fn can_extend(&self) -> bool {
        !self.points.is_full()
    }

    /// Extend the path by one more line segment.
    /// [`can_extend`](Self::can_extend) must be true.
    pub fn extend(&mut self, dst: coords::PosPhys) {
        assert!(self.can_extend(), "path buffer full");
        self.points.push_back(dst).ok();
    }

    /// Distance available to retract before hitting the oldest retained point.
    pub fn retract_remaining(&self) -> f32 {
        self.behind.max(0.0)
    }

    /// Net distance from the move's start point along the path.
    pub fn distance(&self) -> f32 {
        self.traveled
    }

    /// Advances by `d` mm. Negative `d` retracts. Over-runs in either direction
    /// clip silently to the limit; observe via [`at_dst`](Self::at_dst) or
    /// [`retract_remaining`](Self::retract_remaining).
    /// Takes O(segments crossed) time.
    pub fn move_by(&mut self, d: f32) {
        let moved = if d >= 0.0 {
            self.advance(d)
        } else {
            -self.retract(-d)
        };
        self.behind += moved;
        self.traveled += moved;
        self.maybe_evict();
    }

    /// Walk the cursor forward by `dist` (>=0), clipping at the frontier.
    /// Returns the distance actually moved.
    fn advance(&mut self, mut dist: f32) -> f32 {
        let mut moved = 0.0;
        loop {
            let room = self.seg_len(self.seg) - self.cursor;
            let frontier = self.seg + 2 == self.points.len();
            if dist <= room || frontier {
                let step = dist.min(room);
                self.cursor += step;
                return moved + step;
            }
            moved += room;
            dist -= room;
            self.seg += 1;
            self.cursor = 0.0;
        }
    }

    /// Walk the cursor back by `dist` (>=0), clipping at the oldest point.
    /// Returns the distance actually moved.
    fn retract(&mut self, mut dist: f32) -> f32 {
        let mut moved = 0.0;
        loop {
            if dist <= self.cursor || self.seg == 0 {
                let step = dist.min(self.cursor);
                self.cursor -= step;
                return moved + step;
            }
            moved += self.cursor;
            dist -= self.cursor;
            self.seg -= 1;
            self.cursor = self.seg_len(self.seg);
        }
    }

    /// Drop the oldest point once the cursor has reached the frontier segment of
    /// a full queue, freeing a slot for the next [`extend`](Self::extend).
    fn maybe_evict(&mut self) {
        if self.points.is_full() && self.seg + 2 == self.points.len() && self.seg > 0 {
            self.behind -= self.seg_len(0);
            self.points.pop_front();
            self.seg -= 1;
        }
    }

    /// `i`-th point of the polyline (0 = oldest, `points.len() - 1` = frontier).
    fn point(&self, i: usize) -> coords::PosPhys {
        let (a, b) = self.points.as_slices();
        if i < a.len() {
            a[i]
        } else {
            b[i - a.len()]
        }
    }

    /// Length of segment `i` (from `point(i)` to `point(i + 1)`).
    fn seg_len(&self, i: usize) -> f32 {
        self.point(i).distance_to(&self.point(i + 1))
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    /// Point capacity for the tests; large enough to avoid eviction.
    const N: usize = 401;

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

    const TOL: f32 = 1e-4;

    #[test]
    fn pb_init_basic() {
        let pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(10.0, 0.0, 0.0));
        assert_within(pb.position().x, 0.0, TOL);
        assert!(pb.can_extend(), "buffer available after construction");
        assert!(!pb.at_dst(), "initial pos is not end");
    }

    #[test]
    fn pb_move_forward_simple() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.move_by(0.5);
        assert_within(pb.position().x, 0.5, TOL);
    }

    #[test]
    fn pb_move_backward() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.move_by(0.5);
        pb.move_by(-0.2);
        assert_within(pb.position().x, 0.3, TOL);
    }

    #[test]
    fn pb_move_to_end() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(0.5, 0.0, 0.0));
        pb.move_by(1.0);
        assert!(pb.at_dst(), "should be at end after overshooting");
        assert_within(pb.position().x, 0.5, TOL);
    }

    #[test]
    fn pb_write_and_traverse() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        assert!(pb.can_extend());
        pb.extend(p3(1.0, 1.0, 0.0));
        pb.move_by(1.5);
        let pos = pb.position();
        assert_within(pos.x, 1.0, TOL);
        assert_within(pos.y, 0.5, TOL);
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
        assert_within(pos.x, 1.0, TOL);
        assert_within(pos.y, 0.0, TOL);
        assert!(!pb.at_dst(), "not ended yet (1.0 of 2.0)");
    }

    #[test]
    fn pb_zero_length_segment() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(5.0, 5.0, 5.0), p3(5.0, 5.0, 5.0));
        pb.move_by(1.0);
        assert!(pb.at_dst(), "zero-length segment should be at end");
        assert_within(pb.position().x, 5.0, TOL);
    }

    #[test]
    fn pb_full_buffer_backpressure() {
        // N=4 points → 3 segments.
        let mut pb: PathBuffer<4> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.extend(p3(2.0, 0.0, 0.0));
        pb.extend(p3(3.0, 0.0, 0.0));
        assert!(!pb.can_extend(), "queue full, cursor not at frontier");
    }

    #[test]
    fn pb_evict_oldest_when_full_at_frontier() {
        let mut pb: PathBuffer<4> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.extend(p3(2.0, 0.0, 0.0));
        pb.extend(p3(3.0, 0.0, 0.0));
        // Advance onto the frontier segment: oldest (0,0,0) is dropped.
        pb.move_by(2.5);
        assert!(pb.can_extend(), "eviction freed a slot");
        assert_within(pb.position().x, 2.5, TOL);
        // retract_remaining is data-dependent: only back to the new oldest (1,0,0).
        assert_within(pb.retract_remaining(), 1.5, TOL);
        // distance counts net travel from the start, unaffected by eviction.
        assert_within(pb.distance(), 2.5, TOL);
    }

    #[test]
    fn pb_retract_clips_at_oldest() {
        let mut pb: PathBuffer<4> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(1.0, 0.0, 0.0));
        pb.extend(p3(2.0, 0.0, 0.0));
        pb.extend(p3(3.0, 0.0, 0.0));
        pb.move_by(2.5); // evicts (0,0,0); oldest is now (1,0,0)
        pb.move_by(-5.0); // retract past the dropped history
        assert_within(pb.position().x, 1.0, TOL);
        assert_within(pb.retract_remaining(), 0.0, TOL);
        assert_within(pb.distance(), 1.0, TOL);
    }

    #[test]
    fn pb_get_distance() {
        let mut pb: PathBuffer<N> = PathBuffer::new(p3(0.0, 0.0, 0.0), p3(4.0, 0.0, 0.0));
        assert_within(pb.distance(), 0.0, TOL);
        pb.move_by(2.0);
        assert_within(pb.distance(), 2.0, TOL);
        pb.move_by(-1.0);
        assert_within(pb.distance(), 1.0, TOL);
        pb.extend(p3(4.0, 4.0, 0.0));
        pb.move_by(5.0);
        assert_within(pb.distance(), 6.0, TOL);
        pb.move_by(100.0); // clipped at end
        assert_within(pb.distance(), 8.0, TOL);
    }
}
