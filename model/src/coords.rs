// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

use core::f32::consts;

/// Single physical coordinate (G-code coordinate specification).
///
/// `c` is in turns, conventionally in `[0, 1)`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PosPhys {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub c: f32,
}

impl PosPhys {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        c: 0.0,
    };

    /// Add an XYZ work-offset (work-coords -> machine-coords). C-axis is never
    /// offset, so it passes through unchanged.
    pub fn with_offset_added(self, off: PosPhys) -> Self {
        Self {
            x: self.x + off.x,
            y: self.y + off.y,
            z: self.z + off.z,
            c: self.c,
        }
    }

    /// Remove an XYZ work-offset (machine-coords -> work-coords). Inverse of
    /// [`with_offset_added`](Self::with_offset_added); C-axis unchanged.
    pub fn with_offset_removed(self, off: PosPhys) -> Self {
        Self {
            x: self.x - off.x,
            y: self.y - off.y,
            z: self.z - off.z,
            c: self.c,
        }
    }

    /// Distance in mm. C-axis contribution assumes 2 mm effective radius.
    pub fn distance_to(&self, other: &Self) -> f32 {
        let dx = other.x - self.x;
        let dy = other.y - self.y;
        let dz = other.z - self.z;
        let mut dc = shortest_turn_delta(self.c, other.c);
        dc *= 2.0 * consts::PI * 2.0;
        libm::sqrtf(dx * dx + dy * dy + dz * dz + dc * dc)
    }

    /// Linear interpolation; `t=0` returns self, `t=1` returns other.
    /// C-axis uses shortest-path interpolation, wrapped to `[0, 1)`.
    pub fn interp(&self, other: &Self, t: f32) -> Self {
        let x = self.x + (other.x - self.x) * t;
        let y = self.y + (other.y - self.y) * t;
        let z = self.z + (other.z - self.z) * t;
        let c_delta = shortest_turn_delta(self.c, other.c);
        let c = wrap_turns(self.c + c_delta * t);
        Self { x, y, z, c }
    }
}

/// Non-machine coords.
#[derive(Clone, Copy, Debug, PartialEq, Eq, enum_map::Enum)]
pub enum CoordSys {
    W,
    G,
}

/// A linear/rotary machine axis.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActiveCoordSys {
    Machine,
    Offset(CoordSys),
}

/// Wrap a turn value to `[0, 1)`.
pub fn wrap_turns(turns: f32) -> f32 {
    let r = libm::fmodf(turns, 1.0);
    if r < 0.0 {
        r + 1.0
    } else {
        r
    }
}

/// Shortest rotational delta in turns, in `(-0.5, 0.5]`.
pub fn shortest_turn_delta(current: f32, target: f32) -> f32 {
    let delta = target - current;
    if delta > 0.5 {
        delta - 1.0
    } else if delta <= -0.5 {
        delta + 1.0
    } else {
        delta
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f32, y: f32, z: f32) -> PosPhys {
        PosPhys { x, y, z, c: 0.0 }
    }

    fn within(a: f32, b: f32, tol: f32) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn posp_dist_basic() {
        let a = p(0.0, 0.0, 0.0);
        let b = p(3.0, 4.0, 0.0);
        assert!(within(a.distance_to(&b), 5.0, 1e-4));
    }

    #[test]
    fn posp_dist_zero() {
        let a = p(1.0, 2.0, 3.0);
        assert!(within(a.distance_to(&a), 0.0, 1e-4));
    }

    #[test]
    fn posp_dist_3d() {
        let a = p(0.0, 0.0, 0.0);
        let b = p(1.0, 1.0, 1.0);
        assert!(within(a.distance_to(&b), libm::sqrtf(3.0), 1e-4));
    }

    #[test]
    fn posp_interp_midpoint() {
        let a = p(0.0, 0.0, 0.0);
        let b = p(10.0, 20.0, 30.0);
        let mid = a.interp(&b, 0.5);
        assert!(within(mid.x, 5.0, 1e-4));
        assert!(within(mid.y, 10.0, 1e-4));
        assert!(within(mid.z, 15.0, 1e-4));
    }

    #[test]
    fn posp_interp_extrapolate() {
        let a = p(0.0, 0.0, 0.0);
        let b = p(10.0, 10.0, 10.0);
        let r = a.interp(&b, -0.5);
        assert!(within(r.x, -5.0, 1e-4));
        assert!(within(r.y, -5.0, 1e-4));
        assert!(within(r.z, -5.0, 1e-4));
    }

    #[test]
    fn posp_interp_endpoints() {
        let a = p(1.0, 2.0, 3.0);
        let b = p(4.0, 5.0, 6.0);
        let r0 = a.interp(&b, 0.0);
        assert!(within(r0.x, 1.0, 1e-4));
        let r1 = a.interp(&b, 1.0);
        assert!(within(r1.x, 4.0, 1e-4));
    }
}
