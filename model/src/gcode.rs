// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! G-code parser.

use core::str;

use crate::coords;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Parsed {
    /// G0: rapid move.
    Rapid(MoveSpec),
    /// G1: feed move.
    Feed(MoveSpec),
    /// G28: home axes.
    Home(HomeSpec),
    /// G38.3: probe toward target, stop on contact, no error if not reached.
    Probe(MoveSpec),
    /// G53-G55: select the active (modal) coordinate system.
    SelectCoordSys(coords::ActiveCoordSys),
    /// M8: start the pump.
    PumpOn,
    /// M9: stop the pump.
    PumpOff,
    /// M10: start wire feeding at the given feedrate in mm/min.
    WirefeedStart(f32),
    /// M11: stop wire feeding.
    WirefeedStop,
    /// M3/M4: set the modal pulser parameters used by the next G1/G38.3.
    Pulser(PulserSpec),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PulserSpec {
    pub tool_negative: bool,
    /// Pulse on-time, µs (P).
    pub pulse_us: Option<f32>,
    /// Pulse current, A (Q).
    pub current_a: Option<f32>,
    /// Max duty cycle, percent (R).
    pub duty_pct: Option<f32>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MoveSpec {
    pub x: Option<f32>,
    pub y: Option<f32>,
    pub z: Option<f32>,
    /// C-axis in turns. Parser converts the incoming degrees from G-code.
    pub c: Option<f32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HomeSpec {
    All,
    One(coords::Axis),
}

pub fn parse(line: &[u8]) -> Option<Parsed> {
    let mut p = Cursor::new(line);
    p.skip_ws();
    let (letter, code) = p.read_letter_int()?;
    let sub = p.read_subcode()?;
    match letter {
        b'G' => parse_gcode(&mut p, code, sub),
        b'M' => parse_mcode(&mut p, code, sub),
        _ => None,
    }
}

fn parse_gcode(p: &mut Cursor, code: i32, sub: Option<i32>) -> Option<Parsed> {
    match (code, sub) {
        (0, None) => parse_move(p).map(Parsed::Rapid),
        (1, None) => parse_move(p).map(Parsed::Feed),
        (28, None) => parse_home(p).map(Parsed::Home),
        (38, Some(3)) => parse_move(p).map(Parsed::Probe),
        (53..=56, None) => parse_select(p, code),
        _ => None,
    }
}

fn parse_mcode(p: &mut Cursor, code: i32, sub: Option<i32>) -> Option<Parsed> {
    match (code, sub) {
        (3, None) => parse_pulser(p, true).map(Parsed::Pulser),
        (4, None) => parse_pulser(p, false).map(Parsed::Pulser),
        (8, None) => no_params(p).map(|_| Parsed::PumpOn),
        (9, None) => no_params(p).map(|_| Parsed::PumpOff),
        (10, None) => parse_required_r(p).map(Parsed::WirefeedStart),
        (11, None) => no_params(p).map(|_| Parsed::WirefeedStop),
        _ => None,
    }
}

/// Reject any trailing parameter for M-codes that take none.
fn no_params(p: &mut Cursor) -> Option<()> {
    p.eof_or_only_ws().then_some(())
}

/// Parse the single required `R<float>` parameter (M10 feedrate, mm/min).
fn parse_required_r(p: &mut Cursor) -> Option<f32> {
    if p.eof_or_only_ws() {
        return None;
    }
    if !p.require_ws() {
        return None;
    }
    let letter = p.read_letter()?;
    if letter != b'R' {
        return None;
    }
    let value = p.read_float()?;
    no_params(p)?;
    Some(value)
}

/// Parse M3/M4 pulser parameters: optional `P`/`Q`/`R` floats in any order.
/// Omitted parameters stay `None` (the executor applies defaults). A bare letter
/// (no value) or an unrecognized letter is rejected.
fn parse_pulser(p: &mut Cursor, tool_negative: bool) -> Option<PulserSpec> {
    let mut params = PulserSpec {
        tool_negative,
        pulse_us: None,
        current_a: None,
        duty_pct: None,
    };
    loop {
        if p.eof_or_only_ws() {
            break;
        }
        if !p.require_ws() {
            return None;
        }
        let letter = p.read_letter()?;
        let value = p.read_float()?;
        match letter {
            b'P' => params.pulse_us = Some(value),
            b'Q' => params.current_a = Some(value),
            b'R' => params.duty_pct = Some(value),
            _ => return None,
        }
    }
    Some(params)
}

/// Parse a `G28` target: no axis letter (home all), or exactly one bare X/Y/Z.
/// Two axes, a value after a letter, or a non-homeable letter (e.g. C) is rejected.
fn parse_home(p: &mut Cursor) -> Option<HomeSpec> {
    if p.eof_or_only_ws() {
        return Some(HomeSpec::All);
    }
    if !p.require_ws() {
        return None;
    }
    let axis = match p.read_letter()? {
        b'X' => coords::Axis::X,
        b'Y' => coords::Axis::Y,
        b'Z' => coords::Axis::Z,
        _ => return None,
    };
    // Exactly one bare axis: nothing but trailing whitespace may follow (this
    // rejects both a value, e.g. `G28 X10`, and a second axis, e.g. `G28 X Y`).
    if !p.eof_or_only_ws() {
        return None;
    }
    Some(HomeSpec::One(axis))
}

/// Parse a coordinate-system select (G53-G55). Takes no parameters.
fn parse_select(p: &mut Cursor, code: i32) -> Option<Parsed> {
    if !p.eof_or_only_ws() {
        return None;
    }
    let cs = match code {
        53 => coords::ActiveCoordSys::Machine,
        54 => coords::ActiveCoordSys::Offset(coords::CoordSys::G),
        55 => coords::ActiveCoordSys::Offset(coords::CoordSys::W),
        _ => return None,
    };
    Some(Parsed::SelectCoordSys(cs))
}

fn parse_move(p: &mut Cursor) -> Option<MoveSpec> {
    let mut spec = MoveSpec::default();
    let mut seen_axis = false;
    loop {
        if p.eof_or_only_ws() {
            break;
        }
        if !p.require_ws() {
            return None;
        }
        let letter = p.read_letter()?;
        let value = p.read_float()?;
        match letter {
            b'X' => spec.x = Some(value),
            b'Y' => spec.y = Some(value),
            b'Z' => spec.z = Some(value),
            b'C' => spec.c = Some(value / 360.0),
            _ => return None,
        }
        seen_axis = true;
    }
    // A move needs at least one axis; bare `G0`/`G1`/`G38.3` is a form error.
    seen_axis.then_some(spec)
}

struct Cursor<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    fn eof(&self) -> bool {
        self.pos >= self.buf.len()
    }

    fn eof_or_only_ws(&self) -> bool {
        self.buf[self.pos..]
            .iter()
            .all(|b| matches!(b, b' ' | b'\t'))
    }

    fn skip_ws(&mut self) {
        while !self.eof() && matches!(self.buf[self.pos], b' ' | b'\t') {
            self.pos += 1;
        }
    }

    /// Skip whitespace and return whether any was consumed.
    fn require_ws(&mut self) -> bool {
        let start = self.pos;
        self.skip_ws();
        self.pos > start
    }

    fn read_letter(&mut self) -> Option<u8> {
        if self.eof() {
            return None;
        }
        let b = self.buf[self.pos];
        if b.is_ascii_uppercase() {
            self.pos += 1;
            Some(b)
        } else {
            None
        }
    }

    fn read_letter_int(&mut self) -> Option<(u8, i32)> {
        let letter = self.read_letter()?;
        let value = self.read_int()?;
        Some((letter, value))
    }

    /// Read an optional `.N` sub-code attached to the command number. Outer
    /// `None` signals a parse failure (a dot with no digits); inner `None` means
    /// no dot follows.
    fn read_subcode(&mut self) -> Option<Option<i32>> {
        if self.eof() || self.buf[self.pos] != b'.' {
            return Some(None);
        }
        self.pos += 1;
        self.read_int().map(Some)
    }

    fn read_int(&mut self) -> Option<i32> {
        let start = self.pos;
        while !self.eof() && self.buf[self.pos].is_ascii_digit() {
            self.pos += 1;
        }
        if self.pos == start {
            return None;
        }
        str::from_utf8(&self.buf[start..self.pos])
            .ok()?
            .parse()
            .ok()
    }

    fn read_float(&mut self) -> Option<f32> {
        let start = self.pos;
        if !self.eof() && matches!(self.buf[self.pos], b'-' | b'+') {
            self.pos += 1;
        }
        let int_start = self.pos;
        // Read all chars that could be part of a float (digits + at most one dot).
        // We don't validate the shape here — let parse() reject malformed runs of
        // digits/dots (e.g. "10..5", "10.5.2").
        while !self.eof() && (self.buf[self.pos].is_ascii_digit() || self.buf[self.pos] == b'.') {
            self.pos += 1;
        }
        if self.pos == start || (self.pos == start + 1 && matches!(self.buf[start], b'-' | b'+')) {
            return None;
        }
        // Reject a leading zero in the integer part (`05`). `0`, `0.5`, `.5` are fine.
        if self.buf[int_start] == b'0'
            && int_start + 1 < self.pos
            && self.buf[int_start + 1].is_ascii_digit()
        {
            return None;
        }
        str::from_utf8(&self.buf[start..self.pos])
            .ok()?
            .parse()
            .ok()
    }
}

#[cfg(test)]
mod tests {
    //! Organized by the claim each test defends, not by example:
    //!   1. dispatch table — every command maps to its `Parsed` variant;
    //!   2. form rejections — near-misses one mutation away from valid;
    //!   3. properties — invariants over the whole input space (proptest).
    //!
    //! Concrete cases snapshot the full `Debug` of `parse(line)` (rejections render
    //! as `None`); run with `UPDATE_EXPECT=1` to regenerate the expected strings.

    extern crate std;

    use expect_test::expect;
    use proptest::prelude::*;

    use super::*;

    fn check(line: &str, expected: expect_test::Expect) {
        expected.assert_eq(&std::format!("{:?}", parse(line.as_bytes())));
    }

    // --- 1. Dispatch table: each command maps to its variant. ---

    #[test]
    fn dispatch_moves() {
        check(
            "G0 X12.3",
            expect![["Some(Rapid(MoveSpec { x: Some(12.3), y: None, z: None, c: None }))"]],
        );
        check(
            "G1 X10 Y20",
            expect![["Some(Feed(MoveSpec { x: Some(10.0), y: Some(20.0), z: None, c: None }))"]],
        );
        check(
            "G38.3 X10 Y3.5",
            expect![["Some(Probe(MoveSpec { x: Some(10.0), y: Some(3.5), z: None, c: None }))"]],
        );
    }

    #[test]
    fn dispatch_home() {
        check("G28", expect!["Some(Home(All))"]);
        check("G28 X", expect!["Some(Home(One(X)))"]);
    }

    #[test]
    fn dispatch_coord_select() {
        check("G53", expect!["Some(SelectCoordSys(Machine))"]);
        check("G54", expect!["Some(SelectCoordSys(Offset(G)))"]);
        check("G55", expect!["Some(SelectCoordSys(Offset(W)))"]);
    }

    #[test]
    fn dispatch_pulser() {
        // M3 = tool-negative, all params default (left None for the executor).
        check(
            "M3",
            expect![[
                "Some(Pulser(PulserSpec { tool_negative: true, pulse_us: None, current_a: None, duty_pct: None }))"
            ]],
        );
        // M4 = tool-positive; P/Q/R map to pulse_us/current_a/duty_pct.
        check(
            "M4 P1000 Q0.8",
            expect![[
                "Some(Pulser(PulserSpec { tool_negative: false, pulse_us: Some(1000.0), current_a: Some(0.8), duty_pct: None }))"
            ]],
        );
    }

    #[test]
    fn dispatch_pump() {
        check("M8", expect!["Some(PumpOn)"]);
        check("M9", expect!["Some(PumpOff)"]);
    }

    #[test]
    fn dispatch_wirefeed() {
        check("M10 R120", expect!["Some(WirefeedStart(120.0))"]);
        check("M11", expect!["Some(WirefeedStop)"]);
    }

    #[test]
    fn pulser_omitted_params_stay_none() {
        // A different subset/order from dispatch_pulser: P omitted stays None.
        check(
            "M3 R30 Q2.0",
            expect![[
                "Some(Pulser(PulserSpec { tool_negative: true, pulse_us: None, current_a: Some(2.0), duty_pct: Some(30.0) }))"
            ]],
        );
    }

    // --- 2. Form rejections: errors decidable from the line alone. ---

    #[test]
    fn rejects_bare_move() {
        // A move needs at least one axis (spec: "G0 ; error").
        check("G0", expect!["None"]);
        check("G1", expect!["None"]);
        check("G38.3", expect!["None"]);
    }

    #[test]
    fn rejects_lowercase() {
        check("g0 X10", expect!["None"]); // command
        check("G0 x10", expect!["None"]); // parameter
    }

    #[test]
    fn requires_whitespace_between_tokens() {
        check("G0X1Y2", expect!["None"]);
    }

    #[test]
    fn subcode_must_match_exactly() {
        // Only G38.3 is handled; a different subcode or a bare G38 is unknown.
        check("G38.2", expect!["None"]);
        check("G38", expect!["None"]);
    }

    #[test]
    fn rejects_malformed_subcode() {
        check("G38.", expect!["None"]);
    }

    #[test]
    fn rejects_unknown_param() {
        check("G0 S5", expect!["None"]);
        check("M3 P500 S100", expect!["None"]);
    }

    #[test]
    fn rejects_bare_param() {
        check("G0 X", expect!["None"]);
        check("M3 P", expect!["None"]);
    }

    #[test]
    fn no_param_command_rejects_args() {
        check("M8 X1", expect!["None"]);
    }

    #[test]
    fn m10_requires_r() {
        check("M10", expect!["None"]); // R is required
        check("M10 P5", expect!["None"]); // wrong letter
    }

    #[test]
    fn g28_at_most_one_axis() {
        check("G28 X Y", expect!["None"]);
    }

    #[test]
    fn g28_rejects_value() {
        check("G28 X10", expect!["None"]);
    }

    #[test]
    fn g28_rejects_non_homeable_axis() {
        check("G28 C", expect!["None"]);
    }

    #[test]
    fn rejects_leading_zero() {
        // `05` is not a valid number (spec: "## Numbers").
        check("G0 X05", expect!["None"]);
    }

    #[test]
    fn rejects_malformed_number() {
        check("G0 X10.5.2", expect!["None"]);
    }

    #[test]
    fn rejects_garbage_after_command() {
        check("G0abc X10", expect!["None"]);
    }

    #[test]
    fn rejects_empty_and_whitespace() {
        check("", expect!["None"]);
        check("   ", expect!["None"]);
    }

    // --- 3. Properties over the whole input space. ---

    // Build a move command from a non-empty axis subset, rendering it with a
    // shuffled order and varied inter-token spacing alongside the spec it must
    // parse back to. This one property subsumes order-independence, whitespace
    // insensitivity, value round-tripping, and the C-axis degrees->turns conversion.
    prop_compose! {
        fn move_case()(
            pairs in prop::sample::subsequence(std::vec![b'X', b'Y', b'Z', b'C'], 1..=4)
                .prop_flat_map(|letters| {
                    let n = letters.len();
                    (Just(letters), prop::collection::vec(-1000.0f32..1000.0, n))
                })
                .prop_map(|(letters, values)| {
                    letters.into_iter().zip(values).collect::<std::vec::Vec<(u8, f32)>>()
                })
                .prop_shuffle(),
            gaps in prop::collection::vec(1usize..=3, 4),
        ) -> (std::string::String, MoveSpec) {
            let mut line = std::string::String::from("G0");
            let mut spec = MoveSpec::default();
            for (i, (letter, value)) in pairs.iter().enumerate() {
                for _ in 0..gaps[i] {
                    line.push(' ');
                }
                line.push(*letter as char);
                line.push_str(&std::format!("{value}"));
                match letter {
                    b'X' => spec.x = Some(*value),
                    b'Y' => spec.y = Some(*value),
                    b'Z' => spec.z = Some(*value),
                    b'C' => spec.c = Some(value / 360.0),
                    _ => unreachable!(),
                }
            }
            (line, spec)
        }
    }

    proptest! {
        /// The parser must never panic.
        #[test]
        fn never_panics_on_arbitrary_bytes(bytes in prop::collection::vec(any::<u8>(), 0..32)) {
            let _ = parse(&bytes);
        }

        /// Axis order and inter-token spacing don't change the parsed move.
        #[test]
        fn move_recovers_axes_regardless_of_order_and_spacing((line, spec) in move_case()) {
            prop_assert_eq!(parse(line.as_bytes()), Some(Parsed::Rapid(spec)));
        }
    }
}
