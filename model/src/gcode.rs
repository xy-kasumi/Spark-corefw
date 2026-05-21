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
    /// G53-G56: select the active (modal) coordinate system.
    SelectCoordSys(coords::ActiveCoordSys),
    /// M8: start the pump.
    PumpOn,
    /// M9: stop the pump.
    PumpOff,
    /// M10: start wire feeding at the given feedrate in mm/min.
    WirefeedStart(f32),
    /// M11: stop wire feeding.
    WirefeedStop,
    /// M60: open the tool supply.
    ToolSupplyOpen,
    /// M61: close the tool supply.
    ToolSupplyClose,
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
        (60, None) => no_params(p).map(|_| Parsed::ToolSupplyOpen),
        (61, None) => no_params(p).map(|_| Parsed::ToolSupplyClose),
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

/// Parse a coordinate-system select (G53-G56). Takes no parameters.
fn parse_select(p: &mut Cursor, code: i32) -> Option<Parsed> {
    if !p.eof_or_only_ws() {
        return None;
    }
    let cs = coords::ActiveCoordSys::from_gcode(code)?;
    Some(Parsed::SelectCoordSys(cs))
}

fn parse_move(p: &mut Cursor) -> Option<MoveSpec> {
    let mut spec = MoveSpec::default();
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
    }
    Some(spec)
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
        // Read all chars that could be part of a float (digits + at most one dot).
        // We don't validate here — let parse() reject malformed (e.g. "10..5", "10.5.2").
        while !self.eof() && (self.buf[self.pos].is_ascii_digit() || self.buf[self.pos] == b'.') {
            self.pos += 1;
        }
        if self.pos == start || (self.pos == start + 1 && matches!(self.buf[start], b'-' | b'+')) {
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
    //! The typed parser rejects unknown M-codes, so only M3/M4 are accepted
    //! (a generic parser accepting any M number would also take M5/M999).

    use super::*;

    #[test]
    fn basic_m3_command() {
        assert_eq!(
            parse(b"M3").unwrap(),
            Parsed::Pulser(PulserSpec {
                tool_negative: true,
                pulse_us: None,
                current_a: None,
                duty_pct: None,
            })
        );
    }

    #[test]
    fn m3_with_all_parameters() {
        let Parsed::Pulser(c) = parse(b"M3 P750 Q1.5 R30").unwrap() else {
            panic!("expected Pulser");
        };
        assert!(c.tool_negative);
        assert_eq!(c.pulse_us, Some(750.0));
        assert_eq!(c.current_a, Some(1.5));
        assert_eq!(c.duty_pct, Some(30.0));
    }

    #[test]
    fn m4_with_partial_parameters() {
        // Tool-positive; P omitted stays None (executor defaults it), Q/R given.
        let Parsed::Pulser(c) = parse(b"M4 Q2.0 R25").unwrap() else {
            panic!("expected Pulser");
        };
        assert!(!c.tool_negative);
        assert_eq!(c.pulse_us, None);
        assert_eq!(c.current_a, Some(2.0));
        assert_eq!(c.duty_pct, Some(25.0));
    }

    #[test]
    fn m3_mixed_parameters() {
        let Parsed::Pulser(c) = parse(b"M3 P1000 R50").unwrap() else {
            panic!("expected Pulser");
        };
        assert_eq!(c.pulse_us, Some(1000.0));
        assert_eq!(c.current_a, None);
        assert_eq!(c.duty_pct, Some(50.0));
    }

    #[test]
    fn m3_bare_param_fails() {
        assert!(parse(b"M3 P").is_none());
    }

    #[test]
    fn m3_unknown_param_fails() {
        assert!(parse(b"M3 P500 S100").is_none());
    }

    #[test]
    fn basic_g0_command() {
        let cmd = parse(b"G0").unwrap();
        assert_eq!(cmd, Parsed::Rapid(MoveSpec::default()));
    }

    #[test]
    fn g1_with_coordinates() {
        let Parsed::Feed(s) = parse(b"G1 X10.5 Y-20.3 Z5").unwrap() else {
            panic!("expected Linear");
        };
        assert_eq!(s.x, Some(10.5));
        assert_eq!(s.y, Some(-20.3));
        assert_eq!(s.z, Some(5.0));
    }

    #[test]
    fn g0_with_c_axis() {
        // C parser stores c in degrees; our parser converts to turns at parse time.
        let Parsed::Rapid(s) = parse(b"G0 X10 Y20 C45.5").unwrap() else {
            panic!("expected Rapid");
        };
        assert_eq!(s.x, Some(10.0));
        assert_eq!(s.y, Some(20.0));
        assert_eq!(s.c, Some(45.5 / 360.0));
    }

    #[test]
    fn g1_with_all_axes() {
        // C: c=90 (degrees). Rust: c=0.25 (turns).
        let Parsed::Feed(s) = parse(b"G1 X1.5 Y2.5 Z3.5 C90").unwrap() else {
            panic!("expected Linear");
        };
        assert_eq!(s.x, Some(1.5));
        assert_eq!(s.y, Some(2.5));
        assert_eq!(s.z, Some(3.5));
        assert_eq!(s.c, Some(0.25));
    }

    #[test]
    fn g38_3_command() {
        let Parsed::Probe(s) = parse(b"G38.3").unwrap() else {
            panic!("expected Probe");
        };
        assert_eq!(s, MoveSpec::default());
    }

    #[test]
    fn g38_3_with_target() {
        let Parsed::Probe(s) = parse(b"G38.3 Z-5").unwrap() else {
            panic!("expected Probe");
        };
        assert_eq!(s.z, Some(-5.0));
    }

    #[test]
    fn g38_2_unsupported() {
        // Only G38.3 is handled; G38.2 and bare G38 are unknown.
        assert_eq!(parse(b"G38.2"), None);
        assert_eq!(parse(b"G38"), None);
    }

    #[test]
    fn g28_axis_only() {
        assert_eq!(
            parse(b"G28 X").unwrap(),
            Parsed::Home(HomeSpec::One(coords::Axis::X))
        );
    }

    #[test]
    fn g28_c_rejected() {
        // C is not homeable (spec lists X/Y/Z only).
        assert!(parse(b"G28 C").is_none());
    }

    #[test]
    fn g28_home_all() {
        assert_eq!(parse(b"G28").unwrap(), Parsed::Home(HomeSpec::All));
    }

    #[test]
    fn g28_rejects_value() {
        assert!(parse(b"G28 X10").is_none());
    }

    #[test]
    fn g28_rejects_two_axes() {
        assert!(parse(b"G28 X Y").is_none());
    }

    #[test]
    fn empty_string() {
        assert_eq!(parse(b""), None);
    }

    #[test]
    fn whitespace_only() {
        assert_eq!(parse(b"   "), None);
    }

    #[test]
    fn extra_whitespace_success() {
        let Parsed::Rapid(s) = parse(b"G0   X10.5    Y20").unwrap() else {
            panic!("expected Rapid");
        };
        assert_eq!(s.x, Some(10.5));
        assert_eq!(s.y, Some(20.0));
    }

    #[test]
    fn lowercase_command_fails() {
        assert!(parse(b"g0 X10").is_none());
    }

    #[test]
    fn lowercase_parameter_fails() {
        assert!(parse(b"G0 x10").is_none());
    }

    #[test]
    fn garbled_command_fails() {
        assert!(parse(b"G0abc X10").is_none());
    }

    #[test]
    fn garbled_number_fails() {
        assert!(parse(b"G0 X10.5.2").is_none());
    }

    #[test]
    fn no_whitespace_between_params_fails() {
        assert!(parse(b"G0X1Y2").is_none());
    }
}
