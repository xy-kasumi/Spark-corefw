// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! G-code parser. Phase 3 sketch: G0 and G1 implemented; further commands stubbed for Phase 4.
//!
//! Whitespace and case rules mirror the C reference parser:
//! - Letters must be uppercase.
//! - Whitespace is required between command and parameters, and between each parameter.

use core::str;

use crate::coords::ActiveCoordSys;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    /// G0: rapid positioning.
    Rapid(MoveSpec),
    /// G1: feed (linear) move.
    Linear(MoveSpec),
    /// G28: home the named axes (or all, if none named).
    Home(HomeAxes),
    /// G38.3: probe toward target, stop on contact, no error if not reached.
    Probe(MoveSpec),
    /// G53-G56: select the active (modal) coordinate system.
    SelectCoordSys(ActiveCoordSys),
    /// M8/M9: enable (true) or disable (false) the pump.
    Pump(bool),
    /// M10: start wire feeding at the given feedrate in mm/min.
    WirefeedStart(f32),
    /// M11: stop wire feeding.
    WirefeedStop,
    /// M60/M61: move the tool supply servo to the given state.
    ToolSupply(ToolSupplyState),
    /// M3/M4: set the modal pulser configuration used by the next G1/G38.3.
    Pulser(PulserConfig),
}

/// Modal pulser parameters set by M3 (tool-negative) / M4 (tool-positive).
/// Unspecified P/Q/R fall back to defaults — M3/M4 fully replace the prior
/// config rather than merging, matching the C `decode_pulser_params`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PulserConfig {
    pub tool_negative: bool,
    /// Pulse on-time, µs (P).
    pub pulse_us: f32,
    /// Pulse current, A (Q).
    pub current_a: f32,
    /// Max duty cycle, percent (R).
    pub duty_pct: f32,
}

impl Default for PulserConfig {
    fn default() -> Self {
        Self {
            tool_negative: true,
            pulse_us: 500.0,
            current_a: 1.0,
            duty_pct: 25.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ToolSupplyState {
    Open,
    Closed,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MoveSpec {
    pub x: Option<f32>,
    pub y: Option<f32>,
    pub z: Option<f32>,
    /// C-axis in turns. Parser converts the incoming degrees from G-code.
    pub c: Option<f32>,
}

/// Which axes a `G28` named (bare letters, no values). No axis named means
/// "home all"; the executor rejects C and multi-axis combinations.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct HomeAxes {
    pub x: bool,
    pub y: bool,
    pub z: bool,
    pub c: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParseError {
    Empty,
    UnknownCommand,
    /// Any malformed-body failure (bad axis/number, missing or trailing param,
    /// missing separator). The offending line is echoed back, so the category
    /// alone is enough.
    Syntax,
}

pub fn parse(line: &[u8]) -> Result<Command, ParseError> {
    let mut p = Cursor::new(line);
    p.skip_ws();
    let (letter, code) = p.read_letter_int().ok_or(ParseError::Empty)?;
    let sub = p.read_subcode()?;
    match letter {
        b'G' => parse_gcode(&mut p, code, sub),
        b'M' => parse_mcode(&mut p, code, sub),
        _ => Err(ParseError::UnknownCommand),
    }
}

fn parse_gcode(p: &mut Cursor, code: i32, sub: Option<i32>) -> Result<Command, ParseError> {
    match (code, sub) {
        (0, None) => parse_move(p).map(Command::Rapid),
        (1, None) => parse_move(p).map(Command::Linear),
        (28, None) => parse_home(p).map(Command::Home),
        (38, Some(3)) => parse_move(p).map(Command::Probe),
        (53..=56, None) => parse_select(p, code),
        _ => Err(ParseError::UnknownCommand),
    }
}

fn parse_mcode(p: &mut Cursor, code: i32, sub: Option<i32>) -> Result<Command, ParseError> {
    match (code, sub) {
        (3, None) => parse_pulser(p, true).map(Command::Pulser),
        (4, None) => parse_pulser(p, false).map(Command::Pulser),
        (8, None) => no_params(p).map(|_| Command::Pump(true)),
        (9, None) => no_params(p).map(|_| Command::Pump(false)),
        (10, None) => parse_required_r(p).map(Command::WirefeedStart),
        (11, None) => no_params(p).map(|_| Command::WirefeedStop),
        (60, None) => no_params(p).map(|_| Command::ToolSupply(ToolSupplyState::Open)),
        (61, None) => no_params(p).map(|_| Command::ToolSupply(ToolSupplyState::Closed)),
        _ => Err(ParseError::UnknownCommand),
    }
}

/// Reject any trailing parameter for M-codes that take none.
fn no_params(p: &mut Cursor) -> Result<(), ParseError> {
    if p.eof_or_only_ws() {
        Ok(())
    } else {
        Err(ParseError::Syntax)
    }
}

/// Parse the single required `R<float>` parameter (M10 feedrate, mm/min).
fn parse_required_r(p: &mut Cursor) -> Result<f32, ParseError> {
    if p.eof_or_only_ws() {
        return Err(ParseError::Syntax);
    }
    if !p.require_ws() {
        return Err(ParseError::Syntax);
    }
    let letter = p.read_letter().ok_or(ParseError::Syntax)?;
    if letter != b'R' {
        return Err(ParseError::Syntax);
    }
    let value = p.read_float().ok_or(ParseError::Syntax)?;
    no_params(p)?;
    Ok(value)
}

/// Parse M3/M4 pulser parameters: optional `P`/`Q`/`R` floats in any order.
/// Omitted parameters keep their [`PulserConfig::default`] value. A bare letter
/// (no value) or an unrecognized letter is rejected.
fn parse_pulser(p: &mut Cursor, tool_negative: bool) -> Result<PulserConfig, ParseError> {
    let mut cfg = PulserConfig {
        tool_negative,
        ..Default::default()
    };
    loop {
        if p.eof_or_only_ws() {
            break;
        }
        if !p.require_ws() {
            return Err(ParseError::Syntax);
        }
        let letter = p.read_letter().ok_or(ParseError::Syntax)?;
        let value = p.read_float().ok_or(ParseError::Syntax)?;
        match letter {
            b'P' => cfg.pulse_us = value,
            b'Q' => cfg.current_a = value,
            b'R' => cfg.duty_pct = value,
            _ => return Err(ParseError::Syntax),
        }
    }
    Ok(cfg)
}

/// Parse a `G28` axis list: bare uppercase axis letters separated by whitespace,
/// with no values (a value after a letter is rejected).
fn parse_home(p: &mut Cursor) -> Result<HomeAxes, ParseError> {
    let mut axes = HomeAxes::default();
    loop {
        if p.eof_or_only_ws() {
            break;
        }
        if !p.require_ws() {
            return Err(ParseError::Syntax);
        }
        let letter = p.read_letter().ok_or(ParseError::Syntax)?;
        match letter {
            b'X' => axes.x = true,
            b'Y' => axes.y = true,
            b'Z' => axes.z = true,
            b'C' => axes.c = true,
            _ => return Err(ParseError::Syntax),
        }
        // Bare letters only: a digit (value) following an axis is invalid.
        if !p.eof_or_only_ws() && !p.at_ws() {
            return Err(ParseError::Syntax);
        }
    }
    Ok(axes)
}

/// Parse a coordinate-system select (G53-G56). Takes no parameters.
fn parse_select(p: &mut Cursor, code: i32) -> Result<Command, ParseError> {
    if !p.eof_or_only_ws() {
        return Err(ParseError::Syntax);
    }
    let cs = ActiveCoordSys::from_gcode(code).ok_or(ParseError::UnknownCommand)?;
    Ok(Command::SelectCoordSys(cs))
}

fn parse_move(p: &mut Cursor) -> Result<MoveSpec, ParseError> {
    let mut spec = MoveSpec::default();
    loop {
        if p.eof_or_only_ws() {
            break;
        }
        if !p.require_ws() {
            return Err(ParseError::Syntax);
        }
        let letter = p.read_letter().ok_or(ParseError::Syntax)?;
        let value = p.read_float().ok_or(ParseError::Syntax)?;
        match letter {
            b'X' => spec.x = Some(value),
            b'Y' => spec.y = Some(value),
            b'Z' => spec.z = Some(value),
            b'C' => spec.c = Some(value / 360.0),
            _ => return Err(ParseError::Syntax),
        }
    }
    Ok(spec)
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

    fn at_ws(&self) -> bool {
        !self.eof() && matches!(self.buf[self.pos], b' ' | b'\t')
    }

    /// Read an optional `.N` sub-code attached to the command number. `None` when
    /// no dot follows; `Err(BadNumber)` for a dot with no digits.
    fn read_subcode(&mut self) -> Result<Option<i32>, ParseError> {
        if self.eof() || self.buf[self.pos] != b'.' {
            return Ok(None);
        }
        self.pos += 1;
        self.read_int().map(Some).ok_or(ParseError::Syntax)
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
            Command::Pulser(PulserConfig {
                tool_negative: true,
                ..Default::default()
            })
        );
    }

    #[test]
    fn m3_with_all_parameters() {
        let Command::Pulser(c) = parse(b"M3 P750 Q1.5 R30").unwrap() else {
            panic!("expected Pulser");
        };
        assert!(c.tool_negative);
        assert_eq!(c.pulse_us, 750.0);
        assert_eq!(c.current_a, 1.5);
        assert_eq!(c.duty_pct, 30.0);
    }

    #[test]
    fn m4_with_partial_parameters() {
        // Tool-positive; P omitted keeps the default, Q/R override.
        let Command::Pulser(c) = parse(b"M4 Q2.0 R25").unwrap() else {
            panic!("expected Pulser");
        };
        assert!(!c.tool_negative);
        assert_eq!(c.pulse_us, 500.0);
        assert_eq!(c.current_a, 2.0);
        assert_eq!(c.duty_pct, 25.0);
    }

    #[test]
    fn m3_mixed_parameters() {
        let Command::Pulser(c) = parse(b"M3 P1000 R50").unwrap() else {
            panic!("expected Pulser");
        };
        assert_eq!(c.pulse_us, 1000.0);
        assert_eq!(c.current_a, 1.0);
        assert_eq!(c.duty_pct, 50.0);
    }

    #[test]
    fn m3_bare_param_fails() {
        assert!(parse(b"M3 P").is_err());
    }

    #[test]
    fn m3_unknown_param_fails() {
        assert!(parse(b"M3 P500 S100").is_err());
    }

    #[test]
    fn basic_g0_command() {
        let cmd = parse(b"G0").unwrap();
        assert_eq!(cmd, Command::Rapid(MoveSpec::default()));
    }

    #[test]
    fn g1_with_coordinates() {
        let Command::Linear(s) = parse(b"G1 X10.5 Y-20.3 Z5").unwrap() else {
            panic!("expected Linear");
        };
        assert_eq!(s.x, Some(10.5));
        assert_eq!(s.y, Some(-20.3));
        assert_eq!(s.z, Some(5.0));
    }

    #[test]
    fn g0_with_c_axis() {
        // C parser stores c in degrees; our parser converts to turns at parse time.
        let Command::Rapid(s) = parse(b"G0 X10 Y20 C45.5").unwrap() else {
            panic!("expected Rapid");
        };
        assert_eq!(s.x, Some(10.0));
        assert_eq!(s.y, Some(20.0));
        assert_eq!(s.c, Some(45.5 / 360.0));
    }

    #[test]
    fn g1_with_all_axes() {
        // C: c=90 (degrees). Rust: c=0.25 (turns).
        let Command::Linear(s) = parse(b"G1 X1.5 Y2.5 Z3.5 C90").unwrap() else {
            panic!("expected Linear");
        };
        assert_eq!(s.x, Some(1.5));
        assert_eq!(s.y, Some(2.5));
        assert_eq!(s.z, Some(3.5));
        assert_eq!(s.c, Some(0.25));
    }

    #[test]
    fn g38_3_command() {
        let Command::Probe(s) = parse(b"G38.3").unwrap() else {
            panic!("expected Probe");
        };
        assert_eq!(s, MoveSpec::default());
    }

    #[test]
    fn g38_3_with_target() {
        let Command::Probe(s) = parse(b"G38.3 Z-5").unwrap() else {
            panic!("expected Probe");
        };
        assert_eq!(s.z, Some(-5.0));
    }

    #[test]
    fn g38_2_unsupported() {
        // Only G38.3 is handled; G38.2 and bare G38 are unknown.
        assert_eq!(parse(b"G38.2"), Err(ParseError::UnknownCommand));
        assert_eq!(parse(b"G38"), Err(ParseError::UnknownCommand));
    }

    #[test]
    fn g28_axis_only() {
        let Command::Home(a) = parse(b"G28 X").unwrap() else {
            panic!("expected Home");
        };
        assert_eq!(
            a,
            HomeAxes {
                x: true,
                ..Default::default()
            }
        );
    }

    #[test]
    fn g28_c_axis() {
        let Command::Home(a) = parse(b"G28 C").unwrap() else {
            panic!("expected Home");
        };
        assert!(a.c && !a.x && !a.y && !a.z);
    }

    #[test]
    fn g28_home_all() {
        assert_eq!(parse(b"G28").unwrap(), Command::Home(HomeAxes::default()));
    }

    #[test]
    fn g28_rejects_value() {
        assert!(parse(b"G28 X10").is_err());
    }

    #[test]
    fn empty_string() {
        assert_eq!(parse(b""), Err(ParseError::Empty));
    }

    #[test]
    fn whitespace_only() {
        assert_eq!(parse(b"   "), Err(ParseError::Empty));
    }

    #[test]
    fn extra_whitespace_success() {
        let Command::Rapid(s) = parse(b"G0   X10.5    Y20").unwrap() else {
            panic!("expected Rapid");
        };
        assert_eq!(s.x, Some(10.5));
        assert_eq!(s.y, Some(20.0));
    }

    #[test]
    fn lowercase_command_fails() {
        assert!(parse(b"g0 X10").is_err());
    }

    #[test]
    fn lowercase_parameter_fails() {
        assert!(parse(b"G0 x10").is_err());
    }

    #[test]
    fn garbled_command_fails() {
        assert!(parse(b"G0abc X10").is_err());
    }

    #[test]
    fn garbled_number_fails() {
        assert!(parse(b"G0 X10.5.2").is_err());
    }

    #[test]
    fn no_whitespace_between_params_fails() {
        assert!(parse(b"G0X1Y2").is_err());
    }
}
