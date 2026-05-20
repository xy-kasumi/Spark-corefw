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
    BadAxis,
    BadNumber,
    ExpectedSeparator,
    TrailingGarbage,
}

pub fn parse(line: &[u8]) -> Result<Command, ParseError> {
    let mut p = Cursor::new(line);
    p.skip_ws();
    let (letter, code) = p.read_letter_int().ok_or(ParseError::Empty)?;
    if letter != b'G' {
        return Err(ParseError::UnknownCommand);
    }
    let sub = p.read_subcode()?;
    match (code, sub) {
        (0, None) => parse_move(&mut p).map(Command::Rapid),
        (1, None) => parse_move(&mut p).map(Command::Linear),
        (28, None) => parse_home(&mut p).map(Command::Home),
        (38, Some(3)) => parse_move(&mut p).map(Command::Probe),
        (53..=56, None) => parse_select(&mut p, code),
        _ => Err(ParseError::UnknownCommand),
    }
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
            return Err(ParseError::ExpectedSeparator);
        }
        let letter = p.read_letter().ok_or(ParseError::BadAxis)?;
        match letter {
            b'X' => axes.x = true,
            b'Y' => axes.y = true,
            b'Z' => axes.z = true,
            b'C' => axes.c = true,
            _ => return Err(ParseError::BadAxis),
        }
        // Bare letters only: a digit (value) following an axis is invalid.
        if !p.eof_or_only_ws() && !p.at_ws() {
            return Err(ParseError::TrailingGarbage);
        }
    }
    Ok(axes)
}

/// Parse a coordinate-system select (G53-G56). Takes no parameters.
fn parse_select(p: &mut Cursor, code: i32) -> Result<Command, ParseError> {
    if !p.eof_or_only_ws() {
        return Err(ParseError::TrailingGarbage);
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
            return Err(ParseError::ExpectedSeparator);
        }
        let letter = p.read_letter().ok_or(ParseError::BadAxis)?;
        let value = p.read_float().ok_or(ParseError::BadNumber)?;
        match letter {
            b'X' => spec.x = Some(value),
            b'Y' => spec.y = Some(value),
            b'Z' => spec.z = Some(value),
            b'C' => spec.c = Some(value / 360.0),
            _ => return Err(ParseError::BadAxis),
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
        self.read_int().map(Some).ok_or(ParseError::BadNumber)
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
    //! Tests mirror tests/app/src/gcode_base_test.c. M-code tests stay deferred
    //! until the Command enum gains M variants.

    use super::*;

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
