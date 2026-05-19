//! Host-side command parsing. Bytes arrive from `comm::Framer`; this module
//! turns them into the typed `Command` the executor queues. Sits in `model`
//! so it can be host-fuzzed without firmware deps.
//!
//! Errors are categorical (one variant per failure shape); the wire-format
//! error message comes from Debug printing in the caller.

use crate::gcode;
use crate::settings::SettingId;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Command {
    Gcode(gcode::Command),
    /// `set <key> <value>`. Path resolved to a typed id at parse time; value
    /// is finite-checked. Apply-side validation is the executor's problem.
    Set(SettingId, f32),
    /// `get` — dump all settings as one `stg` p-state.
    Get,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParseError {
    Empty,
    UnknownCommand,
    Gcode(gcode::ParseError),
    SetMissingKey,
    SetMissingValue,
    SetUnknownKey,
    SetBadValue,
    GetExtraArgs,
}

pub fn parse(bytes: &[u8]) -> Result<Command, ParseError> {
    if bytes.is_empty() {
        return Err(ParseError::Empty);
    }
    match bytes[0] {
        b'G' | b'M' => gcode::parse(bytes)
            .map(Command::Gcode)
            .map_err(ParseError::Gcode),
        _ => parse_text(bytes),
    }
}

fn parse_text(bytes: &[u8]) -> Result<Command, ParseError> {
    let s = core::str::from_utf8(bytes).map_err(|_| ParseError::UnknownCommand)?;
    let (word, rest) = split_word(s);
    match word {
        "set" => parse_set(rest),
        "get" => parse_get(rest),
        _ => Err(ParseError::UnknownCommand),
    }
}

fn parse_set(rest: &str) -> Result<Command, ParseError> {
    let (key, rest) = split_word(rest);
    if key.is_empty() {
        return Err(ParseError::SetMissingKey);
    }
    let (value_str, tail) = split_word(rest);
    if value_str.is_empty() {
        return Err(ParseError::SetMissingValue);
    }
    if !tail.trim_start_matches(is_ws).is_empty() {
        return Err(ParseError::SetBadValue);
    }
    let id = SettingId::parse(key).ok_or(ParseError::SetUnknownKey)?;
    let value: f32 = value_str.parse().map_err(|_| ParseError::SetBadValue)?;
    if !value.is_finite() {
        return Err(ParseError::SetBadValue);
    }
    Ok(Command::Set(id, value))
}

fn parse_get(rest: &str) -> Result<Command, ParseError> {
    if !rest.trim_start_matches(is_ws).is_empty() {
        return Err(ParseError::GetExtraArgs);
    }
    Ok(Command::Get)
}

fn is_ws(c: char) -> bool {
    c == ' ' || c == '\t'
}

fn split_word(s: &str) -> (&str, &str) {
    let trimmed = s.trim_start_matches(is_ws);
    let end = trimmed.find(is_ws).unwrap_or(trimmed.len());
    (&trimmed[..end], &trimmed[end..])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::SettingId;

    #[test]
    fn empty_is_empty_error() {
        assert_eq!(parse(b""), Err(ParseError::Empty));
    }

    #[test]
    fn gcode_dispatches() {
        assert!(matches!(parse(b"G0 X1"), Ok(Command::Gcode(_))));
        assert!(matches!(parse(b"G99"), Err(ParseError::Gcode(_))));
    }

    #[test]
    fn set_basic() {
        let got = parse(b"set m.0.microstep 32").unwrap();
        assert_eq!(got, Command::Set(SettingId::MotorMicrostep(0), 32.0));
    }

    #[test]
    fn set_extra_spaces_ok() {
        let got = parse(b"set   m.0.microstep   32").unwrap();
        assert_eq!(got, Command::Set(SettingId::MotorMicrostep(0), 32.0));
    }

    #[test]
    fn set_negative_value() {
        let got = parse(b"set m.1.unitsteps -200").unwrap();
        assert_eq!(got, Command::Set(SettingId::MotorUnitsteps(1), -200.0));
    }

    #[test]
    fn set_missing_key() {
        assert_eq!(parse(b"set"), Err(ParseError::SetMissingKey));
        assert_eq!(parse(b"set   "), Err(ParseError::SetMissingKey));
    }

    #[test]
    fn set_missing_value() {
        assert_eq!(parse(b"set m.0.current"), Err(ParseError::SetMissingValue));
        assert_eq!(
            parse(b"set m.0.current   "),
            Err(ParseError::SetMissingValue)
        );
    }

    #[test]
    fn set_unknown_key() {
        assert_eq!(parse(b"set bogus.x 1"), Err(ParseError::SetUnknownKey));
        assert_eq!(parse(b"set m.99.current 1"), Err(ParseError::SetUnknownKey));
    }

    #[test]
    fn set_bad_value() {
        assert_eq!(
            parse(b"set m.0.current notnumeric"),
            Err(ParseError::SetBadValue)
        );
        assert_eq!(
            parse(b"set m.0.current 1.0.0"),
            Err(ParseError::SetBadValue)
        );
        // Rust's f32::from_str accepts "nan"/"inf" but write() must not.
        assert_eq!(parse(b"set m.0.current nan"), Err(ParseError::SetBadValue));
        assert_eq!(parse(b"set m.0.current inf"), Err(ParseError::SetBadValue));
        assert_eq!(parse(b"set m.0.current -inf"), Err(ParseError::SetBadValue));
    }

    #[test]
    fn set_trailing_garbage() {
        assert_eq!(
            parse(b"set m.0.current 1 extra"),
            Err(ParseError::SetBadValue)
        );
    }

    #[test]
    fn set_trailing_whitespace_ok() {
        let got = parse(b"set m.0.current 5   ").unwrap();
        assert_eq!(got, Command::Set(SettingId::MotorCurrent(0), 5.0));
    }

    #[test]
    fn get_basic() {
        assert_eq!(parse(b"get"), Ok(Command::Get));
        assert_eq!(parse(b"get  "), Ok(Command::Get));
    }

    #[test]
    fn get_extra_args() {
        assert_eq!(parse(b"get foo"), Err(ParseError::GetExtraArgs));
    }

    #[test]
    fn unknown_command() {
        assert_eq!(parse(b"foo bar"), Err(ParseError::UnknownCommand));
        assert_eq!(parse(b"stat"), Err(ParseError::UnknownCommand));
    }
}
