// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Host-side parsing of non-signal lines. Bytes arrive from `comm::Framer`;
//! this module turns them into an [`Outcome`]: either a queued [`Command`] or
//! an immediate [`FastSet`]. Sits in `model` so it can be host-fuzzed without
//! firmware deps.
use crate::gcode;
use crate::settings;

#[derive(Clone, Debug, PartialEq)]
pub enum Command {
    Gcode(gcode::Command),
    /// `set` - Set single (key, val)
    Set(heapless::String<{ settings::STG_KEY_CAP }>, settings::Value),
    /// `get` - dump all settings as one `stg` p-state.
    Get,
    /// `stat` - dump per-module debug status as one `stat` p-state.
    Stat,
}

/// `fset <key> <value>`: an unqueued "fast set" override (see protocol.md).
/// Applied immediately in the tick loop, bypassing the command queue.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FastSet {
    /// `ov.pump_en`: true forces the pump on; false lets M8/M9 control it.
    PumpEn(bool),
}

/// Result of parsing a non-signal line: a queued command, or an immediate
/// fast-set. The caller dispatches each down its own path.
#[derive(Clone, Debug, PartialEq)]
pub enum Outcome {
    Command(Command),
    FastSet(FastSet),
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ParseError {
    Empty,
    UnknownCommand,
    Gcode(gcode::ParseError),
    Syntax,
}

pub fn parse(bytes: &[u8]) -> Result<Outcome, ParseError> {
    if bytes.is_empty() {
        return Err(ParseError::Empty);
    }
    match bytes[0] {
        b'G' | b'M' => gcode::parse(bytes)
            .map(|c| Outcome::Command(Command::Gcode(c)))
            .map_err(ParseError::Gcode),
        _ => parse_text(bytes),
    }
}

fn parse_text(bytes: &[u8]) -> Result<Outcome, ParseError> {
    let s = core::str::from_utf8(bytes).map_err(|_| ParseError::UnknownCommand)?;
    let (word, rest) = split_word(s);
    match word {
        "set" => parse_set(rest).map(Outcome::Command),
        "get" => parse_get(rest).map(Outcome::Command),
        "stat" => parse_stat(rest).map(Outcome::Command),
        "fset" => parse_fset(rest).map(Outcome::FastSet),
        _ => Err(ParseError::UnknownCommand),
    }
}

fn parse_set(rest: &str) -> Result<Command, ParseError> {
    let (key, rest) = split_word(rest);
    if key.is_empty() {
        return Err(ParseError::Syntax);
    }
    let (value_str, tail) = split_word(rest);
    if value_str.is_empty() {
        return Err(ParseError::Syntax);
    }
    if !tail.trim_start_matches(is_ws).is_empty() {
        return Err(ParseError::Syntax);
    }
    let key = heapless::String::<{ settings::STG_KEY_CAP }>::try_from(key)
        .map_err(|_| ParseError::Syntax)?;
    let value = settings::Value::parse(value_str).ok_or(ParseError::Syntax)?;
    Ok(Command::Set(key, value))
}

fn parse_get(rest: &str) -> Result<Command, ParseError> {
    if !rest.trim_start_matches(is_ws).is_empty() {
        return Err(ParseError::Syntax);
    }
    Ok(Command::Get)
}

fn parse_stat(rest: &str) -> Result<Command, ParseError> {
    if !rest.trim_start_matches(is_ws).is_empty() {
        return Err(ParseError::Syntax);
    }
    Ok(Command::Stat)
}

fn parse_fset(rest: &str) -> Result<FastSet, ParseError> {
    let (key, rest) = split_word(rest);
    if key.is_empty() {
        return Err(ParseError::Syntax);
    }
    let (value_str, tail) = split_word(rest);
    if value_str.is_empty() {
        return Err(ParseError::Syntax);
    }
    if !tail.trim_start_matches(is_ws).is_empty() {
        return Err(ParseError::Syntax);
    }
    let value = parse_bool(value_str).ok_or(ParseError::Syntax)?;
    match key {
        "ov.pump_en" => Ok(FastSet::PumpEn(value)),
        _ => Err(ParseError::Syntax),
    }
}

fn parse_bool(s: &str) -> Option<bool> {
    match s {
        "true" => Some(true),
        "false" => Some(false),
        _ => None,
    }
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

    #[test]
    fn fset_pump_en() {
        assert_eq!(
            parse(b"fset ov.pump_en true"),
            Ok(Outcome::FastSet(FastSet::PumpEn(true)))
        );
        assert_eq!(
            parse(b"fset ov.pump_en false"),
            Ok(Outcome::FastSet(FastSet::PumpEn(false)))
        );
    }
}
