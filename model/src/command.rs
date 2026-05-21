// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Host-side parsing of non-signal lines. Bytes arrive from `comm::Framer`;
//! this module turns them into an [`Outcome`]: either a queued [`Command`] or
//! an immediate [`FastSet`]. Sits in `model` so it can be host-fuzzed without
//! firmware deps.
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
    /// `stat` — dump per-module debug status as one `stat` p-state.
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
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Outcome {
    Command(Command),
    FastSet(FastSet),
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
    StatExtraArgs,
    FsetMissingKey,
    FsetMissingValue,
    FsetUnknownKey,
    FsetBadValue,
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

fn parse_stat(rest: &str) -> Result<Command, ParseError> {
    if !rest.trim_start_matches(is_ws).is_empty() {
        return Err(ParseError::StatExtraArgs);
    }
    Ok(Command::Stat)
}

fn parse_fset(rest: &str) -> Result<FastSet, ParseError> {
    let (key, rest) = split_word(rest);
    if key.is_empty() {
        return Err(ParseError::FsetMissingKey);
    }
    let (value_str, tail) = split_word(rest);
    if value_str.is_empty() {
        return Err(ParseError::FsetMissingValue);
    }
    if !tail.trim_start_matches(is_ws).is_empty() {
        return Err(ParseError::FsetBadValue);
    }
    let value = parse_bool(value_str).ok_or(ParseError::FsetBadValue)?;
    match key {
        "ov.pump_en" => Ok(FastSet::PumpEn(value)),
        _ => Err(ParseError::FsetUnknownKey),
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

    #[test]
    fn fset_errors() {
        assert_eq!(parse(b"fset"), Err(ParseError::FsetMissingKey));
        assert_eq!(parse(b"fset ov.pump_en"), Err(ParseError::FsetMissingValue));
        assert_eq!(parse(b"fset ov.pump_en yes"), Err(ParseError::FsetBadValue));
        assert_eq!(
            parse(b"fset ov.pump_en true extra"),
            Err(ParseError::FsetBadValue)
        );
        assert_eq!(
            parse(b"fset ov.unknown true"),
            Err(ParseError::FsetUnknownKey)
        );
    }
}
