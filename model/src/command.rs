// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

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
    /// `stat` — dump per-module debug status as one `stat` p-state.
    Stat,
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
        "stat" => parse_stat(rest),
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

fn is_ws(c: char) -> bool {
    c == ' ' || c == '\t'
}

fn split_word(s: &str) -> (&str, &str) {
    let trimmed = s.trim_start_matches(is_ws);
    let end = trimmed.find(is_ws).unwrap_or(trimmed.len());
    (&trimmed[..end], &trimmed[end..])
}
