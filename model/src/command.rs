// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Parser for line content from host (command, signal, G-code).

use crate::gcode;
use crate::settings;

/// Query-like signal (?...)
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum QuerySignal {
    Queue,
    Pos,
    Edm,
    /// Recognized `?` byte but unknown content.
    Unknown,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Command {
    Gcode(gcode::Parsed),
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
    /// `ov.pump_en`: true forces the pump on; false lets G-code control it.
    PumpEn(bool),
}

/// Outcome of parsing a framed line. The caller dispatches each variant down
/// its own path: cancel/query/fast-set are immediate; command goes through
/// the queue; error emits a diagnostic.
#[derive(Clone, Debug, PartialEq)]
pub enum Parsed {
    Cancel,
    Query(QuerySignal),
    Command(Command),
    FastSet(FastSet),
    Error,
}

pub fn parse(bytes: &[u8]) -> Parsed {
    match bytes.first() {
        Some(b'!') | Some(b'?') => match bytes {
            b"!" => Parsed::Cancel,
            b"?queue" => Parsed::Query(QuerySignal::Queue),
            b"?pos" => Parsed::Query(QuerySignal::Pos),
            b"?edm" => Parsed::Query(QuerySignal::Edm),
            _ => Parsed::Query(QuerySignal::Unknown),
        },
        Some(b'G') | Some(b'M') => match gcode::parse(bytes) {
            Some(c) => Parsed::Command(Command::Gcode(c)),
            None => Parsed::Error,
        },
        _ => parse_text(bytes).unwrap_or(Parsed::Error),
    }
}

fn parse_text(bytes: &[u8]) -> Option<Parsed> {
    let s = core::str::from_utf8(bytes).ok()?;
    let (word, rest) = split_word(s);
    match word {
        "set" => parse_set(rest).map(Parsed::Command),
        "get" => parse_get(rest).map(Parsed::Command),
        "stat" => parse_stat(rest).map(Parsed::Command),
        "fset" => parse_fset(rest).map(Parsed::FastSet),
        _ => None,
    }
}

fn parse_set(rest: &str) -> Option<Command> {
    let (key, rest) = split_word(rest);
    if key.is_empty() {
        return None;
    }
    let (value_str, tail) = split_word(rest);
    if value_str.is_empty() {
        return None;
    }
    if !tail.trim_start_matches(is_ws).is_empty() {
        return None;
    }
    let key = heapless::String::<{ settings::STG_KEY_CAP }>::try_from(key).ok()?;
    let value = settings::Value::parse(value_str)?;
    Some(Command::Set(key, value))
}

fn parse_get(rest: &str) -> Option<Command> {
    if !rest.trim_start_matches(is_ws).is_empty() {
        return None;
    }
    Some(Command::Get)
}

fn parse_stat(rest: &str) -> Option<Command> {
    if !rest.trim_start_matches(is_ws).is_empty() {
        return None;
    }
    Some(Command::Stat)
}

fn parse_fset(rest: &str) -> Option<FastSet> {
    let (key, rest) = split_word(rest);
    if key.is_empty() {
        return None;
    }
    let (value_str, tail) = split_word(rest);
    if value_str.is_empty() {
        return None;
    }
    if !tail.trim_start_matches(is_ws).is_empty() {
        return None;
    }
    let value = parse_bool(value_str)?;
    match key {
        "ov.pump_en" => Some(FastSet::PumpEn(value)),
        _ => None,
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
            Parsed::FastSet(FastSet::PumpEn(true))
        );
        assert_eq!(
            parse(b"fset ov.pump_en false"),
            Parsed::FastSet(FastSet::PumpEn(false))
        );
    }
}
