// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Parser for line content from host (command, signal, G-code).

use crate::gcode;
use crate::settings;

/// Canonical parsed representation of single line from host.
#[derive(Clone, Debug, PartialEq)]
pub enum Parsed {
    /// `!`
    Cancel,
    /// `?...`
    Query(QuerySignal),
    /// `fset <key> <val>`
    FastSet(FastKey),
    Command(Command),
    Error,
}

/// Query-like signal (?...)
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum QuerySignal {
    Queue,
    Pos,
    Edm,
    /// Recognized `?` byte but unknown content.
    Unknown,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FastKey {
    /// `ov.pump_en`: true forces the pump on; false lets G-code control it.
    PumpEn(bool),
    /// `ov.edm.retr_thresh`
    EdmRetrThresh(Option<f32>),
    /// `ov.edm.adv_thresh`
    EdmAdvThresh(Option<f32>),
    /// `ov.edm.retr_speed`
    EdmRetrSpeed(Option<f32>),
    /// `ov.edm.adv_speed`
    EdmAdvSpeed(Option<f32>),
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

impl Command {
    pub fn is_write(&self) -> bool {
        match &self {
            Command::Get => false,
            Command::Stat => false,
            _ => true,
        }
    }
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

fn parse_fset(rest: &str) -> Option<FastKey> {
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
    let parse_thresh = |s| parse_float_or_none(s, |v| (0.0..=1.0).contains(&v));
    let parse_speed = |s| parse_float_or_none(s, |v| v > 0.0);
    match key {
        "ov.pump_en" => parse_bool(value_str).map(FastKey::PumpEn),
        "ov.edm.retr_thresh" => parse_thresh(value_str).map(FastKey::EdmRetrThresh),
        "ov.edm.adv_thresh" => parse_thresh(value_str).map(FastKey::EdmAdvThresh),
        "ov.edm.retr_speed" => parse_speed(value_str).map(FastKey::EdmRetrSpeed),
        "ov.edm.adv_speed" => parse_speed(value_str).map(FastKey::EdmAdvSpeed),
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

/// finite float that satisfies `pred`, or "none". (or error)
fn parse_float_or_none(s: &str, pred: impl Fn(f32) -> bool) -> Option<Option<f32>> {
    if s == "none" {
        return Some(None);
    }
    let v = s.parse::<f32>().ok()?;
    if !v.is_finite() {
        return None;
    }
    pred(v).then_some(Some(v))
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
            Parsed::FastSet(FastKey::PumpEn(true))
        );
        assert_eq!(
            parse(b"fset ov.pump_en false"),
            Parsed::FastSet(FastKey::PumpEn(false))
        );
    }

    #[test]
    fn fset_edm_accepts_none() {
        assert_eq!(
            parse(b"fset ov.edm.retr_thresh none"),
            Parsed::FastSet(FastKey::EdmRetrThresh(None))
        );
        assert_eq!(
            parse(b"fset ov.edm.adv_speed none"),
            Parsed::FastSet(FastKey::EdmAdvSpeed(None))
        );
    }

    #[test]
    fn fset_edm_rejects_thresh_outofrange() {
        assert_eq!(parse(b"fset ov.edm.retr_thresh 50"), Parsed::Error);
        assert_eq!(parse(b"fset ov.edm.adv_thresh 50"), Parsed::Error);
    }

    #[test]
    fn fset_edm_rejects_speed_outofrange() {
        assert_eq!(parse(b"fset ov.edm.retr_speed 0"), Parsed::Error);
        assert_eq!(parse(b"fset ov.edm.retr_speed -1"), Parsed::Error);
        assert_eq!(parse(b"fset ov.edm.adv_speed 0"), Parsed::Error);
        assert_eq!(parse(b"fset ov.edm.adv_speed -1"), Parsed::Error);
    }
}
