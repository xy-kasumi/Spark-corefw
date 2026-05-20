// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Host signal lines (`!`, `?xxx`): enums and parser.

/// Query-like signal (?...)
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum QuerySignal {
    Queue,
    Pos,
    Edm,
    /// Recognized `?` byte but unknown content.
    Unknown,
}

/// One signal (! or ?...)
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Signal {
    Cancel,
    Query(QuerySignal),
}

/// note: `bytes` must include "!" or "?", but not whitespaces.
pub fn parse(bytes: &[u8]) -> Signal {
    match bytes {
        b"!" => Signal::Cancel,
        b"?queue" => Signal::Query(QuerySignal::Queue),
        b"?pos" => Signal::Query(QuerySignal::Pos),
        b"?edm" => Signal::Query(QuerySignal::Edm),
        _ => Signal::Query(QuerySignal::Unknown),
    }
}
