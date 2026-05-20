//! Host signal lines (`!`, `?xxx`): the typed enum and byte-level parser.
//! Execution lives firmware-side since handlers touch hardware state.

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Signal {
    /// `!` — cancel motion and drop queued commands.
    Cancel,
    /// `?queue` — emit queue capacity + outstanding count.
    QueryQueue,
    /// `?pos` — emit current machine position.
    QueryPos,
    /// `?edm` — emit EDM/move telemetry.
    QueryEdm,
    /// Recognized signal byte but unknown verb; silently ignored.
    Unknown,
}

/// Classify a framed signal line. `bytes` includes the leading `!` or `?`.
pub fn parse(bytes: &[u8]) -> Signal {
    match bytes {
        b"!" => Signal::Cancel,
        b"?queue" => Signal::QueryQueue,
        b"?pos" => Signal::QueryPos,
        b"?edm" => Signal::QueryEdm,
        _ => Signal::Unknown,
    }
}
