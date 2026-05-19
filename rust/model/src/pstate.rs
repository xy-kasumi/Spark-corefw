//! P-state line builder. One Line value = one wire payload (no LF).
//!
//! Each builder method appends an element with the spec-mandated leading space.
//! Output never exceeds [`LINE_CAP`]; if the body would overflow the buffer,
//! further appends are dropped and [`Line::overflowed`] reports it.

use core::fmt::Write;

use heapless::Vec;

/// Spec caps the payload at 100 VCHAR; round up to a power of two.
pub const LINE_CAP: usize = 128;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PsType {
    Error,
    Queue,
    Pos,
    Edm,
    Init,
    Settings,
    Stat,
}

impl PsType {
    fn tag(self) -> &'static [u8] {
        match self {
            PsType::Error => b"error",
            PsType::Queue => b"queue",
            PsType::Pos => b"pos",
            PsType::Edm => b"edm",
            PsType::Init => b"init",
            PsType::Settings => b"stg",
            PsType::Stat => b"stat",
        }
    }
}

pub struct Line {
    buf: Vec<u8, LINE_CAP>,
    overflowed: bool,
}

impl Line {
    pub fn new(ps: PsType) -> Self {
        let mut me = Self {
            buf: Vec::new(),
            overflowed: false,
        };
        me.append(ps.tag());
        me
    }

    pub fn begin(mut self) -> Self {
        self.append(b" <");
        self
    }

    pub fn end(mut self) -> Self {
        self.append(b" >");
        self
    }

    pub fn bool(mut self, k: &str, v: bool) -> Self {
        self.write_key(k);
        self.append(if v { b"true" } else { b"false" });
        self
    }

    pub fn int(mut self, k: &str, v: i32) -> Self {
        self.write_key(k);
        let _ = write!(self, "{}", v);
        self
    }

    pub fn float(mut self, k: &str, v: f32) -> Self {
        self.write_key(k);
        let _ = write!(self, "{}", v);
        self
    }

    pub fn hex32(mut self, k: &str, v: u32) -> Self {
        self.write_key(k);
        let _ = write!(self, "0x{:08x}", v);
        self
    }

    pub fn str_val(mut self, k: &str, v: &str) -> Self {
        self.write_key(k);
        self.write_quoted(v.as_bytes());
        self
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.buf
    }

    pub fn overflowed(&self) -> bool {
        self.overflowed
    }

    fn append(&mut self, b: &[u8]) {
        if self.buf.extend_from_slice(b).is_err() {
            self.overflowed = true;
        }
    }

    fn write_key(&mut self, k: &str) {
        self.append(b" ");
        self.append(k.as_bytes());
        self.append(b":");
    }

    fn write_quoted(&mut self, v: &[u8]) {
        self.append(b"\"");
        for &b in v {
            if b == b'"' || b == b'\\' {
                self.append(b"\\");
            }
            self.append(&[b]);
        }
        self.append(b"\"");
    }
}

impl Write for Line {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.append(s.as_bytes());
        Ok(())
    }
}

/// Builder for the fixed-shape `error` line:
/// `error < [src:".."] msg:".." >`. Source is optional and capped at 50 bytes.
pub struct ErrorLine {
    line: Line,
}

impl ErrorLine {
    pub fn new() -> Self {
        Self {
            line: Line::new(PsType::Error).begin(),
        }
    }

    /// Attach the offending input line. Truncated to 50 bytes.
    pub fn source(mut self, src: &[u8]) -> Self {
        let truncated = &src[..src.len().min(50)];
        self.line.write_key("src");
        self.line.write_quoted(truncated);
        self
    }

    pub fn msg(mut self, args: core::fmt::Arguments<'_>) -> Self {
        self.line.write_key("msg");
        self.line.append(b"\"");
        let _ = self.line.write_fmt(args);
        self.line.append(b"\"");
        self
    }

    pub fn finish(self) -> Line {
        self.line.end()
    }
}

impl Default for ErrorLine {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn s(line: &Line) -> &str {
        core::str::from_utf8(line.as_bytes()).unwrap()
    }

    #[test]
    fn tag_only() {
        assert_eq!(s(&Line::new(PsType::Queue)), "queue");
    }

    #[test]
    fn one_shot_queue() {
        let l = Line::new(PsType::Queue)
            .begin()
            .int("cap", 100)
            .int("num", 5)
            .end();
        assert_eq!(s(&l), "queue < cap:100 num:5 >");
    }

    #[test]
    fn empty_pstate() {
        let l = Line::new(PsType::Settings).begin().end();
        assert_eq!(s(&l), "stg < >");
    }

    #[test]
    fn streaming_chunks_independent() {
        // Each builder call models one wire line; chunks don't carry state across.
        let open = Line::new(PsType::Init).begin();
        let pair = Line::new(PsType::Init).bool("motor.ok", true);
        let close = Line::new(PsType::Init).end();
        assert_eq!(s(&open), "init <");
        assert_eq!(s(&pair), "init motor.ok:true");
        assert_eq!(s(&close), "init >");
    }

    #[test]
    fn hex32_zero_pads() {
        let l = Line::new(PsType::Stat).hex32("r", 0x1234);
        assert_eq!(s(&l), "stat r:0x00001234");
    }

    #[test]
    fn str_val_escapes_quote_and_backslash() {
        let l = Line::new(PsType::Init).str_val("msg", r#"bad "quote" and \slash"#);
        assert_eq!(s(&l), r#"init msg:"bad \"quote\" and \\slash""#);
    }

    #[test]
    fn float_formats() {
        let l = Line::new(PsType::Pos).float("m.x", 1.5);
        assert_eq!(s(&l), "pos m.x:1.5");
    }

    #[test]
    fn bool_formats() {
        let t = Line::new(PsType::Init).bool("ok", true);
        let f = Line::new(PsType::Init).bool("ok", false);
        assert_eq!(s(&t), "init ok:true");
        assert_eq!(s(&f), "init ok:false");
    }

    #[test]
    fn error_with_source_and_msg() {
        let l = ErrorLine::new()
            .source(b"G99 X1")
            .msg(format_args!("unknown: {}", "G99"))
            .finish();
        assert_eq!(s(&l), r#"error < src:"G99 X1" msg:"unknown: G99" >"#);
    }

    #[test]
    fn error_msg_only() {
        let l = ErrorLine::new().msg(format_args!("boom")).finish();
        assert_eq!(s(&l), r#"error < msg:"boom" >"#);
    }

    #[test]
    fn error_source_truncated_to_50() {
        let long = [b'A'; 80];
        let l = ErrorLine::new()
            .source(&long)
            .msg(format_args!("x"))
            .finish();
        let body = s(&l);
        // Walk the literal we expect: 50 'A's between quotes.
        let expected = b"error < src:\"".len() + 50 + b"\" msg:\"x\" >".len();
        assert_eq!(body.len(), expected);
    }

    #[test]
    fn overflow_flag_set_when_too_big() {
        let mut l = Line::new(PsType::Stat).begin();
        for i in 0..50 {
            l = l.int("k", i);
        }
        assert!(l.overflowed());
        assert!(l.as_bytes().len() <= LINE_CAP);
    }
}
