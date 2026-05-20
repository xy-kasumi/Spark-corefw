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
