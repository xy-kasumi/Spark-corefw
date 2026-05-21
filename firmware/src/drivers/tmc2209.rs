// SPDX-FileCopyrightText: 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! TMC2209 wire protocol: CRC, datagram framing, register read/write with verify.
//!
//! Generic over any half-duplex byte transport (write, write-then-read).
#![allow(dead_code)]

// --- Transport ---------------------------------------------------------------

pub trait TmcTransport {
    type Error;
    async fn write(&mut self, data: &[u8]) -> Result<(), Self::Error>;
    async fn write_then_read(&mut self, tx: &[u8], rx: &mut [u8]) -> Result<(), Self::Error>;
}

// --- Register addresses (subset) --------------------------------------------

pub const REG_GCONF: u8 = 0x00;
pub const REG_IFCNT: u8 = 0x02;
pub const REG_IOIN: u8 = 0x06;
pub const REG_IHOLD_IRUN: u8 = 0x10;
pub const REG_TCOOLTHRS: u8 = 0x14;
pub const REG_SGTHRS: u8 = 0x40;
pub const REG_SG_RESULT: u8 = 0x41;
pub const REG_COOLCONF: u8 = 0x42;
pub const REG_CHOPCONF: u8 = 0x6C;

// --- Datagram framing --------------------------------------------------------

const SYNC_BYTE: u8 = 0x05;
const NODE_ADDR_BROADCAST: u8 = 0x00;
const MASTER_ADDR: u8 = 0xFF;
const WRITE_BIT_MASK: u8 = 0x80;
const REG_ADDR_MASK: u8 = 0x7F;

const SETTLE_MS: u64 = 10;

/// TMC UART CRC: poly 0x07, MSB-out, processes data bits LSB-first per byte.
pub fn crc(data: &[u8]) -> u8 {
    let mut crc: u8 = 0;
    for &byte in data {
        let mut b = byte;
        for _ in 0..8 {
            if ((crc >> 7) ^ (b & 1)) != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
            b >>= 1;
        }
    }
    crc
}

pub fn encode_read(reg: u8) -> [u8; 4] {
    let mut buf = [0u8; 4];
    buf[0] = SYNC_BYTE;
    buf[1] = NODE_ADDR_BROADCAST;
    buf[2] = reg & REG_ADDR_MASK;
    buf[3] = crc(&buf[..3]);
    buf
}

pub fn encode_write(reg: u8, val: u32) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0] = SYNC_BYTE;
    buf[1] = NODE_ADDR_BROADCAST;
    buf[2] = (reg & REG_ADDR_MASK) | WRITE_BIT_MASK;
    buf[3..7].copy_from_slice(&val.to_be_bytes());
    buf[7] = crc(&buf[..7]);
    buf
}

#[derive(Debug)]
pub enum Error<E> {
    Transport(E),
    ReplyBadSync,
    ReplyBadMaster,
    ReplyRegMismatch,
    ReplyCrc,
    WriteVerifyFailed,
}

fn parse_reply<E>(buf: &[u8; 8], expected_reg: u8) -> Result<u32, Error<E>> {
    if buf[0] != SYNC_BYTE {
        return Err(Error::ReplyBadSync);
    }
    if buf[1] != MASTER_ADDR {
        return Err(Error::ReplyBadMaster);
    }
    if (buf[2] & REG_ADDR_MASK) != (expected_reg & REG_ADDR_MASK) {
        return Err(Error::ReplyRegMismatch);
    }
    if buf[7] != crc(&buf[..7]) {
        return Err(Error::ReplyCrc);
    }
    Ok(u32::from_be_bytes([buf[3], buf[4], buf[5], buf[6]]))
}

// --- Driver ------------------------------------------------------------------

pub struct Device<T: TmcTransport> {
    transport: T,
}

impl<T: TmcTransport> Device<T> {
    pub fn new(transport: T) -> Self {
        Self { transport }
    }

    pub async fn read_reg(&mut self, addr: u8) -> Result<u32, Error<T::Error>> {
        let req = encode_read(addr);
        let mut reply = [0u8; 8];
        self.transport
            .write_then_read(&req, &mut reply)
            .await
            .map_err(Error::Transport)?;
        let v = parse_reply::<T::Error>(&reply, addr)?;
        embassy_time::Timer::after(embassy_time::Duration::from_millis(SETTLE_MS)).await;
        Ok(v)
    }

    pub async fn write_reg(&mut self, addr: u8, val: u32) -> Result<(), Error<T::Error>> {
        // check IFCNT before & after to simplify setup by spending more read time.
        // note: this prevents gotcha of firmware rewrite w/o board power cycle (IFCNT stays non-zero)
        let before = self.read_reg(REG_IFCNT).await? as u8;
        let req = encode_write(addr, val);
        self.transport.write(&req).await.map_err(Error::Transport)?;
        embassy_time::Timer::after(embassy_time::Duration::from_millis(SETTLE_MS)).await;
        let after = self.read_reg(REG_IFCNT).await? as u8;
        if after == before.wrapping_add(1) {
            Ok(())
        } else {
            Err(Error::WriteVerifyFailed)
        }
    }

    /// `microstep` must be a power of 2 in 1..=256.
    pub async fn set_microstep(&mut self, microstep: u32) -> Result<(), Error<T::Error>> {
        // GCONF.mstep_reg_select = 1: take MRES from CHOPCONF rather than MS1/MS2 pins.
        let mut gconf = self.read_reg(REG_GCONF).await?;
        gconf |= 1 << 7;
        self.write_reg(REG_GCONF, gconf).await?;

        // MRES field encoding: 0=256, 1=128, ..., 8=1 µstep.
        let mres_bits = 8 - microstep.trailing_zeros();

        let mut chopconf = self.read_reg(REG_CHOPCONF).await?;
        chopconf &= 0xF0FF_FFFF; // clear MRES[27:24]
        chopconf |= mres_bits << 24;
        self.write_reg(REG_CHOPCONF, chopconf).await
    }

    /// Set run + hold current as 0..=100 percent (mapped to IRUN/IHOLD 0..31).
    /// IHOLDDELAY is fixed at the datasheet-recommended 10.
    pub async fn set_current(
        &mut self,
        run_percent: u32,
        hold_percent: u32,
    ) -> Result<(), Error<T::Error>> {
        let irun = (run_percent * 31 + 50) / 100;
        let ihold = (hold_percent * 31 + 50) / 100;
        let ihold_delay: u32 = 10;
        let reg = (ihold_delay << 16) | (irun << 8) | ihold;
        self.write_reg(REG_IHOLD_IRUN, reg).await
    }

    pub async fn set_tcoolthrs(&mut self, value: u32) -> Result<(), Error<T::Error>> {
        self.write_reg(REG_TCOOLTHRS, value & 0x000F_FFFF).await
    }

    pub async fn set_stallguard_threshold(&mut self, threshold: u8) -> Result<(), Error<T::Error>> {
        self.write_reg(REG_SGTHRS, threshold as u32).await
    }
}
