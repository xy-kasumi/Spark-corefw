// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Single digital output pin.

pub trait DigitalOutput {
    /// Drive the output high (`true`) or low (`false`).
    fn set(&mut self, high: bool);
}
