// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Single PWM output channel: fixed-period with a settable duty cycle.

pub trait PwmOutput {
    /// Set the carrier period (milliseconds) and enable the output.
    fn init(&mut self, period_ms: f32);
    /// Set the on-time as a duty fraction in `[0, 1]` of the carrier period.
    fn set(&mut self, duty: f32);
}
