// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) EDM pulser board interface
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/** (blocking) Initialize pulser subsystem */
bool pulser_init();

/** (blocking) Dump pulser status for debugging */
void pulser_dump_status();

/**
 * (blocking) Energize pulser with specified parameters
 * @param negative true for tool negative, false for tool positive
 * @param pulse_us pulse duration in microseconds (100-1000)
 * @param current_a pulse current in amperes (0-20). 0 means minimum possible
 * current.
 * @param duty_pct duty cycle percentage (1-95)
 */
void pulser_energize(bool negative,
                     float pulse_us,
                     float current_a,
                     float duty_pct);

/** (blocking)  De-energize pulser */
void pulser_deenergize();

/**
 * Get latest short rate from EDM polling
 * @return short rate (0-255), typically >127 indicates retraction needed
 */
uint8_t pulser_get_short_rate();

/**
 * Get latest pulse rate from EDM polling
 * @return pulse rate (0-255).
 */
uint8_t pulser_get_pulse_rate();

/**
 * Get latest open rate from EDM polling
 * @return open rate (0-255)
 */
uint8_t pulser_get_open_rate();

/**
 * Check if there is active discharge (pulse or short)
 * @return true if r_pulse > 0 or r_short > 0
 */
bool pulser_has_discharge();
