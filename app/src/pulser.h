// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) EDM pulser board interface
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/** Initialize pulser subsystem */
void pulser_init();

/** Dump pulser status for debugging */
void pulser_dump_status();

/**
 * Energize pulser with specified parameters
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

/** De-energize pulser (M5) */
void pulser_deenergize();

/**
 * Get latest short rate from EDM polling
 * @return short rate (0-255), typically >127 indicates retraction needed
 */
uint8_t pulser_get_short_rate();

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

/**
 * Get number of entries currently stored in EDM buffer
 * @return number of entries (0 to 10000)
 */
uint32_t pulser_get_buffer_count();

/**
 * Get EDM polling data entry from buffer
 * @param index entry index (0 = oldest entry still in buffer)
 * @param r_short pointer to store r_short value
 * @param r_open pointer to store r_open value
 * @param num_pulse pointer to store num_pulse value
 * @return true if entry exists, false if index out of range
 */
bool pulser_get_buffer_entry(uint32_t index,
                             uint8_t* r_short,
                             uint8_t* r_open,
                             uint8_t* num_pulse);

/**
 * Clear all entries from EDM buffer
 */
void pulser_clear_buffer();

/**
 * Copy EDM buffer data to download buffer
 * @param buffer destination buffer
 * @param max_size maximum size of destination buffer
 * @return number of bytes copied
 */
uint32_t pulser_copy_log_to_buffer(uint8_t* buffer, uint32_t max_size);
