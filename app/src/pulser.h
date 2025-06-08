// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) EDM pulser board interface
 */
#pragma once

#include <stdbool.h>

/** Initialize pulser subsystem */
void pulser_init();

/** Dump pulser status for debugging */
void pulser_dump_status();

/**
 * Energize pulser with specified parameters
 * @param negative true for tool negative, false for tool positive
 * @param pulse_us pulse duration in microseconds (100-1000)
 * @param current_a pulse current in amperes (0-20). 0 means minimum possible current.
 * @param duty_pct duty cycle percentage (1-95)
 */
void pulser_energize(bool negative,
                     float pulse_us,
                     float current_a,
                     float duty_pct);

/** De-energize pulser (M5) */
void pulser_deenergize();
