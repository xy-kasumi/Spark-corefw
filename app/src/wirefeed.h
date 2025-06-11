// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) Wire feeding controller for motor6.
 */
#pragma once

/**
 * (blocking) Initialize wirefeed subsystem.
 */
void wirefeed_init();

/**
 * Start feeding wire at specified rate.
 * @param feedrate_mm_per_min Feed rate in mm/min
 */
void wirefeed_start(float feedrate_mm_per_min);

/**
 * Stop wire feeding.
 */
void wirefeed_stop();

/**
 * Set motor6 unitsteps.
 * @param unitsteps Steps per mm for motor6
 */
void wirefeed_set_unitsteps(float unitsteps);

/**
 * (blocking) Dump wirefeed subsystem status for debugging.
 */
void wirefeed_dump_status();
