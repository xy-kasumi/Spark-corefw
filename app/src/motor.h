// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) Multiple stepper motors
 */
#pragma once

#include <stdbool.h>
#include <zephyr/device.h>

#define MOTOR_COUNT 7

/** (blocking) Initialize motor subsystem and step generation */
void motor_init();

/**
 * Queue a single step for specified motor for ASAP execution.
 * If different dir is enqueued faster than internal processing, they will be
 * canceled out before sent to hardware.
 */
void queue_step(int motor_num, bool dir);

/**
 * Set absolute target position for a specific motor (microsteps)
 * @param motor_num Motor number (0-6)
 * @param target_steps Target position in microsteps
 */
void motor_set_target_steps(int motor_num, int target_steps);

/**
 * Get current position for a specific motor (microsteps)
 * @param motor_num Motor number (0-6)
 * @return Current position in microsteps
 */
int motor_get_current_steps(int motor_num);

/**
 * Get motor device by number (0-6). Returns NULL for invalid motor
 * numbers.
 */
const struct device* motor_get_device(int motor_num);

/**
 * Set motor to de-energize after specified timeout when idle.
 * Negative values (e.g. -1) mean always keep energized.
 * Zero or positive values specify timeout in ms.
 */
void motor_deenergize_after(int motor_num, int timeout_ms);

/** (blocking) Dump motor subsystem status for debugging. */
void motor_dump_status();

/** (blocking) Run step test on specified motor (0-6) */
void motor_run_steptest(int motor_num);
