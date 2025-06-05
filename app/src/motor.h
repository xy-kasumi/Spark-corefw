/**
 * Multiple stepper motors
 */
#pragma once

#include <stdbool.h>
#include <zephyr/device.h>

/** Initialize motor subsystem and step generation */
void motor_init();

/**
 * Queue a single step for specified motor for ASAP execution.
 * If different dir is enqueued faster than internal processing, they will be
 * canceled out before sent to hardware.
 */
void queue_step(int motor_num, bool dir);

/** Get motor device by number (0, 1, or 2). Returns NULL for invalid motor
 * numbers. */
const struct device* motor_get_device(int motor_num);

/** Read & dump TMC registers for all motors for debugging. */
void motor_dump_registers();

/** Run step test on specified motor (0, 1, or 2) */
void motor_run_steptest(int motor_num);
