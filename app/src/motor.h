#pragma once

#include <stdbool.h>
#include <zephyr/device.h>

/** Initialize motor subsystem and step generation */
void motor_init();

/** Queue a single step (true=forward, false=backward) */
void queue_step(bool dir);

/** Get motor device by number (0, 1, or 2). Returns NULL for invalid motor
 * numbers. */
const struct device* motor_get_device(int motor_num);

/** Read TMC registers for all motors */
void motor_read_registers();

/** Run step test on motor 0 */
void motor_run_steptest();
