/**
 * Multiple stepper motors
 */
#pragma once

#include <stdbool.h>
#include <zephyr/device.h>

/** Represents position in driver coordinates (microsteps) */
typedef struct {
  int m0, m1, m2;  // Position in microsteps
} pos_drv_t;

/** Initialize motor subsystem and step generation */
void motor_init();

/**
 * Queue a single step for specified motor for ASAP execution.
 * If different dir is enqueued faster than internal processing, they will be
 * canceled out before sent to hardware.
 */
void queue_step(int motor_num, bool dir);

/** Set absolute target position in driver coordinates (microsteps)
 * Target can be any absolute position, but caller should ensure it's
 * "near enough" to current position for reasonable step rate.
 * If target is far from current position, motors will step as fast
 * as possible towards target.
 */
void motor_set_target_pos_drv(pos_drv_t target);

/** Get current position in driver coordinates (microsteps) */
pos_drv_t motor_get_current_pos_drv();

/** Get motor device by number (0, 1, or 2). Returns NULL for invalid motor
 * numbers. */
const struct device* motor_get_device(int motor_num);

/** Energize or de-energize a motor (0, 1, or 2) */
void motor_energize(int motor_num, bool enable);

/** Read & dump TMC registers for all motors for debugging. */
void motor_dump_registers();

/** Run step test on specified motor (0, 1, or 2) */
void motor_run_steptest(int motor_num);
