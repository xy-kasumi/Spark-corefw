/**
 * @file tmc_driver.h
 * @brief TMC stepper motor driver API
 *
 * Provides interface for TMC stepper motor drivers with UART communication
 * and StallGuard load detection capabilities.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>

/**
 * @brief Initialize TMC driver
 * @return 0 on success, negative error code on failure
 */
int tmc_init();

/**
 * @brief Read TMC register
 * @param addr Register address
 * @return Register value, 0 on communication error
 */
uint32_t tmc_tx_regread(uint8_t addr);

/**
 * @brief Write TMC register
 * @param addr Register address
 * @param value Register value
 * @return 0 on success, negative error code on failure
 */
int tmc_tx_regwrite(uint8_t addr, uint32_t value);

/**
 * @brief Dump TMC registers to buffer for debugging
 * @param buf Output buffer
 * @param buf_size Buffer size
 * @return 0 on success, -EINVAL for invalid parameters, -ENOSPC if buffer too
 * small
 */
int tmc_dump_regs(char* buf, size_t buf_size);

/**
 * @brief Set TMC microstep resolution
 * @param microstep Resolution (1, 2, 4, 8, 16, 32, 64, 128, 256)
 * @return 0 on success, -EINVAL for invalid parameter, negative error code on
 * failure
 */
int tmc_set_microstep(int microstep);

/**
 * @brief Set TMC motor current
 * @param run_percent Run current (0-100%)
 * @param hold_percent Hold current (0-100%)
 * @return 0 on success, -EINVAL for invalid parameters, negative error code on
 * failure
 */
int tmc_set_current(int run_percent, int hold_percent);

/**
 * @brief Enable/disable TMC motor
 * @param enable true=energized, false=disabled
 */
void tmc_energize(bool enable);

/**
 * @brief Set step pin state
 * @param step true=HIGH, false=LOW
 */
void tmc_set_step(bool step);

/**
 * @brief Set direction pin state
 * @param dir true=HIGH, false=LOW
 */
void tmc_set_dir(bool dir);

/**
 * @brief Check if motor is stalled
 * @return true if stall detected via diag pin
 */
bool tmc_stalled();

/**
 * @brief Set StallGuard threshold
 * @param threshold Sensitivity (0-255: higher=more sensitive, lower=less
 * sensitive)
 * @return 0 on success, negative error code on failure
 */
int tmc_set_stallguard_threshold(uint8_t threshold);

/**
 * @brief Read StallGuard result
 * @return Load measurement (0-510: 0=highest load), negative on error
 */
int tmc_sgresult();

/**
 * @brief Set TCOOLTHRS register
 * @param value Threshold value (1 to 2^20-1)
 * @return 0 on success, -EINVAL for invalid parameter, negative error code on
 * failure
 */
int tmc_set_tcoolthrs(int value);

/**
 * @brief Device-based TMC driver API
 */

/**
 * @brief Read TMC register from device
 * @param dev TMC device instance
 * @param addr Register address
 * @return Register value, 0 on communication error
 */
uint32_t tmc_dev_regread(const struct device* dev, uint8_t addr);

/**
 * @brief Write TMC register to device
 * @param dev TMC device instance
 * @param addr Register address
 * @param value Register value
 * @return 0 on success, negative error code on failure
 */
int tmc_dev_regwrite(const struct device* dev, uint8_t addr, uint32_t value);

/**
 * @brief Set TMC microstep resolution for device
 * @param dev TMC device instance
 * @param microstep Resolution (1, 2, 4, 8, 16, 32, 64, 128, 256)
 * @return 0 on success, -EINVAL for invalid parameter, negative error code on
 * failure
 */
int tmc_dev_set_microstep(const struct device* dev, int microstep);

/**
 * @brief Set TMC motor current for device
 * @param dev TMC device instance
 * @param run_percent Run current (0-100%)
 * @param hold_percent Hold current (0-100%)
 * @return 0 on success, -EINVAL for invalid parameters, negative error code on
 * failure
 */
int tmc_dev_set_current(const struct device* dev,
                        int run_percent,
                        int hold_percent);

/**
 * @brief Enable/disable TMC motor
 * @param dev TMC device instance
 * @param enable true=energized, false=disabled
 */
void tmc_dev_energize(const struct device* dev, bool enable);

/**
 * @brief Set step pin state
 * @param dev TMC device instance
 * @param step true=HIGH, false=LOW
 */
void tmc_dev_set_step(const struct device* dev, bool step);

/**
 * @brief Set direction pin state
 * @param dev TMC device instance
 * @param dir true=HIGH, false=LOW
 */
void tmc_dev_set_dir(const struct device* dev, bool dir);

/**
 * @brief Check if motor is stalled
 * @param dev TMC device instance
 * @return true if stall detected via diag pin
 */
bool tmc_dev_stalled(const struct device* dev);

/**
 * @brief Set StallGuard threshold
 * @param dev TMC device instance
 * @param threshold Sensitivity (0-255: higher=more sensitive, lower=less
 * sensitive)
 * @return 0 on success, negative error code on failure
 */
int tmc_dev_set_stallguard_threshold(const struct device* dev,
                                     uint8_t threshold);

/**
 * @brief Read StallGuard result
 * @param dev TMC device instance
 * @return Load measurement (0-510: 0=highest load), negative on error
 */
int tmc_dev_sgresult(const struct device* dev);

/**
 * @brief Set TCOOLTHRS register
 * @param dev TMC device instance
 * @param value Threshold value (1 to 2^20-1)
 * @return 0 on success, -EINVAL for invalid parameter, negative error code on
 * failure
 */
int tmc_dev_set_tcoolthrs(const struct device* dev, int value);

/**
 * @brief Dump TMC registers to buffer for debugging
 * @param dev TMC device instance
 * @param buf Output buffer
 * @param buf_size Buffer size
 * @return 0 on success, -EINVAL for invalid parameters, -ENOSPC if buffer too
 * small
 */
int tmc_dev_dump_regs(const struct device* dev, char* buf, size_t buf_size);
