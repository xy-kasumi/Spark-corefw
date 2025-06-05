/**
 * Single-wire UART implementation using bit-banging
 *
 * Supports multiple GPIOs. All write, read (even to different GPIO pin)
 * will be serialized, and are not thread-safe.
 *
 * Protocol details:
 * - Baud rate: ~11.1 kbps (30us timer × 3 phases = 90us per bit)
 * - Frame format: 1 start bit (0) + 8 data bits + 1 stop bit (1)
 * - Data bits: LSB first (bit 0 transmitted first)
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <zephyr/drivers/gpio.h>

#define UART1WIRE_BUFFER_SIZE 8

/**
 * Initialize uart1wire with shared timer (call once per timer)
 * Multiple calls with same timer are safe.
 * @return 0 if ok. -EINVAL if different timer is used (timer mismatch)
 */
int uart1wire_init(const struct device* timer);

/**
 * Write data over uart1wire (blocking, serialized access)
 * @param gpio GPIO pin for this device
 * @param data Buffer to transmit
 * @param size Number of bytes (max UART1WIRE_BUFFER_SIZE)
 * @return 0 on success, -EBUSY if another operation ongoing, negative error
 *         code on failure
 */
int uart1wire_write(const struct gpio_dt_spec* gpio,
                    const uint8_t* data,
                    size_t size);

/**
 * Read data over uart1wire (blocking, serialized access)
 * @param gpio GPIO pin for this device
 * @param buffer Buffer to store received data
 * @param size Number of bytes (max UART1WIRE_BUFFER_SIZE)
 * @return 0 on success, -EBUSY if another operation ongoing, negative error
 *         code on failure
 */
int uart1wire_read(const struct gpio_dt_spec* gpio,
                   uint8_t* buffer,
                   size_t size);
