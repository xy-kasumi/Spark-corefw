/**
 * Single-wire UART implementation using bit-banging
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

// Initialize uart1wire with shared timer (call once per timer)
// Multiple calls with same timer are safe (returns 0)
// Returns -EINVAL if different timer is used (timer mismatch)
int uart1wire_init(const struct device* timer);

// Write data over uart1wire (blocking, serialized access)
// gpio: GPIO pin for this device, data: Buffer to transmit, size: Number of
// bytes (max UART1WIRE_BUFFER_SIZE) Returns: 0 on success, -EBUSY if another
// operation ongoing, negative error code on failure
int uart1wire_write(const struct gpio_dt_spec* gpio,
                    const uint8_t* data,
                    size_t size);

// Read data over uart1wire (blocking, serialized access)
// gpio: GPIO pin for this device, buffer: Buffer to store received data, size:
// Number of bytes (max UART1WIRE_BUFFER_SIZE) Returns: 0 on success, -EBUSY if
// another operation ongoing, negative error code on failure
int uart1wire_read(const struct gpio_dt_spec* gpio,
                   uint8_t* buffer,
                   size_t size);
