#include "pulser.h"
#include "comm.h"

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

// Register addresses (from plugin_edm.c)
#define PULSER_I2C_ADDR 0x3b
#define REG_TEMPERATURE 0x03

// I2C device from device tree
static const struct device* i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

// Status tracking
static bool init_success = false;
static uint32_t poll_count = 0;

// Read single register from pulser board
static bool read_register(uint8_t reg_addr, uint8_t* value) {
  if (!i2c_dev) {
    return false;
  }

  int ret = i2c_reg_read_byte(i2c_dev, PULSER_I2C_ADDR, reg_addr, value);
  poll_count++;
  return (ret == 0);
}

void pulser_init() {
  if (!i2c_dev) {
    comm_print("Pulser init: I2C device not found");
    init_success = false;
    return;
  }

  if (!device_is_ready(i2c_dev)) {
    comm_print("Pulser init: I2C device not ready");
    init_success = false;
    return;
  }

  init_success = true;
  comm_print("Pulser initialized");
}

void pulser_dump_status() {
  comm_print("=== Pulser Status ===");
  comm_print("Init: %s", init_success ? "OK" : "FAIL");
  comm_print("I2C device: %s", i2c_dev ? "found" : "not found");
  comm_print("Poll count: %u", poll_count);

  if (init_success) {
    uint8_t temperature;
    if (read_register(REG_TEMPERATURE, &temperature)) {
      comm_print("Temperature: %u", temperature);
      comm_print("I2C: OK");
    } else {
      comm_print("I2C: FAIL (register read failed)");
    }
  } else {
    comm_print("I2C: SKIP (init failed)");
  }
}
