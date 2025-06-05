#include "uart1wire.h"

#include <drivers/tmc_driver.h>

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#define DT_DRV_COMPAT adi_tmc2209_uart

// Forward declarations for Zephyr device model
struct tmc2209_config;
struct tmc2209_data;

// Zephyr device model structures (new infrastructure)
struct tmc2209_config {
  struct gpio_dt_spec step_gpio;
  struct gpio_dt_spec dir_gpio;
  struct gpio_dt_spec enable_gpio;
  struct gpio_dt_spec uart_gpio;
  struct gpio_dt_spec diag_gpio;
  const struct device* uart_timer;
};

struct tmc2209_data {
  bool initialized;
  // Runtime state will go here
};

// Device driver API function prototypes (new infrastructure)
static int tmc2209_init(const struct device* dev);

// Device driver registration (new infrastructure)
#define TMC2209_DEVICE_INIT(inst)                                       \
  static struct tmc2209_data tmc2209_data_##inst = {                    \
      .initialized = false,                                             \
  };                                                                    \
  static const struct tmc2209_config tmc2209_config_##inst = {          \
      .step_gpio = GPIO_DT_SPEC_INST_GET(inst, step_gpios),             \
      .dir_gpio = GPIO_DT_SPEC_INST_GET(inst, dir_gpios),               \
      .enable_gpio = GPIO_DT_SPEC_INST_GET(inst, enable_gpios),         \
      .uart_gpio = GPIO_DT_SPEC_INST_GET(inst, uart_gpios),             \
      .diag_gpio = GPIO_DT_SPEC_INST_GET(inst, diag_gpios),             \
      .uart_timer = DEVICE_DT_GET(DT_INST_PHANDLE(inst, uart_timer)),   \
  };                                                                    \
  DEVICE_DT_INST_DEFINE(inst, tmc2209_init, NULL, &tmc2209_data_##inst, \
                        &tmc2209_config_##inst, POST_KERNEL,            \
                        CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(TMC2209_DEVICE_INIT)

#define REG_GCONF 0x00
#define REG_IOIN 0x06
#define REG_IHOLD_IRUN 0x10
#define REG_TCOOLTHRS 0x14
#define REG_SGTHRS 0x40
#define REG_SG_RESULT 0x41
#define REG_COOLCONF 0x42
#define REG_CHOPCONF 0x6C

// 8-byte structure for write request datagram.
typedef struct __attribute__((packed)) {
  uint8_t sync : 4;      // must be 0b0101 = 0x5
  uint8_t reserved : 4;  // 0 recommended
  uint8_t node_addr;
  uint8_t reg_addr : 7;
  bool write : 1;  // must be 1
  uint32_t value;  // big-endian: use sys_cpu_to_be32()
  uint8_t crc;
} tmc_uart_request_write_datagram_t;

// 4-byte structure for read request datagram.
typedef struct __attribute__((packed)) {
  uint8_t sync : 4;      // must be 0b0101 = 0x5
  uint8_t reserved : 4;  // 0 recommended
  uint8_t node_addr;
  uint8_t reg_addr : 7;
  bool write : 1;  // must be 0
  uint8_t crc;
} tmc_uart_request_read_datagram_t;

// 8-byte structure for reply datagram.
typedef struct __attribute__((packed)) {
  uint8_t sync : 4;      // must be 0b0101 = 0x5
  uint8_t reserved : 4;  // 0
  uint8_t master_addr;   // 0xff
  uint8_t reg_addr : 7;
  bool write_reserved : 1;  // 0
  uint32_t value;           // big-endian: use sys_be32_to_cpu()
  uint8_t crc;
} tmc_uart_reply_datagram_t;

static uint8_t tmc_uart_crc(const uint8_t* data, size_t size) {
  uint8_t crc = 0;
  for (int i = 0; i < size; i++) {
    uint8_t b = data[i];
    for (int j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (b & 1)) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc = crc << 1;
      }
      b >>= 1;
    }
  }
  return crc;
}

// Zephyr device model initialization (new infrastructure)
static int tmc2209_init(const struct device* dev) {
  const struct tmc2209_config* config = dev->config;
  struct tmc2209_data* data = dev->data;

  // Initialize GPIO pins
  if (!gpio_is_ready_dt(&config->step_gpio)) {
    return -ENODEV;
  }
  if (!gpio_is_ready_dt(&config->dir_gpio)) {
    return -ENODEV;
  }
  if (!gpio_is_ready_dt(&config->enable_gpio)) {
    return -ENODEV;
  }
  if (!gpio_is_ready_dt(&config->uart_gpio)) {
    return -ENODEV;
  }
  if (!gpio_is_ready_dt(&config->diag_gpio)) {
    return -ENODEV;
  }

  // Configure GPIO pins
  int ret = gpio_pin_configure_dt(&config->step_gpio, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    return ret;
  }

  ret = gpio_pin_configure_dt(&config->dir_gpio, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    return ret;
  }

  ret = gpio_pin_configure_dt(&config->enable_gpio, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    return ret;
  }

  ret = gpio_pin_configure_dt(&config->uart_gpio,
                              GPIO_OUTPUT_ACTIVE | GPIO_OPEN_DRAIN);
  if (ret < 0) {
    return ret;
  }

  ret = gpio_pin_configure_dt(&config->diag_gpio, GPIO_INPUT);
  if (ret < 0) {
    return ret;
  }

  // Initialize uart1wire with the timer from this device
  ret = uart1wire_init(config->uart_timer);
  if (ret < 0) {
    return ret;
  }

  data->initialized = true;
  return 0;
}

// Device-based API implementations (new infrastructure)

uint32_t tmc_regread(const struct device* dev, uint8_t addr) {
  const struct tmc2209_config* config = dev->config;

  tmc_uart_request_read_datagram_t request = {
      .sync = 0x5,
      .node_addr = 0,
      .write = false,
      .reg_addr = addr,
  };
  request.crc = tmc_uart_crc((uint8_t*)&request, sizeof(request) - 1);
  if (uart1wire_write(&config->uart_gpio, (uint8_t*)&request, sizeof(request)) <
      0) {
    return 0;  // comm error
  }

  tmc_uart_reply_datagram_t reply;
  if (uart1wire_read(&config->uart_gpio, (uint8_t*)&reply, sizeof(reply)) < 0) {
    return 0;  // comm error
  }
  uint8_t expected_crc = tmc_uart_crc((uint8_t*)&reply, sizeof(reply) - 1);
  if (reply.crc != expected_crc) {
    return 0;  // CRC error
  }
  if (reply.reg_addr != addr || reply.master_addr != 0xff) {
    return 0;  // Wrong reply
  }
  k_sleep(K_MSEC(10));  // ensure bus returns to idle
  return sys_be32_to_cpu(reply.value);
}

int tmc_regwrite(const struct device* dev, uint8_t addr, uint32_t value) {
  const struct tmc2209_config* config = dev->config;

  tmc_uart_request_write_datagram_t request = {
      .sync = 0x5,
      .node_addr = 0,
      .write = true,
      .reg_addr = addr,
      .value = sys_cpu_to_be32(value),
  };
  request.crc = tmc_uart_crc((uint8_t*)&request, sizeof(request) - 1);
  int ret =
      uart1wire_write(&config->uart_gpio, (uint8_t*)&request, sizeof(request));
  if (ret < 0) {
    return ret;
  }
  k_sleep(K_MSEC(10));  // ensure bus returns to idle
  return 0;
}

int tmc_set_microstep(const struct device* dev, int microstep) {
  if (microstep < 1 || microstep > 256 || (microstep & (microstep - 1)) != 0) {
    return -EINVAL;
  }

  // Enable MRES from register in GCONF
  uint32_t gconf = tmc_regread(dev, REG_GCONF);
  gconf |= (1u << 7);  // mstep_reg_select = 1
  int ret = tmc_regwrite(dev, REG_GCONF, gconf);
  if (ret < 0) {
    return ret;
  }

  // Calculate MRES field: 0=256µsteps, 1=128µsteps, ..., 8=1µstep
  uint8_t mres_bits = 8 - (uint8_t)__builtin_ctz(microstep);

  // Update CHOPCONF register
  uint32_t chopconf = tmc_regread(dev, REG_CHOPCONF);
  chopconf &= 0xF0FFFFFF;                   // Clear MRES[27:24]
  chopconf |= ((uint32_t)mres_bits << 24);  // Set new MRES
  ret = tmc_regwrite(dev, REG_CHOPCONF, chopconf);
  if (ret < 0) {
    return ret;
  }

  return 0;
}

int tmc_set_current(const struct device* dev,
                    int run_percent,
                    int hold_percent) {
  if (run_percent < 0 || run_percent > 100 || hold_percent < 0 ||
      hold_percent > 100) {
    return -EINVAL;
  }

  // Convert percentage to register values (0-31)
  uint8_t irun_bits = (run_percent * 31 + 50) / 100;
  uint8_t ihold_bits = (hold_percent * 31 + 50) / 100;

  // Use default IHOLDDELAY value of 10 (datasheet recommendation)
  uint8_t ihold_delay = 10;

  uint32_t reg = ((uint32_t)ihold_delay << 16) | ((uint32_t)irun_bits << 8) |
                 ((uint32_t)ihold_bits);
  int ret = tmc_regwrite(dev, REG_IHOLD_IRUN, reg);
  if (ret < 0) {
    return ret;
  }

  return 0;
}

void tmc_energize(const struct device* dev, bool enable) {
  const struct tmc2209_config* config = dev->config;
  gpio_pin_set_dt(&config->enable_gpio, enable);
}

void tmc_set_step(const struct device* dev, bool step) {
  const struct tmc2209_config* config = dev->config;
  gpio_pin_set_dt(&config->step_gpio, step);
}

void tmc_set_dir(const struct device* dev, bool dir) {
  const struct tmc2209_config* config = dev->config;
  gpio_pin_set_dt(&config->dir_gpio, dir);
}

bool tmc_stalled(const struct device* dev) {
  const struct tmc2209_config* config = dev->config;
  return gpio_pin_get_dt(&config->diag_gpio);
}

int tmc_set_stallguard_threshold(const struct device* dev, uint8_t threshold) {
  return tmc_regwrite(dev, REG_SGTHRS, threshold);
}

int tmc_sgresult(const struct device* dev) {
  uint32_t result = tmc_regread(dev, REG_SG_RESULT);
  return (int)(result & 0x3FF);  // SG_RESULT is 10-bit value in bits [9:0]
}

int tmc_set_tcoolthrs(const struct device* dev, int value) {
  if (value < 1 || value > ((1 << 20) - 1)) {
    return -EINVAL;
  }

  return tmc_regwrite(dev, REG_TCOOLTHRS, (uint32_t)value);
}

int tmc_dump_regs(const struct device* dev, char* buf, size_t buf_size) {
  if (!buf || buf_size == 0) {
    return -EINVAL;
  }

  // Note: write-only registers are not listed here.
  int ret =
      snprintf(buf, buf_size,
               "TMC2209 GCONF:0x%08x IOIN:0x%08x SG_RESULT:0x%08x CHOPCONF:0x%08x",
               tmc_regread(dev, REG_GCONF), tmc_regread(dev, REG_IOIN),
               tmc_regread(dev, REG_SG_RESULT), tmc_regread(dev, REG_CHOPCONF));

  if (ret >= buf_size) {
    return -ENOSPC;  // Buffer too small
  }

  return 0;
}
