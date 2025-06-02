#include "tmc2209.h"

#include <string.h>
#include <zephyr/console/console.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static const struct gpio_dt_spec step0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(step0), gpios);

static const struct gpio_dt_spec en0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(en0), gpios);

static const struct gpio_dt_spec dir0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(dir0), gpios);

static const struct gpio_dt_spec muart0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(muart0), gpios);

// Handle a complete line received from serial console
void handle_console_line(const char* line) {
  // Trim whitespace
  while (*line == ' ' || *line == '\t')
    line++;

  if (strlen(line) == 0) {
    return;  // Empty line
  }

  printk("Received command: %s\n", line);

  // Add your custom command handlers here
  if (strcmp(line, "help") == 0) {
    printk("Available commands:\n");
    printk("  help - Show this help\n");
    printk("  regs - Read TMC registers\n");
    printk("  step <count> - Step motor <count> times\n");
  } else if (strcmp(line, "regs") == 0) {
    dump_tmc_regs();
  } else if (strncmp(line, "step ", 5) == 0) {
    int steps = atoi(line + 5);
    printk("Stepping %d times\n", steps);
  } else {
    printk("Unknown command: %s\n", line);
    printk("Type 'help' for available commands\n");
  }
}

void console_thread() {
  // console_init();
  console_getline_init();

  printk("\n=== Spark Console ===\n");
  printk("Type 'help' for available commands\n");

  while (1) {
    handle_console_line(console_getline());
  }
}

K_THREAD_DEFINE(console_tid, 1024, console_thread, NULL, NULL, NULL, 7, 0, 0);

void dump_tmc_regs() {
  uint32_t res;

  res = tmc_tx_regread(0x00);
  LOG_INF("GCONF: 0x%08x", res);
  k_sleep(K_MSEC(10));

  res = tmc_tx_regread(0x06);
  LOG_INF("IOIN: 0x%08x", res);
  k_sleep(K_MSEC(10));

  res = tmc_tx_regread(0x41);
  LOG_INF("SG_RESULT: 0x%08x", res);
  k_sleep(K_MSEC(10));

  res = tmc_tx_regread(0x6c);
  LOG_INF("CHOPCONF: 0x%08x", res);
  k_sleep(K_MSEC(10));
}

int main() {
  if (!gpio_is_ready_dt(&step0)) {
    LOG_ERR("LED GPIO not ready");
    return 0;
  }

  int ret = gpio_pin_configure_dt(&step0, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    LOG_ERR("Could not configure step0 GPIO (%d)", ret);
    return 0;
  }

  ret = gpio_pin_configure_dt(&dir0, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    LOG_ERR("Could not configure dir0 GPIO (%d)", ret);
    return 0;
  }

  ret = gpio_pin_configure_dt(&en0,
                              GPIO_OUTPUT_ACTIVE);  // ACTIVE == 1 == disabled
  if (ret < 0) {
    LOG_ERR("Could not configure en0 GPIO (%d)", ret);
    return 0;
  }

  // active is idle
  // for some reason, GPIO_OPEN_DRAIN must be configured now (instead of writing
  // time). Otherwise changing to GPIO_INPUT and reading didn't work.
  ret = gpio_pin_configure_dt(&muart0, GPIO_OUTPUT_ACTIVE | GPIO_OPEN_DRAIN);
  if (ret < 0) {
    LOG_ERR("Could not configure muart0 GPIO (%d)", ret);
    return 0;
  }

  tmc_init();
  printk("Starting main loop\n");
  k_sleep(K_FOREVER);

  while (true) {
    dump_tmc_regs();
    k_sleep(K_MSEC(5000));
  }

  while (1) {
    gpio_pin_set_dt(&step0, true);
    k_sleep(K_USEC(10));
    gpio_pin_set_dt(&step0, false);

    k_sleep(K_MSEC(1));
  }

  return 0;
}
