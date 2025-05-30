#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

static const struct gpio_dt_spec step0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(step0), gpios);

static const struct gpio_dt_spec en0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(en0), gpios);

static const struct gpio_dt_spec dir0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(dir0), gpios);

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

  ret = gpio_pin_configure_dt(&en0, GPIO_OUTPUT_ACTIVE); // ACTIVE == 1 == disabled
  if (ret < 0) {
    LOG_ERR("Could not configure en0 GPIO (%d)", ret);
    return 0;
  }

  printk("Starting main loop\n");

  // gpio_pin_set_dt(&en0, false); // false == ENABLE Output

  while (1) {
    gpio_pin_set_dt(&step0, true);
    k_sleep(K_USEC(10));
	gpio_pin_set_dt(&step0, false);

	k_sleep(K_MSEC(1));
  }

  return 0;
}
