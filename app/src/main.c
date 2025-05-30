#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

#define BLINK_PERIOD_MS 1000U

/* Get the LED device from device tree */
#define LED_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED_NODE, gpios);

int main(void) {
  int ret;
  bool led_state = false;

  printk("Zephyr Simple GPIO Blink Application\n");

  /* Check if LED GPIO is ready */
  if (!gpio_is_ready_dt(&led)) {
    LOG_ERR("LED GPIO not ready");
    return 0;
  }

  /* Configure LED GPIO as output */
  ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    LOG_ERR("Could not configure LED GPIO (%d)", ret);
    return 0;
  }

  printk("LED on PC14 configured successfully\n");
  printk("Starting blink loop - LED should toggle every %u ms\n",
         BLINK_PERIOD_MS);

  while (1) {
    /* Toggle LED state */
    led_state = !led_state;
    ret = gpio_pin_set_dt(&led, led_state);
    if (ret < 0) {
      LOG_ERR("Could not set LED (%d)", ret);
    } else {
      printk("LED %s\n", led_state ? "ON" : "OFF");
    }

    k_sleep(K_MSEC(BLINK_PERIOD_MS));
  }

  return 0;
}
