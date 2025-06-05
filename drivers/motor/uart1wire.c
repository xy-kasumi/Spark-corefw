#include "uart1wire.h"

#include <string.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

// Event handling
#define EVT_DONE BIT(0)
K_EVENT_DEFINE(evt);

// Module state
static const struct device* timer;
static const struct gpio_dt_spec* current_gpio;
static bool busy = false;

// State machine
typedef enum {
  UART_IDLE,
  UART_SEND,
  UART_RECEIVE,
  UART_RECEIVE_SYNCED,
} uart_state_t;

static uart_state_t state = UART_IDLE;
static uint8_t phase = 0;

// Buffer management
// Each byte: START + 8 data bits + STOP = 10 bits total
static uint8_t buffer[UART1WIRE_BUFFER_SIZE];
static int buffer_size;         // Total number of bytes to send/receive
static int current_byte_index;  // Current byte index (0 to size-1)
static int current_bit;  // Internal bit counter: 0=START, 1-8=DATA, 9=STOP

// Handle UART bit-banging state machine (called every 30us)
static void tick() {
  if (state == UART_IDLE) {
    return;
  }

  if (state == UART_SEND) {
    if (phase == 0) {
      bool set;

      if (current_bit == 0) {
        // START bit: always 0
        set = false;
      } else if (current_bit >= 1 && current_bit <= 8) {
        // Data bits (1-8): LSB first
        int data_bit = current_bit - 1;  // 0-7
        set = (buffer[current_byte_index] >> data_bit) & 1;
      } else {
        // STOP bit (9): always 1
        set = true;
      }

      gpio_pin_set_dt(current_gpio, set);
      current_bit++;

      // Check if we finished current byte (START + 8 data + STOP = 10 bits)
      if (current_bit >= 10) {
        current_bit = 0;
        current_byte_index++;
        if (current_byte_index >= buffer_size) {
          state = UART_IDLE;
          k_event_post(&evt, EVT_DONE);
        }
      }
    }
    phase = (phase + 1) % 3;
  } else if (state == UART_RECEIVE) {
    // Wait for START bit (falling edge: 1 -> 0)
    if (!gpio_pin_get_dt(current_gpio)) {
      state = UART_RECEIVE_SYNCED;
      phase = 1;        // falling edge = phase 0. Next ISR will be phase 1.
                        // (good for sampling)
      current_bit = 0;  // We're now at START bit
    }
  } else {  // UART_RECEIVE_SYNCED
    if (phase == 1) {
      bool set = gpio_pin_get_dt(current_gpio);

      if (current_bit >= 1 && current_bit <= 8) {
        // Data bits (1-8): store in buffer
        int data_bit = current_bit - 1;  // 0-7
        if (set) {
          buffer[current_byte_index] |= BIT(data_bit);
        }
      }
      current_bit++;

      if (current_bit >= 10) {
        state = UART_RECEIVE;
        current_byte_index++;
        if (current_byte_index >= buffer_size) {
          state = UART_IDLE;
          k_event_post(&evt, EVT_DONE);
        }
      }
    }
    phase = (phase + 1) % 3;
  }
}

// UART ISR handler: manages UART bit-banging (called every 30us)
static void tick_handler(const struct device* dev, void* user_data) {
  tick();
}

int uart1wire_write(const struct gpio_dt_spec* gpio,
                    const uint8_t* data,
                    size_t size) {
  if (size > UART1WIRE_BUFFER_SIZE) {
    return -EINVAL;
  }

  if (busy) {
    return -EBUSY;
  }

  busy = true;
  current_gpio = gpio;

  gpio_pin_configure_dt(gpio, GPIO_OUTPUT_ACTIVE | GPIO_OPEN_DRAIN);  // idle=H.

  memcpy(buffer, data, size);
  buffer_size = size;
  current_byte_index = 0;
  current_bit = 0;
  phase = 0;
  state = UART_SEND;

  k_event_clear(&evt, EVT_DONE);
  int ret = 0;
  if (!k_event_wait(&evt, EVT_DONE, false, K_MSEC(15))) {
    ret = -ETIMEDOUT;
  }

  busy = false;
  return ret;
}

int uart1wire_read(const struct gpio_dt_spec* gpio,
                   uint8_t* output,
                   size_t size) {
  if (size > UART1WIRE_BUFFER_SIZE) {
    return -EINVAL;
  }

  if (busy) {
    return -EBUSY;
  }

  busy = true;
  current_gpio = gpio;

  gpio_pin_configure_dt(gpio, GPIO_INPUT);

  memset(buffer, 0, size);
  buffer_size = size;
  current_byte_index = 0;
  current_bit = 0;
  phase = 0;
  state = UART_RECEIVE;

  k_event_clear(&evt, EVT_DONE);
  int ret = 0;
  if (!k_event_wait(&evt, EVT_DONE, false, K_MSEC(15))) {
    ret = -ETIMEDOUT;
  } else {
    memcpy(output, buffer, size);
  }

  busy = false;
  return ret;
}

int uart1wire_init(const struct device* timer_dev) {
  // Verify timer compatibility for multi-device support
  if (timer != NULL) {
    if (timer != timer_dev) {
      return -EINVAL;  // Timer mismatch - different devices using different
                       // timers
    }
    return 0;  // Already initialized with same timer - no action needed
  }

  timer = timer_dev;

  // Initialize UART counter for bit-banging
  struct counter_top_cfg uart_top_cfg = {
      .callback = tick_handler,
      .ticks = counter_us_to_ticks(timer, 30),  // 30us ISR -> UART bit-banging
  };

  counter_start(timer);
  int ret = counter_set_top_value(timer, &uart_top_cfg);
  if (ret < 0) {
    return ret;
  }

  return 0;
}
