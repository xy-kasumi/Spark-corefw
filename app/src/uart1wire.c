#include "uart1wire.h"

#include <string.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define UART1WIRE_EVT_DONE BIT(0)
K_EVENT_DEFINE(uart1wire_evt);

static uint8_t uart1wire_buffer[UART1WIRE_BUFFER_SIZE];
static uint8_t uart1wire_phase = 0;

static const struct gpio_dt_spec* uart1wire_gpio;
static const struct device* uart1wire_timer;

// state == IDLE: don't care
// state == TX: need to send these bytes. 0 if sending done.
// state == RX: need to receive these bytes. 0 if receiving done.
// Each byte now has START + 8 data bits + STOP = 10 bits total
static int uart1wire_buffer_size;         // Total number of bytes to send/receive
static int uart1wire_current_byte_index;  // Current byte index (0 to size-1)
static int uart1wire_current_bit;  // Internal bit counter: 0=START, 1-8=DATA, 9,10=STOP
typedef enum {
  UART_IDLE,
  UART_SEND,
  UART_RECEIVE,
  UART_RECEIVE_SYNCED,
} uart_state_t;
static uart_state_t uart1wire_state = UART_IDLE;

// Handle UART bit-banging state machine (called every 30us)
static void uart1wire_tick() {
  if (uart1wire_state == UART_IDLE) {
    return;
  }

  if (uart1wire_state == UART_SEND) {
    if (uart1wire_phase == 0) {
      bool set;

      if (uart1wire_current_bit == 0) {
        // START bit: always 0
        set = false;
      } else if (uart1wire_current_bit >= 1 && uart1wire_current_bit <= 8) {
        // Data bits (1-8): LSB first
        int data_bit = uart1wire_current_bit - 1;  // 0-7
        set = (uart1wire_buffer[uart1wire_current_byte_index] >> data_bit) & 1;
      } else {
        // STOP bit (9): always 1
        set = true;
      }

      gpio_pin_set_dt(uart1wire_gpio, set);
      uart1wire_current_bit++;

      // Check if we finished current byte (START + 8 data + STOP = 10 bits)
      if (uart1wire_current_bit >= 10) {
        uart1wire_current_bit = 0;
        uart1wire_current_byte_index++;
        if (uart1wire_current_byte_index >= uart1wire_buffer_size) {
          uart1wire_state = UART_IDLE;
          k_event_post(&uart1wire_evt, UART1WIRE_EVT_DONE);
        }
      }
    }
    uart1wire_phase = (uart1wire_phase + 1) % 3;
  } else if (uart1wire_state == UART_RECEIVE) {
    // Wait for START bit (falling edge: 1 -> 0)
    if (!gpio_pin_get_dt(uart1wire_gpio)) {
      uart1wire_state = UART_RECEIVE_SYNCED;
      uart1wire_phase = 1;  // falling edge = phase 0. Next ISR will be phase 1.
                           // (good for sampling)
      uart1wire_current_bit = 0;  // We're now at START bit
    }
  } else {  // UART_RECEIVE_SYNCED
    if (uart1wire_phase == 1) {
      bool set = gpio_pin_get_dt(uart1wire_gpio);

      if (uart1wire_current_bit >= 1 && uart1wire_current_bit <= 8) {
        // Data bits (1-8): store in buffer
        int data_bit = uart1wire_current_bit - 1;  // 0-7
        if (set) {
          uart1wire_buffer[uart1wire_current_byte_index] |= BIT(data_bit);
        }
      }
      uart1wire_current_bit++;

      if (uart1wire_current_bit >= 10) {
        uart1wire_state = UART_RECEIVE;
        uart1wire_current_byte_index++;
        if (uart1wire_current_byte_index >= uart1wire_buffer_size) {
          uart1wire_state = UART_IDLE;
          k_event_post(&uart1wire_evt, UART1WIRE_EVT_DONE);
        }
      }
    }
    uart1wire_phase = (uart1wire_phase + 1) % 3;
  }
}

// UART ISR handler: manages UART bit-banging (called every 30us)
static void uart1wire_tick_handler(const struct device* dev, void* user_data) {
  uart1wire_tick();
}

int uart1wire_write(const uint8_t* data, size_t size) {
  if (size > UART1WIRE_BUFFER_SIZE) {
    return -EINVAL;
  }
  gpio_pin_configure_dt(uart1wire_gpio,
                        GPIO_OUTPUT_ACTIVE | GPIO_OPEN_DRAIN);  // idle=H.

  memcpy(uart1wire_buffer, data, size);
  uart1wire_buffer_size = size;
  uart1wire_current_byte_index = 0;
  uart1wire_current_bit = 0;
  uart1wire_phase = 0;
  uart1wire_state = UART_SEND;
  
  k_event_clear(&uart1wire_evt, UART1WIRE_EVT_DONE);
  if (!k_event_wait(&uart1wire_evt, UART1WIRE_EVT_DONE, false, K_MSEC(15))) {
    return -ETIMEDOUT;
  }
  return 0;
}

int uart1wire_read(uint8_t* buffer, size_t size) {
  if (size > UART1WIRE_BUFFER_SIZE) {
    return -EINVAL;
  }
  gpio_pin_configure_dt(uart1wire_gpio, GPIO_INPUT);

  memset(uart1wire_buffer, 0, size);
  uart1wire_buffer_size = size;
  uart1wire_current_byte_index = 0;
  uart1wire_current_bit = 0;
  uart1wire_phase = 0;
  uart1wire_state = UART_RECEIVE;
  
  k_event_clear(&uart1wire_evt, UART1WIRE_EVT_DONE);
  if (!k_event_wait(&uart1wire_evt, UART1WIRE_EVT_DONE, false, K_MSEC(15))) {
    return -ETIMEDOUT;
  }
  
  memcpy(buffer, uart1wire_buffer, size);
  return 0;
}


int uart1wire_init(const struct gpio_dt_spec* gpio, const struct device* timer) {
  uart1wire_gpio = gpio;
  uart1wire_timer = timer;
  
  // Initialize UART counter for bit-banging
  struct counter_top_cfg uart_top_cfg = {
      .callback = uart1wire_tick_handler,
      .ticks = counter_us_to_ticks(timer, 30),  // 30us ISR -> UART bit-banging
  };

  counter_start(timer);
  int ret = counter_set_top_value(timer, &uart_top_cfg);
  if (ret < 0) {
    return ret;
  }
  
  return 0;
}