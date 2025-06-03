#include "tmc2209.h"
#include "console_utils.h"

#include <string.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

static const struct gpio_dt_spec step0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(step0), gpios);

static const struct gpio_dt_spec en0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(en0), gpios);

static const struct gpio_dt_spec dir0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(dir0), gpios);

static const struct gpio_dt_spec muart0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(muart0), gpios);

static const struct device* sw_uart_cnt =
    DEVICE_DT_GET(DT_NODELABEL(sw_uart_cnt));

#define TMC_UART_BUFFER_SIZE 8

#define TMC_UART_EVT_DONE BIT(0)
K_EVENT_DEFINE(tmc_uart_evt);
uint8_t tmc_uart_buffer[TMC_UART_BUFFER_SIZE];
uint8_t tmc_uart_phase = 0;

// state == IDLE: don't care
// state == TX: need to send these bytes. 0 if sending done.
// state == RX: need to receive these bytes. 0 if receiving done.
// Each byte now has START + 8 data bits + STOP = 10 bits total
int tmc_uart_buffer_size;         // Total number of bytes to send/receive
int tmc_uart_current_byte_index;  // Current byte index (0 to size-1)
int tmc_uart_current_bit;  // Internal bit counter: 0=START, 1-8=DATA, 9,10=STOP
typedef enum {
  UART_IDLE,
  UART_SEND,
  UART_RECEIVE,
  UART_RECEIVE_SYNCED,
} uart_state_t;
uart_state_t tmc_uart_state = UART_IDLE;

// Called twice at every baud to do software bit-banging UART with START/STOP
// bits. Twice, beause we want to phase-lock. send: phase % 2 == 0. read: phase
// % 2 == 1. (phase == 0 timing acquired by START bit detection)
// tmc_uart_current_bit: 0=START, 1-8=DATA, 9=STOP
void tmc_uart_tick_handler(const struct device* dev, void* user_data) {
  if (tmc_uart_state == UART_IDLE) {
    return;
  }

  if (tmc_uart_state == UART_SEND) {
    if (tmc_uart_phase == 0) {
      bool set;

      if (tmc_uart_current_bit == 0) {
        // START bit: always 0
        set = false;
      } else if (tmc_uart_current_bit >= 1 && tmc_uart_current_bit <= 8) {
        // Data bits (1-8): LSB first
        int data_bit = tmc_uart_current_bit - 1;  // 0-7
        set = (tmc_uart_buffer[tmc_uart_current_byte_index] >> data_bit) & 1;
      } else {
        // STOP bit (9): always 1
        set = true;
      }

      gpio_pin_set_dt(&muart0, set);
      tmc_uart_current_bit++;

      // Check if we finished current byte (START + 8 data + STOP = 10 bits)
      if (tmc_uart_current_bit >= 10) {
        tmc_uart_current_bit = 0;
        tmc_uart_current_byte_index++;
        if (tmc_uart_current_byte_index >= tmc_uart_buffer_size) {
          tmc_uart_state = UART_IDLE;
          k_event_post(&tmc_uart_evt, TMC_UART_EVT_DONE);
        }
      }
    }
    tmc_uart_phase = (tmc_uart_phase + 1) % 3;
  } else if (tmc_uart_state == UART_RECEIVE) {
    // Wait for START bit (falling edge: 1 -> 0)
    if (!gpio_pin_get_dt(&muart0)) {
      tmc_uart_state = UART_RECEIVE_SYNCED;
      tmc_uart_phase = 1;  // falling edge = phase 0. Next ISR will be phase 1.
                           // (good for sampling)
      tmc_uart_current_bit = 0;  // We're now at START bit
    }
  } else {  // UART_RECEIVE_SYNCED
    if (tmc_uart_phase == 1) {
      bool set = gpio_pin_get_dt(&muart0);

      if (tmc_uart_current_bit >= 1 && tmc_uart_current_bit <= 8) {
        // Data bits (1-8): store in buffer
        int data_bit = tmc_uart_current_bit - 1;  // 0-7
        if (set) {
          tmc_uart_buffer[tmc_uart_current_byte_index] |= BIT(data_bit);
        }
      }
      tmc_uart_current_bit++;

      if (tmc_uart_current_bit >= 10) {
        tmc_uart_state = UART_RECEIVE;
        tmc_uart_current_byte_index++;
        if (tmc_uart_current_byte_index >= tmc_uart_buffer_size) {
          tmc_uart_state = UART_IDLE;
          k_event_post(&tmc_uart_evt, TMC_UART_EVT_DONE);
        }
      }
    }
    tmc_uart_phase = (tmc_uart_phase + 1) % 3;
  }
}

void tmc_uart_write(uint8_t* data, size_t size) {
  if (size > TMC_UART_BUFFER_SIZE) {
    console_printf("ERROR: Data size exceeds buffer size\n");
    return;
  }
  gpio_pin_configure_dt(&muart0,
                        GPIO_OUTPUT_ACTIVE | GPIO_OPEN_DRAIN);  // idle=H.

  memcpy(tmc_uart_buffer, data, size);
  tmc_uart_buffer_size = size;
  tmc_uart_current_byte_index = 0;
  tmc_uart_current_bit = 0;
  tmc_uart_phase = 0;
  tmc_uart_state = UART_SEND;
}

void tmc_uart_read(size_t size) {
  if (size > TMC_UART_BUFFER_SIZE) {
    console_printf("ERROR: Data size exceeds buffer size\n");
    return;
  }
  gpio_pin_configure_dt(&muart0, GPIO_INPUT);

  memset(tmc_uart_buffer, 0, size);
  tmc_uart_buffer_size = size;
  tmc_uart_current_byte_index = 0;
  tmc_uart_current_bit = 0;
  tmc_uart_phase = 0;
  tmc_uart_state = UART_RECEIVE;
}

// 8-byte structure for write request datagram.
typedef struct __attribute__((packed)) {
  uint8_t sync : 4;      // must be 0b0101 = 0x5
  uint8_t reserved : 4;  // 0 recommended
  uint8_t node_addr;
  uint8_t reg_addr : 7;
  bool write : 1;  // must be 1
  uint32_t value;  // big-endian
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
  uint32_t value;           // big-endian
  uint8_t crc;
} tmc_uart_reply_datagram_t;

uint8_t tmc_uart_crc(uint8_t* data, size_t size) {
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

// Blocking read of a register.
uint32_t tmc_tx_regread(uint8_t addr) {
  tmc_uart_request_read_datagram_t request = {
      .sync = 0x5,
      .node_addr = 0,
      .write = false,
      .reg_addr = addr,
  };
  request.crc = tmc_uart_crc((uint8_t*)&request, sizeof(request) - 1);
  k_event_clear(&tmc_uart_evt, TMC_UART_EVT_DONE);
  tmc_uart_write((uint8_t*)&request, sizeof(request));
  if (!k_event_wait(&tmc_uart_evt, TMC_UART_EVT_DONE, false, K_MSEC(15))) {
    console_printf("ERROR: Write got stuck (firmware bug)\n");
    return 0;
  }

  int res = gpio_pin_configure_dt(&muart0, GPIO_INPUT);
  if (res < 0) {
    console_printf("ERROR: Could not configure muart0 GPIO as input (%d)\n", res);
    return 0xffffffff;  // default value if error.
  }

  tmc_uart_reply_datagram_t reply;
  k_event_clear(&tmc_uart_evt, TMC_UART_EVT_DONE);
  tmc_uart_read(sizeof(reply));
  if (!k_event_wait(&tmc_uart_evt, TMC_UART_EVT_DONE, false, K_MSEC(15))) {
    console_printf("ERROR: Read got stuck (could be hardware issue)\n");
    return 0;
  }
  memcpy(&reply, tmc_uart_buffer, sizeof(reply));
  /*
  for (int i = 0; i < sizeof(reply); i++) {
    LOG_INF("reply[%d] = 0x%02x", i, tmc_uart_buffer[i]);
  }
    */

  uint8_t expected_crc = tmc_uart_crc((uint8_t*)&reply, sizeof(reply) - 1);
  if (reply.crc != expected_crc) {
    console_printf("ERROR: CRC error: expected 0x%02x, got 0x%02x\n", expected_crc, reply.crc);
  }
  if (reply.reg_addr != addr || reply.master_addr != 0xff) {
    console_printf("ERROR: Unexpected reply: got reg_addr=0x%02x, master_addr=0x%02x\n",
            reply.reg_addr, reply.master_addr);
  }
  return reply.value;
}

// Blocking write of a register.
void tmc_tx_regwrite(uint8_t addr, uint32_t value) {
  tmc_uart_request_write_datagram_t request = {
      .sync = 0x5,
      .node_addr = 0,
      .write = true,
      .reg_addr = addr,
      .value = value,
  };
  request.crc = tmc_uart_crc((uint8_t*)&request, sizeof(request) - 1);
  tmc_uart_write((uint8_t*)&request, sizeof(request));
  k_event_wait(&tmc_uart_evt, TMC_UART_EVT_DONE, false, K_FOREVER);
}

void tmc_init() {
  struct counter_top_cfg top_cfg = {
      .callback = tmc_uart_tick_handler,
      .user_data = NULL,
      .ticks = counter_us_to_ticks(sw_uart_cnt,
                                   30),  // 30us ISR -> 90us period -> 11.1kbaud
  };

  counter_start(sw_uart_cnt);
  int ret = counter_set_top_value(sw_uart_cnt, &top_cfg);
  if (ret < 0) {
    console_printf("ERROR: Could not set counter top value (%d)\n", ret);
  }
}