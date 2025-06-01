#include <zephyr/drivers/counter.h>
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
// state == TX: need to send these bits. 0 if sending done.
// state == RX: need to receive these bits. 0 if receiving done.
// bits are packed from the beginning to the end.
int tmc_uart_buffer_remaining_bits;
typedef enum {
  UART_SEND,
  UART_RECEIVE,
  UART_RECEIVE_SYNCED,
} uart_state_t;
uart_state_t tmc_uart_state;

// Called twice at every baud to do software bit-banging UART.
// Twice, beause we want to phase-lock.
// send: phase % 2 == 0.
// read: phase % 2 == 1. (phase == 0 timing acquired by sync signal in reply)
void tmc_uart_tick_handler(const struct device* dev, void* user_data) {
  if (tmc_uart_buffer_remaining_bits == 0) {
    return;
  }
  int ix_byte = tmc_uart_buffer_remaining_bits / 8;
  int ix_bit = tmc_uart_buffer_remaining_bits % 8;

  if (tmc_uart_state == UART_SEND) {
    if (tmc_uart_phase == 0) {
      bool set = (tmc_uart_buffer[ix_byte] >> ix_bit) & 1;
      gpio_pin_set_dt(&muart0, set);
      tmc_uart_buffer_remaining_bits--;
      if (tmc_uart_buffer_remaining_bits == 0) {
        k_event_post(&tmc_uart_evt, TMC_UART_EVT_DONE);
      }
    }
  } else if (tmc_uart_state == UART_RECEIVE) {
    // Sync (pick up firts 0 in 1010...)
    if (!gpio_pin_get_dt(&muart0)) {
      tmc_uart_state = UART_RECEIVE_SYNCED;
      tmc_uart_phase = 1;                   // will skip next cycle.
      tmc_uart_buffer[0] = 1;               // first undetected 1
      tmc_uart_buffer_remaining_bits -= 2;  // "10" was received. next is "1".
    }
  } else {
    if (tmc_uart_phase == 1) {
      bool set = gpio_pin_get_dt(&muart0);
      tmc_uart_buffer[ix_byte] =
          tmc_uart_buffer[ix_byte] | (set ? BIT(ix_bit) : 0);
      tmc_uart_buffer_remaining_bits--;
      if (tmc_uart_buffer_remaining_bits == 0) {
        k_event_post(&tmc_uart_evt, TMC_UART_EVT_DONE);
      }
    }
  }
  tmc_uart_phase = (tmc_uart_phase + 1) % 2;
}

void tmc_uart_write(uint8_t* data, size_t size) {
  if (size > TMC_UART_BUFFER_SIZE) {
    LOG_ERR("Data size exceeds buffer size");
    return;
  }
  gpio_pin_configure_dt(&muart0, GPIO_OUTPUT_ACTIVE);  // idle=H.

  memcpy(tmc_uart_buffer, data, size);
  tmc_uart_buffer_remaining_bits = size * 8;
  tmc_uart_phase = 0;
  tmc_uart_state = UART_SEND;
}

void tmc_uart_read(size_t size) {
  if (size > TMC_UART_BUFFER_SIZE) {
    LOG_ERR("Data size exceeds buffer size");
    return;
  }
  gpio_pin_configure_dt(&muart0, GPIO_INPUT);

  memset(tmc_uart_buffer, 0, size);
  tmc_uart_buffer_remaining_bits = size * 8;
  tmc_uart_phase = 0;
  tmc_uart_state = UART_RECEIVE;
}

// 8-byte structure for write request datagram.
typedef struct __attribute__((packed)) {
  uint8_t sync : 4;      // must be 0b0101 = 0x5
  uint8_t reserved : 4;  // 0 recommended
  uint8_t node_addr;
  bool write : 1;  // must be 1
  uint8_t reg_addr : 7;
  uint32_t value;  // big-endian
  uint8_t crc;
} tmc_uart_request_write_datagram_t;

// 4-byte structure for read request datagram.
typedef struct __attribute__((packed)) {
  uint8_t sync : 4;      // must be 0b0101 = 0x5
  uint8_t reserved : 4;  // 0 recommended
  uint8_t node_addr;
  bool write : 1;  // must be 0
  uint8_t reg_addr : 7;
  uint8_t crc;
} tmc_uart_request_read_datagram_t;

// 8-byte structure for reply datagram.
typedef struct __attribute__((packed)) {
  uint8_t sync : 4;         // must be 0b0101 = 0x5
  uint8_t reserved : 4;     // 0
  uint8_t master_addr;      // 0xff
  bool write_reserved : 1;  // 0
  uint8_t reg_addr : 7;
  uint32_t value;  // big-endian
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
  k_event_wait(&tmc_uart_evt, TMC_UART_EVT_DONE, false, K_FOREVER);
  return 0xffffffff;  // default value if no reply received.

  tmc_uart_reply_datagram_t reply;
  k_event_clear(&tmc_uart_evt, TMC_UART_EVT_DONE);
  tmc_uart_read(sizeof(reply));
  k_event_wait(&tmc_uart_evt, TMC_UART_EVT_DONE, false, K_FOREVER);
  memcpy(&reply, tmc_uart_buffer, sizeof(reply));
  uint8_t expected_crc = tmc_uart_crc((uint8_t*)&reply, sizeof(reply) - 1);
  if (reply.crc != expected_crc) {
    LOG_ERR("CRC error: expected 0x%02x, got 0x%02x", expected_crc, reply.crc);
  }
  if (reply.reg_addr != addr || reply.master_addr != 0xff) {
    LOG_ERR("Unexpected reply: reg_addr=0x%02x, master_addr=0x%02x",
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

void tmc_setup() {
  // k_event_init(&tmc_uart_evt);
}

void check_reg() {
  uint32_t res;

  res = tmc_tx_regread(0);
  LOG_INF("GCONF: 0x%08x", res);
}

static bool test_set = false;

static void top_cb(const struct device* dev, void* user_data) {
  gpio_pin_set_dt(&muart0, test_set);
  test_set = !test_set;
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

  ret = gpio_pin_configure_dt(&muart0, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
    LOG_ERR("Could not configure muart0 GPIO (%d)", ret);
    return 0;
  }

  printk("Starting main loop\n");

  struct counter_top_cfg top_cfg = {
      .callback = tmc_uart_tick_handler,
      .user_data = NULL,
      .ticks = counter_us_to_ticks(sw_uart_cnt, 10),
  };

  counter_start(sw_uart_cnt);
  ret = counter_set_top_value(sw_uart_cnt, &top_cfg);
  if (ret < 0) {
    LOG_ERR("Could not set counter top value (%d)", ret);
    return 0;
  }

  while (true) {
    check_reg();
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
