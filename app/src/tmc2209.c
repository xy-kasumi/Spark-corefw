#include "tmc2209.h"

#include "comm.h"

#include <string.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#define REG_GCONF 0x00
#define REG_IOIN 0x06
#define REG_IHOLD_IRUN 0x10
#define REG_TCOOLTHRS 0x14
#define REG_SGTHRS 0x40
#define REG_SG_RESULT 0x41
#define REG_COOLCONF 0x42
#define REG_CHOPCONF 0x6C

static const struct gpio_dt_spec step0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(step0), gpios);

static const struct gpio_dt_spec en0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(en0), gpios);

static const struct gpio_dt_spec dir0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(dir0), gpios);

static const struct gpio_dt_spec muart0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(muart0), gpios);

static const struct gpio_dt_spec diag0 =
    GPIO_DT_SPEC_GET(DT_NODELABEL(diag0), gpios);

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

// Step generation state
static volatile int remaining_steps = 0;  // Positive=forward, negative=backward
static bool current_direction = false;    // false=backward, true=forward
typedef enum {
  STEP_IDLE,        // No stepping in progress
  STEP_PULSE_HIGH,  // Step pin is HIGH (1 tick)
  STEP_PULSE_LOW,   // Step pin is LOW, waiting before next step (1 tick)
} step_state_t;
static step_state_t step_state = STEP_IDLE;

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

// Handle UART bit-banging state machine (called every 30us)
static void tmc_uart_tick() {
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

// Handle step pulse generation state machine (called every 30us)
static void tmc_step_tick() {
  switch (step_state) {
    case STEP_IDLE:
      if (remaining_steps != 0) {
        // Start new step: set direction and begin pulse
        bool dir = (remaining_steps > 0);
        if (dir != current_direction) {
          current_direction = dir;
          gpio_pin_set_dt(&dir0, dir);
        }

        // Start step pulse (HIGH)
        gpio_pin_set_dt(&step0, true);
        step_state = STEP_PULSE_HIGH;
      }
      break;

    case STEP_PULSE_HIGH:
      // End step pulse (LOW)
      gpio_pin_set_dt(&step0, false);
      step_state = STEP_PULSE_LOW;

      // Consume one step
      if (remaining_steps > 0) {
        remaining_steps--;
      } else {
        remaining_steps++;
      }
      break;

    case STEP_PULSE_LOW:
      // Wait one tick before allowing next step
      step_state = STEP_IDLE;
      break;
  }
}

// Main ISR handler: manages both UART and step generation (called every 30us)
static void tmc_tick_handler(const struct device* dev, void* user_data) {
  tmc_uart_tick();
  tmc_step_tick();
}

static void tmc_uart_write(uint8_t* data, size_t size) {
  if (size > TMC_UART_BUFFER_SIZE) {
    comm_print_err("Data size exceeds buffer size");
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

static void tmc_uart_read(size_t size) {
  if (size > TMC_UART_BUFFER_SIZE) {
    comm_print_err("Data size exceeds buffer size");
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

static uint8_t tmc_uart_crc(uint8_t* data, size_t size) {
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
    comm_print_err("Write got stuck (firmware bug)");
    return 0;
  }

  int res = gpio_pin_configure_dt(&muart0, GPIO_INPUT);
  if (res < 0) {
    comm_print_err("Could not configure muart0 GPIO as input (%d)", res);
    return 0xffffffff;  // default value if error.
  }

  tmc_uart_reply_datagram_t reply;
  k_event_clear(&tmc_uart_evt, TMC_UART_EVT_DONE);
  tmc_uart_read(sizeof(reply));
  if (!k_event_wait(&tmc_uart_evt, TMC_UART_EVT_DONE, false, K_MSEC(15))) {
    comm_print_err("Read got stuck (could be hardware issue)");
    return 0;
  }
  memcpy(&reply, tmc_uart_buffer, sizeof(reply));

  uint8_t expected_crc = tmc_uart_crc((uint8_t*)&reply, sizeof(reply) - 1);
  if (reply.crc != expected_crc) {
    comm_print_err("CRC error: expected 0x%02x, got 0x%02x", expected_crc,
                   reply.crc);
  }
  if (reply.reg_addr != addr || reply.master_addr != 0xff) {
    comm_print_err("Unexpected reply: got reg_addr=0x%02x, master_addr=0x%02x",
                   reply.reg_addr, reply.master_addr);
  }
  k_sleep(K_MSEC(10));  // ensure bus returns to idle
  return sys_be32_to_cpu(reply.value);
}

// Blocking write of a register.
void tmc_tx_regwrite(uint8_t addr, uint32_t value) {
  tmc_uart_request_write_datagram_t request = {
      .sync = 0x5,
      .node_addr = 0,
      .write = true,
      .reg_addr = addr,
      .value = sys_cpu_to_be32(value),
  };
  request.crc = tmc_uart_crc((uint8_t*)&request, sizeof(request) - 1);
  tmc_uart_write((uint8_t*)&request, sizeof(request));
  k_event_wait(&tmc_uart_evt, TMC_UART_EVT_DONE, false, K_FOREVER);
  k_sleep(K_MSEC(10));  // ensure bus returns to idle
}

// Set TMC microstep resolution (1, 2, 4, 8, 16, 32, 64, 128, 256)
void tmc_set_microstep(int microstep) {
  if (microstep < 1 || microstep > 256 || (microstep & (microstep - 1)) != 0) {
    comm_print_err("Invalid microstep: %d (must be 1,2,4,8,16,32,64,128,256)",
                   microstep);
    return;
  }

  // Enable MRES from register in GCONF
  uint32_t gconf = tmc_tx_regread(REG_GCONF);
  gconf |= (1u << 7);  // mstep_reg_select = 1
  tmc_tx_regwrite(REG_GCONF, gconf);

  // Calculate MRES field: 0=256µsteps, 1=128µsteps, ..., 8=1µstep
  uint8_t mres_bits = 8 - (uint8_t)__builtin_ctz(microstep);

  // Update CHOPCONF register
  uint32_t chopconf = tmc_tx_regread(REG_CHOPCONF);
  chopconf &= 0xF0FFFFFF;                   // Clear MRES[27:24]
  chopconf |= ((uint32_t)mres_bits << 24);  // Set new MRES
  tmc_tx_regwrite(REG_CHOPCONF, chopconf);

  comm_print("Microstep set to %d", microstep);
}

// Set TMC motor current (run: 0-100%, hold: 0-100%)
void tmc_set_current(int run_percent, int hold_percent) {
  if (run_percent < 0 || run_percent > 100 || hold_percent < 0 ||
      hold_percent > 100) {
    comm_print_err("Invalid current: run=%d%% hold=%d%% (both must be 0-100%%)",
                   run_percent, hold_percent);
    return;
  }

  // Convert percentage to register values (0-31)
  uint8_t irun_bits = (run_percent * 31 + 50) / 100;
  uint8_t ihold_bits = (hold_percent * 31 + 50) / 100;

  // Use default IHOLDDELAY value of 10 (datasheet recommendation)
  uint8_t ihold_delay = 10;

  uint32_t reg = ((uint32_t)ihold_delay << 16) | ((uint32_t)irun_bits << 8) |
                 ((uint32_t)ihold_bits);
  tmc_tx_regwrite(REG_IHOLD_IRUN, reg);

  comm_print("Current set: run=%d%% hold=%d%%", run_percent, hold_percent);
}

// Enable/disable TMC motor (true=energized, false=disabled)
void tmc_energize(bool enable) {
  gpio_pin_set_dt(&en0, !enable);
  comm_print("Motor %s", enable ? "energized" : "disabled");
}

void tmc_dump_regs() {
  // Note: write-only registers are not listed here.
  comm_print("GCONF: 0x%08x", tmc_tx_regread(REG_GCONF));
  comm_print("IOIN: 0x%08x", tmc_tx_regread(REG_IOIN));
  comm_print("SG_RESULT: 0x%08x", tmc_tx_regread(REG_SG_RESULT));
  comm_print("CHOPCONF: 0x%08x", tmc_tx_regread(REG_CHOPCONF));
}

// Queue a single step (true=forward, false=backward)
void tmc_step(bool dir) {
  if (dir) {
    remaining_steps++;
  } else {
    remaining_steps--;
  }
}

// Check if motor is stalled (reads diag0 pin)
bool tmc_stalled() {
  return gpio_pin_get_dt(&diag0);
}

// Set StallGuard threshold (0-255: 0=most sensitive, 255=least sensitive)
void tmc_set_stallguard_threshold(uint8_t threshold) {
  tmc_tx_regwrite(REG_SGTHRS, threshold);
  comm_print("StallGuard threshold set to %d", threshold);
}

void tmc_init() {
  // Initialize TMC GPIO pins
  if (!gpio_is_ready_dt(&step0)) {
    comm_print_err("step0 GPIO not ready");
    return;
  }

  if (!gpio_is_ready_dt(&dir0)) {
    comm_print_err("dir0 GPIO not ready");
    return;
  }

  if (!gpio_is_ready_dt(&en0)) {
    comm_print_err("en0 GPIO not ready");
    return;
  }

  if (!gpio_is_ready_dt(&muart0)) {
    comm_print_err("muart0 GPIO not ready");
    return;
  }

  if (!gpio_is_ready_dt(&diag0)) {
    comm_print_err("diag0 GPIO not ready");
    return;
  }

  int ret = gpio_pin_configure_dt(&step0, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    comm_print_err("Could not configure step0 GPIO (%d)", ret);
    return;
  }

  ret = gpio_pin_configure_dt(&dir0, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {
    comm_print_err("Could not configure dir0 GPIO (%d)", ret);
    return;
  }

  ret = gpio_pin_configure_dt(&en0, GPIO_OUTPUT_ACTIVE);  // ACTIVE = disabled
  if (ret < 0) {
    comm_print_err("Could not configure en0 GPIO (%d)", ret);
    return;
  }

  ret = gpio_pin_configure_dt(&muart0, GPIO_OUTPUT_ACTIVE | GPIO_OPEN_DRAIN);
  if (ret < 0) {
    comm_print_err("Could not configure muart0 GPIO (%d)", ret);
    return;
  }

  ret = gpio_pin_configure_dt(&diag0, GPIO_INPUT);
  if (ret < 0) {
    comm_print_err("Could not configure diag0 GPIO (%d)", ret);
    return;
  }

  // Initialize TMC counter for UART and step generation
  struct counter_top_cfg top_cfg = {
      .callback = tmc_tick_handler,
      .user_data = NULL,
      .ticks = counter_us_to_ticks(
          sw_uart_cnt,
          30),  // 30us ISR -> UART bit-banging + step pulse generation
  };

  counter_start(sw_uart_cnt);
  ret = counter_set_top_value(sw_uart_cnt, &top_cfg);
  if (ret < 0) {
    comm_print_err("Could not set counter top value (%d)", ret);
    return;
  }

  comm_print("TMC2209 initialized");
}