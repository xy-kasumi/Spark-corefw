#pragma once

#include <stdint.h>

// Initialize tmc driver handling.
void tmc_init();

// Blocking read of a register.
uint32_t tmc_tx_regread(uint8_t addr);

// Blocking write of a register.
void tmc_tx_regwrite(uint8_t addr, uint32_t value);

// Dump TMC register values for debugging.
void dump_tmc_regs();

// Set TMC microstep resolution (1, 2, 4, 8, 16, 32, 64, 128, 256).
void set_tmc_microstep(int microstep);

// Set TMC motor current (run: 0-100%, hold: 0-100%).
void set_tmc_current(int run_percent, int hold_percent);
