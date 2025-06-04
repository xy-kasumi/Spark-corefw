#pragma once

#include <stdbool.h>
#include <stdint.h>

// Initialize tmc driver handling.
void tmc_init();

// Blocking read of a register.
uint32_t tmc_tx_regread(uint8_t addr);

// Blocking write of a register.
void tmc_tx_regwrite(uint8_t addr, uint32_t value);

// Dump TMC register values for debugging.
void tmc_dump_regs();

// Set TMC microstep resolution (1, 2, 4, 8, 16, 32, 64, 128, 256).
void tmc_set_microstep(int microstep);

// Set TMC motor current (run: 0-100%, hold: 0-100%).
void tmc_set_current(int run_percent, int hold_percent);

// Enable/disable TMC motor (true=energized, false=disabled).
void tmc_energize(bool enable);

// Queue a single step (true=forward, false=backward).
void tmc_step(bool dir);

// Check if motor is stalled (reads diag0 pin).
bool tmc_stalled();

// Set StallGuard threshold (0-255: 0=least sensitive, 255=most sensitive).
void tmc_set_stallguard_threshold(uint8_t threshold);

// Read StallGuard result (SG_RESULT register, returns 0-510).
// 0=highest load.
int tmc_sgresult();

// Set TCOOLTHRS register (1 to 2^20-1).
// Lower threshold = StallGuard enabled at higher velocities.
void tmc_set_tcoolthrs(int value);
