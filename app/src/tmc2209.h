#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Initialize tmc driver handling.
int tmc_init();

// Blocking read of a register.
uint32_t tmc_tx_regread(uint8_t addr);

// Blocking write of a register.
int tmc_tx_regwrite(uint8_t addr, uint32_t value);

// Dump TMC register values for debugging.
int tmc_dump_regs(char* buf, size_t buf_size);

// Set TMC microstep resolution (1, 2, 4, 8, 16, 32, 64, 128, 256).
int tmc_set_microstep(int microstep);

// Set TMC motor current (run: 0-100%, hold: 0-100%).
int tmc_set_current(int run_percent, int hold_percent);

// Enable/disable TMC motor (true=energized, false=disabled).
void tmc_energize(bool enable);

// Set step pin state (true=HIGH, false=LOW).
void tmc_set_step(bool step);

// Set direction pin state (true=HIGH, false=LOW).
void tmc_set_dir(bool dir);

// Check if motor is stalled (reads diag0 pin).
bool tmc_stalled();

// Set StallGuard threshold (0-255: 0=least sensitive, 255=most sensitive).
int tmc_set_stallguard_threshold(uint8_t threshold);

// Read StallGuard result (SG_RESULT register, returns 0-510).
// 0=highest load.
int tmc_sgresult();

// Set TCOOLTHRS register (1 to 2^20-1).
// Lower threshold = StallGuard enabled at higher velocities.
int tmc_set_tcoolthrs(int value);
