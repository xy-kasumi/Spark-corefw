//! Chip-specific hardware driver layer.
//! (UARTs, step pulse generation, TMC2209 driver wire protocol).
//!
//! Drivers do not own any hardware peripherals themselves. The caller (board
//! setup) owns the peripherals and injects them at construction; drivers only
//! borrow / hold the handles they were given.

pub mod serial;
pub mod soft_uart;
pub mod step_gen;
pub mod tmc2209;
