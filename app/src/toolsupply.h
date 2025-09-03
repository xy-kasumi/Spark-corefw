// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * (Singleton) Tool supply control
 */
#pragma once

typedef enum {
    TOOL_SUPPLY_OPEN,
    TOOL_SUPPLY_CLOSED
} tool_supply_state_t;

/** Initialize tool supply PWM servo. */
void toolsupply_init();

/** Configure servo positions for tool supply states.
 * @param state Tool supply state to configure
 * @param on_ms on-cycle time in msec
 */
void configure_tool_supply_servo_on(tool_supply_state_t state, float on_ms);

/** Set tool supply to target state (blocking). */
void set_tool_supply_state(tool_supply_state_t target);
