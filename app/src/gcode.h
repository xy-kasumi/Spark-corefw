// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
/**
 * G-code execution (main entry point for G/M-code commands).
 */
#pragma once

/**
 * Execute a G-code or M-code command.
 * @param full_command The complete G/M-code command string
 */
void exec_gcode(char* full_command);
