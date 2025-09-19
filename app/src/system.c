// SPDX-FileCopyrightText: 2025 夕月霞
// SPDX-License-Identifier: AGPL-3.0-or-later
#include "system.h"

// Cancel request flag
volatile bool g_cancel_requested = false;

void state_machine_init() {
  g_cancel_requested = false;
}
