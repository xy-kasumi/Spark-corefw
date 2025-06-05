# SPDX-FileCopyrightText: 2025 夕月霞
# SPDX-License-Identifier: AGPL-3.0-or-later

board_runner_args(openocd "--config" "interface/cmsis-dap.cfg" "--config" "target/stm32h7x.cfg")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
