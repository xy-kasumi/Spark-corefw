# SPDX-License-Identifier: CC0-1.0
board_runner_args(openocd "--config" "interface/cmsis-dap.cfg" "--config" "target/stm32h7x.cfg")

include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
