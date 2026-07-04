# Copyright (c) 2025 TiaC Systems
# SPDX-License-Identifier: Apache-2.0

board_runner_args(openocd "--config=${BOARD_DIR}/../../coffeecaller_ftdi2232.cfg")
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
