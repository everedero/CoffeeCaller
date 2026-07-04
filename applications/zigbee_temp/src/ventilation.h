/*
 * SPDX-FileCopyrightText: 2025 Alicipy <dev@stefankraus.org>
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Call once from main() after hardware init */
void ventilation_init(void);

/* Call from zcl_ep_handler when a temperature report arrives */
void ventilation_update_temp(int slot, int16_t temp_centideg);

/* Toggle the ventilation alarm on/off (call from button handler) */
void ventilation_toggle(void);

/* Fire the buzzer immediately for 1 s — hardware test */
void ventilation_buzz_test(void);

bool ventilation_is_enabled(void);
