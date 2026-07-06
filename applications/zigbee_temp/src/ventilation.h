/*
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
void vent_buzzer_toggle(void);

/* Fire the buzzer immediately for 1 s — hardware test */
void ventilation_buzz_test(void);

bool vent_buzzer_is_enabled(void);

/* True if the inside or outside sensor hasn't reported recently */
bool ventilation_sensor_missing(void);
