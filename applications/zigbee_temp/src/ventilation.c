/*
 * SPDX-FileCopyrightText: 2025 Alicipy <dev@stefankraus.org>
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ventilation alarm: buzzes when inside temperature is warmer than outside by
 * more than 2°C (20-minute rolling average), inside > 25°C, at most once/hour.
 * Slot 0 = outside sensor, slot 1 = inside sensor.
 */

#include "ventilation.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/led_strip.h>

LOG_MODULE_REGISTER(ventilation, LOG_LEVEL_INF);

/* --- Hardware --------------------------------------------------------------- */

static const struct device *pwm0_dev;
static const struct device *strip_dev;

/* PWM0 channel 0 drives P1.10 (buzzer).  880 Hz ≈ a sharp attention tone. */
#define BUZZER_PERIOD_NS  (NSEC_PER_SEC / 880U)
#define STRIP_NUM_LEDS    4

static struct led_rgb pixels[STRIP_NUM_LEDS];

/* --- State ------------------------------------------------------------------ */

static K_MUTEX_DEFINE(v_lock);
static int16_t v_temp[2];       /* centidegrees; slot 0=inside, 1=outside */
static bool    v_temp_valid[2]; /* true once the slot has received at least one report */

#define SAMPLE_PERIOD_S 60
#define HISTORY_SIZE    20 /* 20 samples × 60 s = 20-minute rolling window */

static int16_t diff_hist[HISTORY_SIZE]; /* inside-outside per sample */
static int     hist_idx;
static int     hist_count;

static bool    vent_enabled;
/* Negative initial value ensures cooldown is not active at first trigger */
static int64_t last_buzz_uptime_ms = -3600000LL;

static struct k_timer          sample_timer;
static struct k_work           sample_work;
static struct k_work_delayable buzz_stop_work;

/* --- Buzzer ----------------------------------------------------------------- */

static void buzzer_on(void)
{
	pwm_set(pwm0_dev, 0, BUZZER_PERIOD_NS, BUZZER_PERIOD_NS / 2,
		PWM_POLARITY_NORMAL);
}

static void buzzer_stop_fn(struct k_work *w)
{
	ARG_UNUSED(w);
	pwm_set(pwm0_dev, 0, BUZZER_PERIOD_NS, 0, PWM_POLARITY_NORMAL);
	LOG_INF("Buzzer off");
}

/* --- LED -------------------------------------------------------------------- */

static void led_set_vent(bool enabled)
{
	pixels[0] = enabled ? ((struct led_rgb){.r = 128, .g = 0, .b = 0})
			    : ((struct led_rgb){.r = 0, .g = 0, .b = 0});
	led_strip_update_rgb(strip_dev, pixels, STRIP_NUM_LEDS);
}

/* --- Sample work handler ---------------------------------------------------- */

static void sample_work_fn(struct k_work *w)
{
	ARG_UNUSED(w);

	int16_t inside, outside;
	bool    both_valid;

	k_mutex_lock(&v_lock, K_FOREVER);
	inside     = v_temp[0];
	outside    = v_temp[1];
	both_valid = v_temp_valid[0] && v_temp_valid[1];
	k_mutex_unlock(&v_lock);

	if (!both_valid) {
		LOG_INF("vent: waiting for both sensors");
		return;
	}

	int16_t diff = inside - outside;

	diff_hist[hist_idx] = diff;
	hist_idx = (hist_idx + 1) % HISTORY_SIZE;
	if (hist_count < HISTORY_SIZE) {
		hist_count++;
	}

	int32_t sum = 0;

	for (int i = 0; i < hist_count; i++) {
		sum += diff_hist[i];
	}
	int16_t avg_diff = (int16_t)(sum / hist_count);

	LOG_INF("vent_sample: inside=%d outside=%d diff=%d avg=%d (n=%d/%d)",
		inside, outside, diff, avg_diff, hist_count, HISTORY_SIZE);

	if (!vent_enabled) {
		return;
	}
	if (hist_count < HISTORY_SIZE) {
		LOG_INF("vent: warming up, waiting for full window");
		return;
	}
	if (inside <= 2500) {
		return; /* inside not warm enough */
	}
	if (avg_diff <= 200) {
		return; /* not enough gradient */
	}
	if (k_uptime_get() - last_buzz_uptime_ms < 3600000LL) {
		LOG_INF("vent: cooldown active");
		return;
	}

	LOG_INF("vent: BUZZ triggered (inside=%d avg_diff=%d)", inside, avg_diff);
	last_buzz_uptime_ms = k_uptime_get();
	buzzer_on();
	k_work_schedule(&buzz_stop_work, K_SECONDS(3));
}

static void sample_timer_fn(struct k_timer *t)
{
	ARG_UNUSED(t);
	k_work_submit(&sample_work);
}

/* --- Public API ------------------------------------------------------------- */

void ventilation_init(void)
{
	pwm0_dev  = DEVICE_DT_GET(DT_NODELABEL(pwm0));
	strip_dev = DEVICE_DT_GET(DT_ALIAS(led_strip));

	if (!device_is_ready(pwm0_dev)) {
		LOG_ERR("PWM0 not ready");
	}
	if (!device_is_ready(strip_dev)) {
		LOG_ERR("LED strip not ready");
	}

	k_work_init(&sample_work, sample_work_fn);
	k_work_init_delayable(&buzz_stop_work, buzzer_stop_fn);
	k_timer_init(&sample_timer, sample_timer_fn, NULL);
	k_timer_start(&sample_timer,
		      K_SECONDS(SAMPLE_PERIOD_S), K_SECONDS(SAMPLE_PERIOD_S));

	led_set_vent(false);

	LOG_INF("Ventilation alarm initialized (disabled; %d-min warmup)", HISTORY_SIZE);
}

void ventilation_update_temp(int slot, int16_t temp_centideg)
{
	if (slot < 0 || slot > 1) {
		return;
	}
	k_mutex_lock(&v_lock, K_FOREVER);
	v_temp[slot]       = temp_centideg;
	v_temp_valid[slot] = true;
	k_mutex_unlock(&v_lock);
}

void ventilation_toggle(void)
{
	vent_enabled = !vent_enabled;
	led_set_vent(vent_enabled);
	LOG_INF("Ventilation alarm %s", vent_enabled ? "ENABLED" : "DISABLED");
}

bool ventilation_is_enabled(void)
{
	return vent_enabled;
}

void ventilation_buzz_test(void)
{
	LOG_INF("Buzzer test");
	buzzer_on();
	k_work_schedule(&buzz_stop_work, K_SECONDS(1));
}
