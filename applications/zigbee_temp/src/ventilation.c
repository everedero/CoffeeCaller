/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Ventilation alarm: beeps in short pulses for 10 s when inside temperature is
 * warmer than outside by more than 2°C (20-minute rolling average), inside >
 * 25°C, at most once/hour. Also drives the outside-vs-inside color indicator
 * (RGB pixel 1) and reports sensor staleness for the LED1 health blink.
 * Slot 1 = outside sensor, slot 0 = inside sensor.
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
/*
 * Guards pixels[] and led_strip_update_rgb(): led_set_vent() runs on the
 * system workqueue (button press) while led_set_outside_indicator() runs on
 * the Zigbee thread (ZCL report) — without this, two concurrent bit-banged
 * GPIO transmissions on the same wire corrupt each other's pixel data.
 */
static K_MUTEX_DEFINE(led_lock);

/* --- State ------------------------------------------------------------------ */

static K_MUTEX_DEFINE(v_lock);
static int16_t v_temp[2];             /* centidegrees; slot 0=inside, 1=outside */
static bool    v_temp_valid[2];       /* true once the slot has received at least one report */
static int64_t v_temp_last_seen_ms[2]; /* uptime of the slot's last report */

#define SAMPLE_PERIOD_S 60
#define HISTORY_SIZE    20 /* 20 samples × 60 s = 20-minute rolling window */

/* Sensor considered missing/stale if silent longer than this (> 2x the 300s
 * max ZCL report interval configured in main.c). Drives the LED1 health blink. */
#define SENSOR_STALE_TIMEOUT_MS (10 * 60 * 1000)

static int16_t diff_hist[HISTORY_SIZE]; /* inside-outside per sample */
static int     hist_idx;
static int     hist_count;

static bool    vent_enabled;
/* Negative initial value ensures cooldown is not active at first trigger */
static int64_t last_buzz_uptime_ms = -3600000LL;

static struct k_timer          sample_timer;
static struct k_work           sample_work;
static struct k_work_delayable buzz_pattern_work;

/* --- Buzzer ----------------------------------------------------------------- */

#define VENT_BUZZ_DURATION_MS 10000
#define VENT_BEEP_ON_MS       200
#define VENT_BEEP_OFF_MS      200

static int64_t buzz_pattern_deadline_ms;
static bool    buzz_pattern_on;

static void buzzer_set(bool on)
{
	pwm_set(pwm0_dev, 0, BUZZER_PERIOD_NS, on ? BUZZER_PERIOD_NS / 2 : 0,
		PWM_POLARITY_NORMAL);
}

static void buzz_pattern_work_fn(struct k_work *w)
{
	ARG_UNUSED(w);

	if (k_uptime_get() >= buzz_pattern_deadline_ms) {
		buzzer_set(false);
		LOG_INF("Buzzer pattern done");
		return;
	}

	buzz_pattern_on = !buzz_pattern_on;
	buzzer_set(buzz_pattern_on);
	k_work_schedule(&buzz_pattern_work,
			K_MSEC(buzz_pattern_on ? VENT_BEEP_ON_MS : VENT_BEEP_OFF_MS));
}

static void buzzer_start_pattern(uint32_t duration_ms)
{
	buzz_pattern_deadline_ms = k_uptime_get() + duration_ms;
	buzz_pattern_on = true;
	buzzer_set(true);
	k_work_schedule(&buzz_pattern_work, K_MSEC(VENT_BEEP_ON_MS));
}

/* --- LED -------------------------------------------------------------------- */

/*
 * Authoritative colors for pixels 0 and 1. These, not pixels[] itself, are
 * the source of truth: the ws2812-gpio driver's update_rgb() converts to
 * on-wire format IN PLACE inside the buffer it's given (it repacks r/g/b
 * bytes across pixel boundaries using the CONFIG_LED_STRIP_RGB_SCRATCH pad
 * byte), so pixels[] no longer holds valid RGB values for any pixel that
 * wasn't just freshly (re)assigned once update_rgb() returns. Every flush
 * therefore rebuilds pixels[] from scratch from these two variables so one
 * indicator's update never leaks stale/corrupted bytes into the other's
 * pixel slot.
 */
static struct led_rgb vent_color;
static struct led_rgb outside_color;

static void leds_flush(void)
{
	k_mutex_lock(&led_lock, K_FOREVER);
	pixels[0] = vent_color;
	pixels[1] = outside_color;
	led_strip_update_rgb(strip_dev, pixels, STRIP_NUM_LEDS);
	k_mutex_unlock(&led_lock);
}

static void led_set_vent(bool enabled)
{
	vent_color = enabled ? (struct led_rgb){.r = 128, .g = 0, .b = 0}
			     : (struct led_rgb){.r = 0, .g = 0, .b = 0};
	leds_flush();
}

static void led_set_outside_indicator(int16_t inside, int16_t outside)
{
	outside_color = (outside <= inside)
				? (struct led_rgb){.r = 0, .g = 0, .b = 128}   /* blue: fresher outside */
				: (struct led_rgb){.r = 160, .g = 60, .b = 0}; /* orange: hotter outside */
	leds_flush();
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
	buzzer_start_pattern(VENT_BUZZ_DURATION_MS);
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
	k_work_init_delayable(&buzz_pattern_work, buzz_pattern_work_fn);
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
	v_temp[slot]              = temp_centideg;
	v_temp_valid[slot]        = true;
	v_temp_last_seen_ms[slot] = k_uptime_get();

	bool    both_valid = v_temp_valid[0] && v_temp_valid[1];
	int16_t inside     = v_temp[0];
	int16_t outside    = v_temp[1];
	k_mutex_unlock(&v_lock);

	if (both_valid) {
		led_set_outside_indicator(inside, outside);
	}
}

bool ventilation_sensor_missing(void)
{
	bool    missing;
	int64_t now = k_uptime_get();

	k_mutex_lock(&v_lock, K_FOREVER);
	missing = !v_temp_valid[0] || !v_temp_valid[1] ||
		  (now - v_temp_last_seen_ms[0]) > SENSOR_STALE_TIMEOUT_MS ||
		  (now - v_temp_last_seen_ms[1]) > SENSOR_STALE_TIMEOUT_MS;
	k_mutex_unlock(&v_lock);

	return missing;
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
	buzzer_start_pattern(1000);
}
