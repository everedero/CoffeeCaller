/*
 * SPDX-License-Identifier: Apache-2.0
 *
 * Temperature sensor and ventilation alarm
 */

#include "ventilation.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/led_strip.h>

LOG_MODULE_REGISTER(ventilation, LOG_LEVEL_INF);

static const struct device *pwm0_dev;
static const struct device *strip_dev;

/* Choose the buzzer freq */
#define BUZZER_FREQ_HZ    880
#define BUZZER_PERIOD_NS  (NSEC_PER_SEC / 880U)
/* Continuous long buzz for window opening */
#define VENT_BUZZ_DURATION_MS 10000
/* Beep beep buzz for window closing */
#define VENT_BEEP_ON_MS       200
#define VENT_BEEP_OFF_MS      200

#define STRIP_NUM_LEDS    4

static struct led_rgb pixels[STRIP_NUM_LEDS];
/*
 * Creating a mutex to guard pixels[] and led_strip_update_rgb().
 * Two concurrent thread use the GPIO transmission: system workqueue for button
 * press, and Zigbee thread for led_set_outside_indicator.
 * GPIO transmissions on the same wire corrupt each other's pixel data.
 */
static K_MUTEX_DEFINE(led_lock);

static K_MUTEX_DEFINE(v_lock);
static int16_t v_temp[2];             /* centidegrees; slot 0=inside, 1=outside */
static bool    v_temp_valid[2];       /* true once the slot has received at least one report */
static int64_t v_temp_last_seen_ms[2]; /* uptime of the slot's last report */

#define SAMPLE_PERIOD_S 60 /* Sample every minute */
#define HISTORY_SIZE    20 /* 20 samples x 60 s = 20-minute rolling window */

/* Sensor considered missing/stale if silent longer than this
 * LED1 blinks if a sensor error is detected
 */
#define SENSOR_STALE_TIMEOUT_MS (10 * 60 * 1000)

static int16_t diff_hist[HISTORY_SIZE];
static int     hist_idx;
static int     hist_count;

static bool    buzz_enabled;
/* Negative initial value ensures cooldown is not active at first trigger */
static int64_t last_buzz_uptime_ms = -3600000LL;

static struct k_timer          sample_timer;
static struct k_work           sample_work;
static struct k_work_delayable buzz_pattern_work;

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

/*
 * LED strip first LED is only on when the buzzer is active.
 * LED second LED is active as soon as both sensors are detected, and
 * tells if temperature is hotter and fresher outside.
 * We store state to update all the LEDs from the LED strip at the same time.
 */
static struct led_rgb buzz_en_color;
static struct led_rgb outside_color;

static void leds_update(void)
{
	k_mutex_lock(&led_lock, K_FOREVER);
	pixels[0] = buzz_en_color;
	pixels[1] = outside_color;
	led_strip_update_rgb(strip_dev, pixels, STRIP_NUM_LEDS);
	k_mutex_unlock(&led_lock);
}

static void led_set_buzzer(bool enabled)
{
	buzz_en_color = enabled ? (struct led_rgb){.r = 128, .g = 0, .b = 0}
			     : (struct led_rgb){.r = 0, .g = 0, .b = 0};
	leds_update();
}

static void led_set_outside_indicator(int16_t inside, int16_t outside)
{
	outside_color = (outside <= inside)
				? (struct led_rgb){.r = 0, .g = 0, .b = 128}   /* blue: fresher outside */
				: (struct led_rgb){.r = 160, .g = 60, .b = 0}; /* orange: hotter outside */
	leds_update();
}

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

	if (!buzz_enabled) {
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

/* Public API */

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

	led_set_buzzer(false);

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

void vent_buzzer_toggle(void)
{
	buzz_enabled = !buzz_enabled;
	led_set_buzzer(buzz_enabled);
	LOG_INF("Ventilation alarm %s", buzz_enabled ? "ENABLED" : "DISABLED");
}

bool vent_buzzer_is_enabled(void)
{
	return buzz_enabled;
}

void ventilation_buzz_test(void)
{
	LOG_INF("Buzzer test");
	buzzer_start_pattern(1000);
}
