/*
 * SPDX-FileCopyrightText: 2025 Alicipy <dev@stefankraus.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(temp_ble);

/* ESS characteristic values (little-endian, updated each cycle) */
static int16_t  temp_raw; /* 0.01 °C, sint16 per BT spec */
static uint16_t hum_raw;  /* 0.01 %, uint16 per BT spec */

static bool temp_notify_enabled;
static bool hum_notify_enabled;

static ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			 void *buf, uint16_t len, uint16_t offset)
{
	int16_t val = sys_cpu_to_le16(temp_raw);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static void temp_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	temp_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

static ssize_t read_hum(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	uint16_t val = sys_cpu_to_le16(hum_raw);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static void hum_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	hum_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
}

/*
 * Environmental Sensing Service (ESS, 0x181A)
 * Attribute map:
 *   [0] Primary service
 *   [1] Temperature characteristic declaration
 *   [2] Temperature characteristic value   <- notify target
 *   [3] Temperature CCC descriptor
 *   [4] Humidity characteristic declaration
 *   [5] Humidity characteristic value      <- notify target
 *   [6] Humidity CCC descriptor
 */
BT_GATT_SERVICE_DEFINE(ess_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_temp, NULL, &temp_raw),
	BT_GATT_CCC(temp_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_hum, NULL, &hum_raw),
	BT_GATT_CCC(hum_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_ESS_VAL)),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err 0x%02x)", err);
	} else {
		LOG_INF("BLE connected");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE disconnected (reason 0x%02x)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected,
	.disconnected = disconnected,
};

int main(void)
{
	int ret;

	/* USB CDC-ACM: enable so the host enumerates the serial port */
	ret = usb_enable(NULL);
	if (ret && ret != -EALREADY) {
		LOG_ERR("USB enable failed (%d)", ret);
	}
	/* Wait for host to open the port before logging */
	k_sleep(K_SECONDS(1));

	/* BLE */
	ret = bt_enable(NULL);
	if (ret) {
		LOG_ERR("BLE init failed (%d)", ret);
		return ret;
	}

	ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (ret) {
		LOG_ERR("Advertising start failed (%d)", ret);
		return ret;
	}
	LOG_INF("BLE advertising started (ESS)");

	/* SHT-40 sensor */
	const struct device *sht_dev = DEVICE_DT_GET_ANY(sensirion_sht4x);

	if (!device_is_ready(sht_dev)) {
		LOG_ERR("SHT4x sensor not ready");
		return -ENODEV;
	}
	LOG_INF("SHT4x ready: %s", sht_dev->name);

	while (1) {
		struct sensor_value temp, hum;

		ret = sensor_sample_fetch(sht_dev);
		if (ret) {
			LOG_ERR("Sensor fetch failed (%d)", ret);
			goto next;
		}

		ret = sensor_channel_get(sht_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		if (ret) {
			LOG_ERR("Temperature read failed (%d)", ret);
			goto next;
		}

		ret = sensor_channel_get(sht_dev, SENSOR_CHAN_HUMIDITY, &hum);
		if (ret) {
			LOG_ERR("Humidity read failed (%d)", ret);
			goto next;
		}

		temp_raw = (int16_t)(sensor_value_to_double(&temp) * 100.0);
		hum_raw  = (uint16_t)(sensor_value_to_double(&hum)  * 100.0);

		LOG_INF("Temperature: %d.%02d C | Humidity: %d.%02d %%",
			temp_raw / 100, temp_raw % 100,
			hum_raw  / 100, hum_raw  % 100);

		if (temp_notify_enabled) {
			int16_t t = sys_cpu_to_le16(temp_raw);

			bt_gatt_notify(NULL, &ess_svc.attrs[2], &t, sizeof(t));
		}

		if (hum_notify_enabled) {
			uint16_t h = sys_cpu_to_le16(hum_raw);

			bt_gatt_notify(NULL, &ess_svc.attrs[5], &h, sizeof(h));
		}

next:
		k_sleep(K_SECONDS(2));
	}

	return 0;
}
