/*
 * SPDX-FileCopyrightText: 2025 Alicipy <dev@stefankraus.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zigbee coordinator + BLE ESS bridge for nRF52840 DK.
 * Receives temperature and humidity ZCL attribute reports from SONOFF SNZB-02P
 * and re-publishes them via BLE Environmental Sensing Service (ESS).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <dk_buttons_and_leds.h>

/* Zigbee / ZBOSS */
#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zb_mem_config_max.h>
#include <zigbee/zigbee_error_handler.h>
#include <zigbee/zigbee_app_utils.h>
#include <zb_nrf_platform.h>
#include <zcl/zb_zcl_temp_measurement.h>
#include <zcl/zb_zcl_rel_humidity_measurement.h>

/* BLE */
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* --- LEDs / buttons (nRF52840 DK) ---------------------------------------- */

#define RUN_STATUS_LED          DK_LED1
#define ZIGBEE_NETWORK_LED      DK_LED3
#define IDENTIFY_LED            DK_LED4
#define NETWORK_REOPEN_BUTTON   DK_BTN1_MSK
#define FACTORY_RESET_BUTTON    DK_BTN4_MSK

/* --- Zigbee endpoint ------------------------------------------------------- */

#define COORDINATOR_ENDPOINT    10

/* 2 server clusters (Basic + Identify) + 2 client clusters (Temp + Humidity) */
#define COORD_IN_CLUSTER_NUM    2
#define COORD_OUT_CLUSTER_NUM   2

struct zb_device_ctx {
	zb_zcl_basic_attrs_t    basic_attr;
	zb_zcl_identify_attrs_t identify_attr;
};

static struct zb_device_ctx dev_ctx;

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST(
	basic_attr_list,
	&dev_ctx.basic_attr.zcl_version,
	&dev_ctx.basic_attr.power_source);

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(
	identify_attr_list,
	&dev_ctx.identify_attr.identify_time);

/* Cluster list: Basic + Identify (server), Temp + Humidity (client) */
zb_zcl_cluster_desc_t coord_clusters[] = {
	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_BASIC,
		ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t),
		basic_attr_list,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID),
	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_IDENTIFY,
		ZB_ZCL_ARRAY_SIZE(identify_attr_list, zb_zcl_attr_t),
		identify_attr_list,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID),
	/* Client clusters for receiving attribute reports */
	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
		0, NULL,
		ZB_ZCL_CLUSTER_CLIENT_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID),
	ZB_ZCL_CLUSTER_DESC(
		ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
		0, NULL,
		ZB_ZCL_CLUSTER_CLIENT_ROLE,
		ZB_ZCL_MANUF_CODE_INVALID),
};

/* ZB_DECLARE_SIMPLE_DESC / ZB_AF_SIMPLE_DESC_TYPE use ## token-pasting, so
 * literal counts must be used here — macro constants are not expanded by ##. */
ZB_DECLARE_SIMPLE_DESC(2, 2);

ZB_AF_SIMPLE_DESC_TYPE(2, 2)
	coord_simple_desc = {
	COORDINATOR_ENDPOINT,
	ZB_AF_HA_PROFILE_ID,
	ZB_HA_RANGE_EXTENDER_DEVICE_ID,
	0,
	0,
	COORD_IN_CLUSTER_NUM,
	COORD_OUT_CLUSTER_NUM,
	ZB_ZCL_CLUSTER_ID_BASIC,
	ZB_ZCL_CLUSTER_ID_IDENTIFY,
	ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
	ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT,
};

ZB_AF_DECLARE_ENDPOINT_DESC(
	coord_ep,
	COORDINATOR_ENDPOINT,
	ZB_AF_HA_PROFILE_ID,
	0, NULL,
	ZB_ZCL_ARRAY_SIZE(coord_clusters, zb_zcl_cluster_desc_t),
	coord_clusters,
	(zb_af_simple_desc_1_1_t *)&coord_simple_desc,
	0, NULL, 0, NULL);

ZBOSS_DECLARE_DEVICE_CTX_1_EP(coord_device, coord_ep);

/* --- BLE ESS shared state -------------------------------------------------- */

/*
 * Written from the ZBOSS callback thread, read from BLE GATT read callbacks.
 * int16_t and uint16_t writes are atomic on Cortex-M4 for aligned accesses,
 * but we use a mutex to also protect the notify flags.
 */
static K_MUTEX_DEFINE(sensor_lock);
static int16_t  temp_raw;  /* 0.01 °C, sint16 per ESS spec */
static uint16_t hum_raw;   /* 0.01 %,  uint16 per ESS spec */

static bool temp_notify_enabled;
static bool hum_notify_enabled;

/* --- BLE ESS GATT service -------------------------------------------------- */

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
 * ESS attribute map:
 *   [0] Primary service (ESS 0x181A)
 *   [1] Temperature char declaration
 *   [2] Temperature char value        <- bt_gatt_notify target
 *   [3] Temperature CCC
 *   [4] Humidity char declaration
 *   [5] Humidity char value           <- bt_gatt_notify target
 *   [6] Humidity CCC
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

static void ble_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("BLE connection failed (err 0x%02x)", err);
	} else {
		LOG_INF("BLE connected");
	}
}

static void ble_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE disconnected (reason 0x%02x)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = ble_connected,
	.disconnected = ble_disconnected,
};

/* --- ZCL endpoint handler -------------------------------------------------- */

/*
 * Called by ZBOSS for every ZCL frame arriving on COORDINATOR_ENDPOINT.
 * We parse ZCL_CMD_REPORT_ATTRIB frames from the SNZB-02P and forward
 * the values to BLE ESS notifications.
 */
static zb_uint8_t zcl_ep_handler(zb_bufid_t bufid)
{
	zb_zcl_parsed_hdr_t *hdr = ZB_BUF_GET_PARAM(bufid, zb_zcl_parsed_hdr_t);

	if (hdr->cmd_id != ZB_ZCL_CMD_REPORT_ATTRIB) {
		return ZB_FALSE;
	}

	zb_zcl_report_attr_req_t *rep = NULL;

	ZB_ZCL_GENERAL_GET_NEXT_REPORT_ATTR_REQ(bufid, rep);
	while (rep != NULL) {
		k_mutex_lock(&sensor_lock, K_FOREVER);

		if (hdr->cluster_id == ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT &&
		    rep->attr_id == ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID) {
			temp_raw = *((int16_t *)rep->attr_value);
			LOG_INF("Temperature: %d.%02d C",
				temp_raw / 100, abs(temp_raw % 100));
			if (temp_notify_enabled) {
				int16_t t = sys_cpu_to_le16(temp_raw);

				bt_gatt_notify(NULL, &ess_svc.attrs[2],
					       &t, sizeof(t));
			}
		} else if (hdr->cluster_id == ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT &&
			   rep->attr_id == ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID) {
			hum_raw = *((uint16_t *)rep->attr_value);
			LOG_INF("Humidity: %d.%02d %%",
				hum_raw / 100, hum_raw % 100);
			if (hum_notify_enabled) {
				uint16_t h = sys_cpu_to_le16(hum_raw);

				bt_gatt_notify(NULL, &ess_svc.attrs[5],
					       &h, sizeof(h));
			}
		}

		k_mutex_unlock(&sensor_lock);
		ZB_ZCL_GENERAL_GET_NEXT_REPORT_ATTR_REQ(bufid, rep);
	}

	return ZB_FALSE;
}

/* --- Zigbee signal handler ------------------------------------------------- */

static void steering_finished(zb_uint8_t param)
{
	ARG_UNUSED(param);
	LOG_INF("Network steering finished");
	dk_set_led_off(ZIGBEE_NETWORK_LED);
}

static void identify_cb(zb_bufid_t bufid)
{
	static int blink;

	if (bufid) {
		dk_set_led(IDENTIFY_LED, (++blink) % 2);
		ZB_SCHEDULE_APP_ALARM(identify_cb, bufid,
				      ZB_MILLISECONDS_TO_BEACON_INTERVAL(100));
	} else {
		zb_ret_t err = ZB_SCHEDULE_APP_ALARM_CANCEL(identify_cb,
							     ZB_ALARM_ANY_PARAM);
		ZVUNUSED(err);
		dk_set_led(IDENTIFY_LED, 0);
	}
}

static void button_changed(uint32_t button_state, uint32_t has_changed)
{
	uint32_t buttons = button_state & has_changed;
	zb_bool_t comm_status;

	if (buttons & NETWORK_REOPEN_BUTTON) {
		(void)ZB_SCHEDULE_APP_ALARM_CANCEL(steering_finished,
						   ZB_ALARM_ANY_PARAM);
		comm_status = bdb_start_top_level_commissioning(
			ZB_BDB_NETWORK_STEERING);
		if (comm_status) {
			LOG_INF("Network steering restarted");
		}
	}

	if (FACTORY_RESET_BUTTON & has_changed) {
		if (!(FACTORY_RESET_BUTTON & button_state)) {
			if (!was_factory_reset_done()) {
				ZB_SCHEDULE_APP_CALLBACK(
					(zb_callback_t)zb_bdb_finding_binding_target,
					COORDINATOR_ENDPOINT);
			}
		}
	}

	check_factory_reset_button(button_state, has_changed);
}

void zboss_signal_handler(zb_bufid_t bufid)
{
	zb_zdo_app_signal_hdr_t *sg = NULL;
	zb_zdo_app_signal_type_t sig = zb_get_app_signal(bufid, &sg);
	zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(bufid);
	zb_ret_t err;
	zb_bool_t comm_status;
	zb_time_t timeout_bi;

	switch (sig) {
	case ZB_BDB_SIGNAL_DEVICE_REBOOT:
		if (status == RET_OK) {
			LOG_INF("Coordinator started, opening network");
			comm_status = bdb_start_top_level_commissioning(
				ZB_BDB_NETWORK_STEERING);
			ZB_COMM_STATUS_CHECK(comm_status);
		} else {
			LOG_ERR("Zigbee stack init failed (status: %d)", status);
		}
		break;

	case ZB_BDB_SIGNAL_STEERING:
		if (status == RET_OK) {
			LOG_INF("Network steering started (180 s open)");
			err = ZB_SCHEDULE_APP_ALARM(
				steering_finished, 0,
				ZB_TIME_ONE_SECOND *
				ZB_ZGP_DEFAULT_COMMISSIONING_WINDOW);
			ZB_ERROR_CHECK(err);
		}
		break;

	case ZB_ZDO_SIGNAL_DEVICE_ANNCE: {
		zb_zdo_signal_device_annce_params_t *params =
			ZB_ZDO_SIGNAL_GET_PARAMS(
				sg, zb_zdo_signal_device_annce_params_t);
		LOG_INF("Device joined: short=0x%04hx",
			params->device_short_addr);
		/* Extend the steering window so more devices can join */
		err = ZB_SCHEDULE_APP_ALARM_CANCEL(steering_finished,
						   ZB_ALARM_ANY_PARAM);
		if (err == RET_OK) {
			err = ZB_SCHEDULE_APP_ALARM(
				steering_finished, 0,
				ZB_TIME_ONE_SECOND *
				ZB_ZGP_DEFAULT_COMMISSIONING_WINDOW);
			ZB_ERROR_CHECK(err);
		}
	} break;

	default:
		ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));
		break;
	}

	if (ZB_JOINED() &&
	    ZB_SCHEDULE_GET_ALARM_TIME(steering_finished, ZB_ALARM_ANY_PARAM,
				       &timeout_bi) == RET_OK) {
		dk_set_led_on(ZIGBEE_NETWORK_LED);
	} else {
		dk_set_led_off(ZIGBEE_NETWORK_LED);
	}

	if (bufid) {
		zb_buf_free(bufid);
	}
}

/* --- main ------------------------------------------------------------------ */

int main(void)
{
	int err;
	int blink = 0;

	LOG_INF("Zigbee coordinator + BLE ESS starting");

	/* DK LEDs and button */
	err = dk_buttons_init(button_changed);
	if (err) {
		LOG_ERR("dk_buttons_init failed (%d)", err);
	}
	err = dk_leds_init();
	if (err) {
		LOG_ERR("dk_leds_init failed (%d)", err);
	}
	register_factory_reset_button(FACTORY_RESET_BUTTON);

	/* Zigbee */
	ZB_AF_REGISTER_DEVICE_CTX(&coord_device);
	dev_ctx.basic_attr.zcl_version  = ZB_ZCL_VERSION;
	dev_ctx.basic_attr.power_source = ZB_ZCL_BASIC_POWER_SOURCE_DC_SOURCE;
	dev_ctx.identify_attr.identify_time =
		ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;
	ZB_AF_SET_IDENTIFY_NOTIFICATION_HANDLER(COORDINATOR_ENDPOINT, identify_cb);
	ZB_AF_SET_ENDPOINT_HANDLER(COORDINATOR_ENDPOINT, zcl_ep_handler);

	/* BLE */
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bt_enable failed (%d)", err);
		return err;
	}
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		LOG_ERR("bt_le_adv_start failed (%d)", err);
		return err;
	}
	LOG_INF("BLE advertising started (ESS)");

	/* Start Zigbee — opens network automatically on first boot */
	zigbee_enable();
	LOG_INF("Zigbee started");

	while (1) {
		dk_set_led(RUN_STATUS_LED, (++blink) % 2);
		k_sleep(K_MSEC(1000));
	}

	return 0;
}
