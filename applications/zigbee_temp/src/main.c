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
#include <zboss_api_aps.h>
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
#include <string.h>
#include "ventilation.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* --- LEDs / buttons (nRF52840 DK) ---------------------------------------- */

#define RUN_STATUS_LED          DK_LED1
#define ZIGBEE_NETWORK_LED      DK_LED3
#define IDENTIFY_LED            DK_LED4
#define VENT_BUTTON             DK_BTN1_MSK
#define NETWORK_REOPEN_BUTTON   DK_BTN2_MSK
#define VENT_TEST_BUTTON        DK_BTN3_MSK
#define FACTORY_RESET_BUTTON    DK_BTN4_MSK

/* --- Zigbee endpoint ------------------------------------------------------- */

#define COORDINATOR_ENDPOINT    1

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
 * MAX_ESS_SENSORS characteristic pairs in the ESS service — one per Zigbee
 * sensor slot.  Slot s → temperature at attrs[ESS_TEMP_ATTR_IDX(s)],
 * humidity at attrs[ESS_HUM_ATTR_IDX(s)].
 * Slots ≥ MAX_ESS_SENSORS are clamped to MAX_ESS_SENSORS-1 (last pair).
 */
#define MAX_ESS_SENSORS 2
#define ESS_TEMP_ATTR_IDX(s)  (2 + (s) * 8)
#define ESS_HUM_ATTR_IDX(s)   (6 + (s) * 8)

/*
 * Written from the ZBOSS callback thread, read from BLE GATT read callbacks.
 * int16_t and uint16_t writes are atomic on Cortex-M4 for aligned accesses,
 * but we use a mutex to also protect the notify flags.
 */
static K_MUTEX_DEFINE(sensor_lock);
static int16_t  temp_raw[MAX_ESS_SENSORS];  /* 0.01 °C, sint16 per ESS spec */
static uint16_t hum_raw[MAX_ESS_SENSORS];   /* 0.01 %,  uint16 per ESS spec */

static bool temp_notify_enabled[MAX_ESS_SENSORS];
static bool hum_notify_enabled[MAX_ESS_SENSORS];

/* --- BLE ESS GATT service -------------------------------------------------- */

/* Single read_s16/read_u16 handles all instances via attr->user_data */
static ssize_t read_s16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	int16_t val = sys_cpu_to_le16(*(int16_t *)attr->user_data);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			void *buf, uint16_t len, uint16_t offset)
{
	uint16_t val = sys_cpu_to_le16(*(uint16_t *)attr->user_data);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &val, sizeof(val));
}

static void temp_ccc_changed_0(const struct bt_gatt_attr *attr, uint16_t value)
{
	temp_notify_enabled[0] = (value == BT_GATT_CCC_NOTIFY);
}

static void temp_ccc_changed_1(const struct bt_gatt_attr *attr, uint16_t value)
{
	temp_notify_enabled[1] = (value == BT_GATT_CCC_NOTIFY);
}

static void hum_ccc_changed_0(const struct bt_gatt_attr *attr, uint16_t value)
{
	hum_notify_enabled[0] = (value == BT_GATT_CCC_NOTIFY);
}

static void hum_ccc_changed_1(const struct bt_gatt_attr *attr, uint16_t value)
{
	hum_notify_enabled[1] = (value == BT_GATT_CCC_NOTIFY);
}

/*
 * ESS attribute map (BT_GATT_CHARACTERISTIC = 2 attrs; each descriptor = 1):
 *   [0]  Primary service (ESS 0x181A)
 *   Sensor 0:
 *   [1][2]  Temp decl+value  [3] CCC  [4] CUD "Sensor 1"
 *   [5][6]  Hum  decl+value  [7] CCC  [8] CUD "Sensor 1"
 *   Sensor 1:
 *   [9][10] Temp decl+value [11] CCC [12] CUD "Sensor 2"
 *  [13][14] Hum  decl+value [15] CCC [16] CUD "Sensor 2"
 *
 *  Notify targets: ESS_TEMP_ATTR_IDX(s) = 2+s*8, ESS_HUM_ATTR_IDX(s) = 6+s*8
 */
BT_GATT_SERVICE_DEFINE(ess_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),
	/* Sensor slot 0 */
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_s16, NULL, &temp_raw[0]),
	BT_GATT_CCC(temp_ccc_changed_0, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CUD("Sensor 1", BT_GATT_PERM_READ),
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &hum_raw[0]),
	BT_GATT_CCC(hum_ccc_changed_0, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CUD("Sensor 1", BT_GATT_PERM_READ),
	/* Sensor slot 1 */
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_s16, NULL, &temp_raw[1]),
	BT_GATT_CCC(temp_ccc_changed_1, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CUD("Sensor 2", BT_GATT_PERM_READ),
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_u16, NULL, &hum_raw[1]),
	BT_GATT_CCC(hum_ccc_changed_1, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CUD("Sensor 2", BT_GATT_PERM_READ),
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

/* --- Sensor table & configure-reporting state ------------------------------ */

#define MAX_SENSORS 4

typedef struct {
	zb_uint16_t short_addr;
	uint8_t     cr_retry;
	bool        active;
} sensor_entry_t;

static sensor_entry_t sensors[MAX_SENSORS];
static zb_uint16_t    cr_pending_addr;
static zb_int16_t     temp_rep_change = 50;   /* 0.5 °C in 0.01 °C units */
static zb_uint16_t    hum_rep_change  = 100;  /* 1.0 % in 0.01 % units */

/*
 * Only the outside sensor is pinned by IEEE (MAC) address; the inside sensor
 * is automatically whichever other Sonoff joins, since only two sensor slots
 * are used (0 = inside, 1 = outside). ieee_addr is in p->long_addr index
 * order [0..7], which is the reverse of the human-readable "MAC: xx:xx:..."
 * log output (that log prints long_addr[7..0]).
 */
static const zb_uint8_t outside_ieee_addr[8] =
	{ 0x87, 0x0c, 0xa5, 0xfe, 0xff, 0x7e, 0xd0, 0x70 }; /* 70:d0:7e:ff:fe:a5:0c:87 */

static bool is_outside_sensor(const zb_uint8_t *ieee)
{
	return ieee != NULL && memcmp(outside_ieee_addr, ieee, 8) == 0;
}

static int sensor_find(zb_uint16_t addr)
{
	for (int i = 0; i < MAX_SENSORS; i++) {
		if (sensors[i].active && sensors[i].short_addr == addr) {
			return i;
		}
	}
	return -1;
}

static int sensor_alloc(zb_uint16_t addr, const zb_uint8_t *ieee)
{
	int slot = sensor_find(addr);

	if (slot >= 0) {
		return slot;
	}

	int fixed_slot = is_outside_sensor(ieee) ? 1 : 0;

	if (!sensors[fixed_slot].active) {
		sensors[fixed_slot].short_addr = addr;
		sensors[fixed_slot].active = true;
		return fixed_slot;
	}

	if (ieee == NULL) {
		LOG_WRN("Unknown IEEE for short=0x%04x, falling back to join-order slot", addr);
	}

	for (int i = 0; i < MAX_SENSORS; i++) {
		if (!sensors[i].active) {
			sensors[i].short_addr = addr;
			sensors[i].active = true;
			return i;
		}
	}
	return -1;
}

/* Forward declaration needed by zcl_ep_handler and retry_cr_alarm */
/* --- IEEE address cache (populated from DEVICE_UPDATE) --------------------- */

#define ADDR_CACHE_SIZE 8

static struct {
	zb_uint16_t short_addr;
	zb_uint8_t  ieee_addr[8];
	bool        valid;
} addr_cache[ADDR_CACHE_SIZE];

static void addr_cache_update(zb_uint16_t short_addr, const zb_uint8_t *ieee)
{
	int slot = 0;

	for (int i = 0; i < ADDR_CACHE_SIZE; i++) {
		if (!addr_cache[i].valid || addr_cache[i].short_addr == short_addr) {
			slot = i;
			break;
		}
	}
	addr_cache[slot].short_addr = short_addr;
	memcpy(addr_cache[slot].ieee_addr, ieee, 8);
	addr_cache[slot].valid = true;
}

static const zb_uint8_t *addr_cache_lookup(zb_uint16_t short_addr)
{
	for (int i = 0; i < ADDR_CACHE_SIZE; i++) {
		if (addr_cache[i].valid && addr_cache[i].short_addr == short_addr) {
			return addr_cache[i].ieee_addr;
		}
	}
	return NULL;
}

static void retry_cr_alarm(zb_uint8_t param);

/* --- ZCL endpoint handler -------------------------------------------------- */

/*
 * Called by ZBOSS for every ZCL frame arriving on COORDINATOR_ENDPOINT.
 * We parse ZCL_CMD_REPORT_ATTRIB frames from the SNZB-02P and forward
 * the values to BLE ESS notifications.
 */
static zb_uint8_t zcl_ep_handler(zb_bufid_t bufid)
{
	zb_zcl_parsed_hdr_t *hdr = ZB_BUF_GET_PARAM(bufid, zb_zcl_parsed_hdr_t);

	LOG_DBG("ZCL frame: cluster=0x%04x cmd=0x%02x src=0x%04x src_ep=%d dir=%d",
		hdr->cluster_id, hdr->cmd_id,
		hdr->addr_data.common_data.source.u.short_addr,
		hdr->addr_data.common_data.src_endpoint,
		hdr->cmd_direction);
	LOG_HEXDUMP_DBG(zb_buf_begin(bufid), MIN(zb_buf_len(bufid), 32U), "ZCL payload");

	/* Any ZCL frame: cancel retry; register if unknown.
	 * slot/ess stay in function scope so the reporting loop below can use them. */
	zb_uint16_t src = hdr->addr_data.common_data.source.u.short_addr;
	int slot = sensor_find(src);

	if (slot >= 0) {
		(void)ZB_SCHEDULE_APP_ALARM_CANCEL(retry_cr_alarm, (zb_uint8_t)slot);
	} else {
		/* Already reporting without rejoining (e.g. after coordinator
		 * reflash) — register it so the table stays consistent */
		slot = sensor_alloc(src, addr_cache_lookup(src));
		if (slot >= 0) {
			LOG_INF("Registered existing sensor 0x%04x from ZCL report", src);
		}
	}
	int ess = (slot >= 0 && slot < MAX_ESS_SENSORS) ? slot : (MAX_ESS_SENSORS - 1);

	if (hdr->cmd_id != ZB_ZCL_CMD_REPORT_ATTRIB) {
		return ZB_FALSE;
	}

	zb_zcl_report_attr_req_t *rep = NULL;

	ZB_ZCL_GENERAL_GET_NEXT_REPORT_ATTR_REQ(bufid, rep);
	while (rep != NULL) {
		k_mutex_lock(&sensor_lock, K_FOREVER);

		if (hdr->cluster_id == ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT &&
		    rep->attr_id == ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID) {
			temp_raw[ess] = *((int16_t *)rep->attr_value);
			LOG_INF("Temperature: %d.%02d C (slot=%d)",
				temp_raw[ess] / 100, abs(temp_raw[ess] % 100), ess);
			ventilation_update_temp(ess, temp_raw[ess]);
			if (temp_notify_enabled[ess]) {
				int16_t t = sys_cpu_to_le16(temp_raw[ess]);

				bt_gatt_notify(NULL,
					       &ess_svc.attrs[ESS_TEMP_ATTR_IDX(ess)],
					       &t, sizeof(t));
			}
		} else if (hdr->cluster_id == ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT &&
			   rep->attr_id == ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID) {
			hum_raw[ess] = *((uint16_t *)rep->attr_value);
			LOG_INF("Humidity: %d.%02d %% (slot=%d)",
				hum_raw[ess] / 100, hum_raw[ess] % 100, ess);
			if (hum_notify_enabled[ess]) {
				uint16_t h = sys_cpu_to_le16(hum_raw[ess]);

				bt_gatt_notify(NULL,
					       &ess_svc.attrs[ESS_HUM_ATTR_IDX(ess)],
					       &h, sizeof(h));
			}
		}

		k_mutex_unlock(&sensor_lock);
		ZB_ZCL_GENERAL_GET_NEXT_REPORT_ATTR_REQ(bufid, rep);
	}

	return ZB_FALSE;
}

/* --- Global APS data indication (diagnostic: catches ALL incoming APS frames) */

static zb_uint8_t aps_data_indication(zb_bufid_t bufid)
{
	zb_apsde_data_indication_t *ind =
		ZB_BUF_GET_PARAM(bufid, zb_apsde_data_indication_t);

	LOG_DBG("APS frame: profile=0x%04x cluster=0x%04x "
		"src=0x%04x src_ep=%d dst_ep=%d len=%d",
		ind->profileid, ind->clusterid,
		ind->src_addr, ind->src_endpoint, ind->dst_endpoint,
		zb_buf_len(bufid));
	return ZB_FALSE; /* not consumed; let normal dispatch continue */
}

/* --- Zigbee signal handler ------------------------------------------------- */

static void steering_finished(zb_uint8_t param)
{
	ARG_UNUSED(param);
	LOG_INF("Network steering finished");
	dk_set_led_off(ZIGBEE_NETWORK_LED);
}

/* --- Configure reporting --------------------------------------------------- */

/* configure_temp_reporting is called via zb_buf_get_out_delayed from retry_cr_alarm */
static void configure_temp_reporting(zb_bufid_t bufid);

static void retry_cr_alarm(zb_uint8_t param)
{
	uint8_t idx = param;

	if (idx >= MAX_SENSORS || !sensors[idx].active) {
		return;
	}
	sensors[idx].cr_retry++;
	LOG_INF("Configure reporting retry %d/10 (0x%04x)",
		sensors[idx].cr_retry, sensors[idx].short_addr);
	cr_pending_addr = sensors[idx].short_addr;
	zb_buf_get_out_delayed(configure_temp_reporting);
}

static void configure_hum_reporting(zb_bufid_t bufid)
{
	zb_uint8_t *ptr;
	int idx;

	if (!bufid) {
		return;
	}
	ZB_ZCL_GENERAL_INIT_CONFIGURE_REPORTING_SRV_REQ(bufid, ptr,
		ZB_ZCL_DISABLE_DEFAULT_RESPONSE);
	ZB_ZCL_GENERAL_ADD_SEND_REPORT_CONFIGURE_REPORTING_REQ(ptr,
		ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID,
		ZB_ZCL_ATTR_TYPE_U16, 10, 300, (zb_uint8_t *)&hum_rep_change);
	ZB_ZCL_GENERAL_SEND_CONFIGURE_REPORTING_REQ(bufid, ptr,
		cr_pending_addr, ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
		1, COORDINATOR_ENDPOINT,
		ZB_AF_HA_PROFILE_ID, ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, NULL);
	idx = sensor_find(cr_pending_addr);
	LOG_INF("Configure reporting sent: humidity 0x%04x (retry=%d)",
		cr_pending_addr, idx >= 0 ? sensors[idx].cr_retry : 0);
	/* Retry every 60 s — sleepy device may not be awake yet */
	if (idx >= 0 && sensors[idx].cr_retry < 10) {
		ZB_ERROR_CHECK(ZB_SCHEDULE_APP_ALARM(retry_cr_alarm, (zb_uint8_t)idx,
						     ZB_TIME_ONE_SECOND * 60));
	}
}

static void configure_temp_reporting(zb_bufid_t bufid)
{
	zb_uint8_t *ptr;

	if (!bufid) {
		return;
	}
	ZB_ZCL_GENERAL_INIT_CONFIGURE_REPORTING_SRV_REQ(bufid, ptr,
		ZB_ZCL_DISABLE_DEFAULT_RESPONSE);
	ZB_ZCL_GENERAL_ADD_SEND_REPORT_CONFIGURE_REPORTING_REQ(ptr,
		ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
		ZB_ZCL_ATTR_TYPE_S16, 10, 300, (zb_uint8_t *)&temp_rep_change);
	ZB_ZCL_GENERAL_SEND_CONFIGURE_REPORTING_REQ(bufid, ptr,
		cr_pending_addr, ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
		1, COORDINATOR_ENDPOINT,
		ZB_AF_HA_PROFILE_ID, ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, NULL);
	LOG_INF("Configure reporting sent: temperature 0x%04x", cr_pending_addr);
	zb_buf_get_out_delayed(configure_hum_reporting);
}

static void start_configure_reporting(zb_uint8_t param)
{
	uint8_t idx = param;

	if (idx >= MAX_SENSORS || !sensors[idx].active) {
		return;
	}
	sensors[idx].cr_retry = 0;
	cr_pending_addr = sensors[idx].short_addr;
	zb_buf_get_out_delayed(configure_temp_reporting);
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

	if ((has_changed & VENT_BUTTON) && (button_state & VENT_BUTTON)) {
		ventilation_toggle();
	}

	if ((has_changed & VENT_TEST_BUTTON) && (button_state & VENT_TEST_BUTTON)) {
		ventilation_buzz_test();
	}

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
			zb_bdb_set_legacy_device_support(1);
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
		LOG_INF("Device announced: short=0x%04hx (join complete)",
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

	/* TC update: device associated at MAC/NWK level (before key transport).
	 * status: 0=secured-rejoin 1=unsecured-join 2=left 3=tc-rejoin
	 * tc_action: 0=authorize(send key) 1=deny(send remove) 2=ignore */
	case ZB_ZDO_SIGNAL_DEVICE_UPDATE: {
		zb_zdo_signal_device_update_params_t *p =
			ZB_ZDO_SIGNAL_GET_PARAMS(
				sg, zb_zdo_signal_device_update_params_t);
		LOG_INF("TC update: short=0x%04x status=%d tc_action=%d parent=0x%04x",
			p->short_addr, p->status, p->tc_action, p->parent_short);
		LOG_INF("  MAC: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
			p->long_addr[7], p->long_addr[6], p->long_addr[5],
			p->long_addr[4], p->long_addr[3], p->long_addr[2],
			p->long_addr[1], p->long_addr[0]);
		addr_cache_update(p->short_addr, p->long_addr);
	} break;

	/* DEVICE_AUTHORIZED fires after the full Zigbee 3.0 TCLK exchange.
	 * type: 0=legacy 1=r21_tclk(ZB3.0)
	 * status(legacy): 0=ok 1=fail
	 * status(r21_tclk): 0=ok 1=timeout 2=fail */
	case ZB_ZDO_SIGNAL_DEVICE_AUTHORIZED: {
		zb_zdo_signal_device_authorized_params_t *p =
			ZB_ZDO_SIGNAL_GET_PARAMS(
				sg, zb_zdo_signal_device_authorized_params_t);
		LOG_INF("Device authorized: short=0x%04x type=%d status=%d",
			p->short_addr, p->authorization_type,
			p->authorization_status);
		if (p->authorization_status == 0) {
			const zb_uint8_t *new_ieee = addr_cache_lookup(p->short_addr);

			/* Deactivate stale slots for the same physical device
			 * (same IEEE, old short address after re-join) */
			if (new_ieee != NULL) {
				for (int i = 0; i < MAX_SENSORS; i++) {
					const zb_uint8_t *old_ieee;

					if (!sensors[i].active ||
					    sensors[i].short_addr == p->short_addr) {
						continue;
					}
					old_ieee = addr_cache_lookup(sensors[i].short_addr);
					if (old_ieee != NULL &&
					    memcmp(old_ieee, new_ieee, 8) == 0) {
						LOG_INF("Sensor 0x%04x rejoined as 0x%04x, clearing stale slot",
							sensors[i].short_addr, p->short_addr);
						(void)ZB_SCHEDULE_APP_ALARM_CANCEL(
							retry_cr_alarm, (zb_uint8_t)i);
						sensors[i].active = false;
					}
				}
			}

			int slot = sensor_alloc(p->short_addr, new_ieee);

			if (slot < 0) {
				LOG_WRN("Sensor table full, ignoring 0x%04x",
					p->short_addr);
				break;
			}
			/* Stagger by slot index to avoid simultaneous configure reporting */
			ZB_SCHEDULE_APP_ALARM(start_configure_reporting,
					      (zb_uint8_t)slot,
					      ZB_TIME_ONE_SECOND * (2 + slot * 5));
		}
	} break;

	/* Permit-join window open/close broadcast */
	case ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
		LOG_INF("Permit-join status changed: duration=%d s", status);
		break;

	case ZB_NLME_STATUS_INDICATION: {
		zb_zdo_signal_nlme_status_indication_params_t *p =
			ZB_ZDO_SIGNAL_GET_PARAMS(
				sg, zb_zdo_signal_nlme_status_indication_params_t);
		LOG_INF("NLME status: nwk_status=0x%02x addr=0x%04x",
			p->nlme_status.status, p->nlme_status.network_addr);
	} break;

	default:
		LOG_DBG("Unhandled signal %d (status %d)", sig, status);
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
	ventilation_init();

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

	/* Register global APS data indication to log all incoming frames */
	zb_af_set_data_indication(aps_data_indication);

	/* Start Zigbee — opens network automatically on first boot */
	zigbee_enable();
	LOG_INF("Zigbee started");

	while (1) {
		if (ventilation_sensor_missing()) {
			dk_set_led(RUN_STATUS_LED, (++blink) % 2);
		} else {
			dk_set_led_off(RUN_STATUS_LED);
		}
		k_sleep(K_MSEC(1000));
	}

	return 0;
}
