/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 * Copyright (c) 2019 Marcio Montenegro <mtuxpe@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <zephyr.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <logging/log.h>
#include <device.h>
#include <drivers/pwm.h>
#include <logging/log.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include "button_svc.h"
#include "led_svc.h"



LOG_MODULE_REGISTER(main);

/*
#if defined(DT_ALIAS_PWM_0_LABEL)
#define PWM_DEV_NAME DT_ALIAS_PWM_0_LABEL
#elif defined(DT_ALIAS_PWM_1_LABEL)
#define PWM_DEV_NAME DT_ALIAS_PWM_1_LABEL
#elif defined(DT_ALIAS_PWM_2_LABEL)
#define PWM_DEV_NAME DT_ALIAS_PWM_2_LABEL
#elif defined(DT_ALIAS_PWM_3_LABEL)
#define PWM_DEV_NAME DT_ALIAS_PWM_3_LABEL
#else
#error "Define a PWM device"
#endif
*/



extern u16_t but_val;
extern struct device *led_dev;
extern bool led_state;

/* Prototype */
static ssize_t recv(struct bt_conn *conn,
		    const struct bt_gatt_attr *attr, const void *buf,
		    u16_t len, u16_t offset, u8_t flags);

/* ST Custom Service  */
static struct bt_uuid_128 st_service_uuid = BT_UUID_INIT_128(
	0x8f, 0xe5, 0xb3, 0xd5, 0x2e, 0x7f, 0x4a, 0x98,
	0x2a, 0x48, 0x7a, 0xcc, 0x40, 0xfe, 0x00, 0x00);

/* ST LED service */
static struct bt_uuid_128 led_char_uuid = BT_UUID_INIT_128(
	0x19, 0xed, 0x82, 0xae, 0xed, 0x21, 0x4c, 0x9d,
	0x41, 0x45, 0x22, 0x8e, 0x41, 0xfe, 0x00, 0x00);

/* ST Notify button service */
static struct bt_uuid_128 but_notif_uuid = BT_UUID_INIT_128(
	0x19, 0xed, 0x82, 0xae, 0xed, 0x21, 0x4c, 0x9d,
	0x41, 0x45, 0x22, 0x8e, 0x42, 0xfe, 0x00, 0x00);


/* My Custom Service  */
static struct bt_uuid_128 my_svc_uuid = BT_UUID_INIT_128(
	0x8f, 0xe5, 0xb3, 0xd6, 0x2e, 0x7f, 0x4a, 0x98,
	0x2a, 0x48, 0x7a, 0xcc, 0x40, 0xfe, 0x00, 0x00);
	

#ifndef CONFIG_BT_DEVICE_NAME
#define CONFIG_BT_DEVICE_NAME "Unknown"
#endif

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define ADV_LEN 12

/* Advertising data */
static u8_t manuf_data[ADV_LEN] = {
	0x01 /*SKD version */,
	0x83 /* STM32WB - P2P Server 1 */,
	0x00 /* GROUP A Feature  */,
	0x00 /* GROUP A Feature */,
	0x00 /* GROUP B Feature */,
	0x00 /* GROUP B Feature */,
	0x00, /* BLE MAC start -MSB */
	0x00,
	0x00,
	0x00,
	0x00,
	0x00, /* BLE MAC stop */
};

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, ADV_LEN)
};

/* BLE connection */
struct bt_conn *conn;
/* Notification state */
volatile bool notify_enable;

static void mpu_ccc_cfg_changed(const struct bt_gatt_attr *attr, u16_t value)
{
	ARG_UNUSED(attr);
	notify_enable = (value == BT_GATT_CCC_NOTIFY);
	//bt_addr_to_str (attr->uuid)
	//BT_DBG ("Ge");

	//char buf [128];
	//bt_uuid_to_str (attr->uuid, buf, 128);
	//printk("Discovered attribute - uuid: %s, handle: %u\n", bt_uuid_str(attr->uuid), attr->handle);

	//LOG_INF("Notification %p : %s", log_strdup (buf), notify_enable ? "enabled" : "disabled");
	LOG_INF("Notification %i : %s", attr->handle, notify_enable ? "enabled" : "disabled");
}





struct es_measurement
{
	u16_t flags; /* Reserved for Future Use */
	u8_t sampling_func;
	u32_t meas_period;
	u32_t update_interval;
	u8_t application;
	u8_t meas_uncertainty;
};

struct temperature_sensor {
	s16_t temp_value;

	/* Valid Range */
	s16_t lower_limit;
	s16_t upper_limit;

	/* ES trigger setting - Value Notification condition */
	u8_t condition;
	union
	{
		u32_t seconds;
		s16_t ref_val; /* Reference temperature */
	};

	struct es_measurement meas;
};

/* ESS Trigger Setting conditions */
#define ESS_TRIGGER_INACTIVE			    0x00
#define ESS_FIXED_TIME_INTERVAL			    0x01
#define ESS_NO_LESS_THAN_SPECIFIED_TIME		0x02
#define ESS_VALUE_CHANGED			        0x03
#define ESS_LESS_THAN_REF_VALUE			    0x04
#define ESS_LESS_OR_EQUAL_TO_REF_VALUE		0x05
#define ESS_GREATER_THAN_REF_VALUE		    0x06
#define ESS_GREATER_OR_EQUAL_TO_REF_VALUE	0x07
#define ESS_EQUAL_TO_REF_VALUE			    0x08
#define ESS_NOT_EQUAL_TO_REF_VALUE		    0x09

static struct temperature_sensor sensor_1 =
{
		.temp_value = 1200,
		.lower_limit = -10000,
		.upper_limit = 10000,
		.condition = ESS_VALUE_CHANGED,
		.meas.sampling_func = 0x00,
		.meas.meas_period = 0x01,
		.meas.update_interval = 5,
		.meas.application = 0x1c,
		.meas.meas_uncertainty = 0x04,
};


static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, u16_t len, u16_t offset)
{
	printk ("read_u16");
	const u16_t *u16 = attr->user_data;
	u16_t value = sys_cpu_to_le16(*u16);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
}

/* The embedded board is acting as GATT server.
 * The ST BLE Android app is the BLE GATT client.
 */

/* ST BLE Sensor GATT services and characteristic */




BT_GATT_SERVICE_DEFINE  (my_svc,
BT_GATT_PRIMARY_SERVICE (&my_svc_uuid),
BT_GATT_CHARACTERISTIC  (BT_UUID_TEMPERATURE,BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,BT_GATT_PERM_READ,read_u16, NULL, &sensor_1.temp_value),
BT_GATT_CCC             (mpu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
BT_GATT_CUD             ("Temperature", BT_GATT_PERM_READ),
);


BT_GATT_SERVICE_DEFINE  (stsensor_svc,
BT_GATT_PRIMARY_SERVICE (&st_service_uuid),
BT_GATT_CHARACTERISTIC  (&led_char_uuid.uuid,BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,BT_GATT_PERM_WRITE, NULL, recv, (void *)1),
BT_GATT_CUD             ("LED", BT_GATT_PERM_READ),
BT_GATT_CHARACTERISTIC  (&but_notif_uuid.uuid, BT_GATT_CHRC_NOTIFY,BT_GATT_PERM_READ, NULL, NULL, &but_val),
BT_GATT_CCC             (mpu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
BT_GATT_CUD             ("Button", BT_GATT_PERM_READ),
);

//BT_GATT_CCC             (mpu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)

static ssize_t recv (struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	if (led_dev)
	{
		if (led_state == true)
		{
			led_on_off(0);
			LOG_INF("Turn off LED");
		}
		else
		{
			led_on_off(1);
			LOG_INF("Turn on LED");
		}
		led_state = !led_state;
	}
	return 0;
}

static void bt_ready (int err)
{
	if (err)
	{
		LOG_ERR("Bluetooth init failed (err %d)", err);
		return;
	}
	LOG_INF("Bluetooth initialized");
	/* Start advertising */
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err)
	{
		LOG_ERR("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_INF("Configuration mode: waiting connections...");
}

static void connected(struct bt_conn *connected, u8_t err)
{
	if (err)
	{
		LOG_ERR("Connection failed (err %u)", err);
	}
	else
	{
		LOG_INF("Connected");
		if (!conn)
		{
			conn = bt_conn_ref(connected);
		}
	}
}

static void disconnected(struct bt_conn *disconn, u8_t reason)
{
	if (conn)
	{
		bt_conn_unref(conn);
		conn = NULL;
	}
	LOG_INF("Disconnected (reason %u)", reason);
}

static struct bt_conn_cb conn_callbacks =
{
	.connected = connected,
	.disconnected = disconnected,
};

#define PWM_DEV "PWM_2"

//#define PERIOD (USEC_PER_SEC / 50U)
#define PERIOD 1000*1000
#define MINPULSEWIDTH 700
#define PULSEWIDTH 1000*500

void main(void)
{
	int err;
	
	struct device * pwm_dev;
	pwm_dev = device_get_binding (PWM_DEV);
	if (!pwm_dev)
	{
		LOG_INF ("Cannot find PWM device %s\n", PWM_DEV);
		return;
	}
	else
	{
		LOG_INF ("Found PWM device %s\n", PWM_DEV);
	}
	
	if (pwm_pin_set_usec (pwm_dev, 1, PERIOD, PULSEWIDTH, 0))
	{
		printk ("pwm pin set fails\n");
		return;
	}

	err = button_init();
	if (err)
	{
		LOG_ERR("Button init error: (err %d)", err);
	}

	led_init();
	bt_conn_cb_register(&conn_callbacks);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(bt_ready);
	if (err)
	{
		LOG_ERR("Bluetooth init failed (err %d)", err);
	}

	while (1)
	{
		//led_on_off(0);
		//k_sleep(K_SECONDS(1));
		//led_on_off(1);
		k_sleep(K_SECONDS(1));
		
	}
}
