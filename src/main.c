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
#define TMC2130_STEP_DEV_NAME DT_ALIAS_PWM_0_LABEL
#elif defined(DT_ALIAS_PWM_1_LABEL)
#define TMC2130_STEP_DEV_NAME DT_ALIAS_PWM_1_LABEL
#elif defined(DT_ALIAS_PWM_2_LABEL)
#define TMC2130_STEP_DEV_NAME DT_ALIAS_PWM_2_LABEL
#elif defined(DT_ALIAS_PWM_3_LABEL)
#define TMC2130_STEP_DEV_NAME DT_ALIAS_PWM_3_LABEL
#else
#error "Define a PWM device"
#endif
*/



extern u16_t but_val;
extern struct device *led_dev;
extern bool led_state;

/* ST Custom Service  */
static struct bt_uuid_128 service_stepper_uuid = BT_UUID_INIT_128(
	0x8f, 0xe5, 0xb3, 0xd5, 0x2e, 0x7f, 0x4a, 0x98,
	0x2a, 0x48, 0x7a, 0xcc, 0x40, 0xfe, 0x00, 0x00);

/* My Custom Service  */
static struct bt_uuid_128 service_helloworld_uuid = BT_UUID_INIT_128(
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




//TMC2130 pin STEP ---yellow--- (PA0,A3,15)
#define TMC2130_STEP_DEV "PWM_2"
#define TMC2130_STEP_PIN 0
#define TMC2130_STEP_PORT "GPIOA"

//#define PERIOD (USEC_PER_SEC / 50U)
#define PERIOD 1000*100
#define MINPULSEWIDTH 700
#define PULSEWIDTH 1000*50

//TMC2130 pin DIR ---orange--- (PA2,D1,17)
#define TMC2130_DIR_PORT "GPIOA"
#define TMC2130_DIR_PIN 2

//TMC2130 pin EN ---brown--- (PC6,D2,50)
#define TMC2130_EN_PORT "GPIOC"
#define TMC2130_EN_PIN 6

//TMC2130 pin CS ---blue--- (PA10,D3,51)
#define TMC2130_CS_PORT "GPIOA"
#define TMC2130_CS_PIN 10

#define TMC2130_SPI_DEV "SPI_1"

//TMC2130 pin CSK --orange--- (PA5,D13,20,SCK)
#define TMC2130_SCK_PORT "GPIOA"
#define TMC2130_SCK_PIN 5

//TMC2130 pin SDO --yellow--- (PA6,D13,21,MISO)
#define TMC2130_SDO_PORT "GPIOA"
#define TMC2130_SDO_PIN 6

//TMC2130 pin SDI --green--- (PA7,D11,22,MOSI)
#define TMC2130_SDI_PORT "GPIOA"
#define TMC2130_SDI_PIN 7


struct tmc2130
{
	struct device * dev_pwm_step;
	struct device * dev_gpio_dir;
	struct device * dev_gpio_en;
	struct device * dev_spi;
	u32_t period;
};

struct device * tmc2130_get_dev (char const * name)
{
	struct device * dev;
	dev = device_get_binding (name);
	if (dev == NULL)
	{
		LOG_INF ("Cannot find device %s", name);
		return NULL;
	}
	else
	{
		LOG_INF ("Found device %s", name);
	}
	return dev;
}

void tmc2130_init (struct tmc2130 * dev)
{
	int ret;
	dev->period = PERIOD;
	dev->dev_pwm_step = tmc2130_get_dev (TMC2130_STEP_DEV);
	dev->dev_gpio_dir = tmc2130_get_dev (TMC2130_DIR_PORT);
	dev->dev_gpio_en = tmc2130_get_dev (TMC2130_EN_PORT);
	dev->dev_spi = tmc2130_get_dev (TMC2130_SPI_DEV);
	if (pwm_pin_set_usec (dev->dev_pwm_step, 1, PERIOD, PULSEWIDTH, 0))
	{
		printk ("PWM pin set fails\n");
		return;
	}
	ret = gpio_pin_configure (dev->dev_gpio_dir, TMC2130_DIR_PIN, GPIO_OUTPUT_INACTIVE);
	if (ret < 0)
	{
		LOG_ERR ("Error %d: failed to configure pin %s.%d\n", ret, TMC2130_DIR_PORT, TMC2130_EN_PIN);
		return;
	}
	ret = gpio_pin_configure (dev->dev_gpio_en, TMC2130_EN_PIN, GPIO_OUTPUT_INACTIVE);
	if (ret < 0)
	{
		LOG_ERR ("Error %d: failed to configure pin %s.%d\n", ret, TMC2130_EN_PORT, TMC2130_EN_PIN);
		return;
	}
}


struct tmc2130 tmc;






static ssize_t direction_recv (struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	uint8_t const * v = buf;
	LOG_INF ("Changing DIR %i : %i", len, v[0]);
	gpio_pin_set (tmc.dev_gpio_dir, TMC2130_DIR_PIN, v[0]);
	return 0;
}

static ssize_t enable_recv (struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	uint8_t const * v = buf;
	LOG_INF ("Changing EN %i : %i", len, v[0]);
	gpio_pin_set (tmc.dev_gpio_en, TMC2130_EN_PIN, v[0]);
	return 0;
}

static ssize_t period_recv (struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	uint8_t const * v = buf;
	LOG_INF ("Changing period %i : %i", len, v[0]);
	return 0;
}


static ssize_t read_u32(struct bt_conn *conn, const struct bt_gatt_attr *attr,void *buf, u16_t len, u16_t offset)
{
	const u32_t *u32 = attr->user_data;
	u32_t value = sys_cpu_to_le32(*u32);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value,sizeof(value));
}


static struct bt_gatt_cpf cha_format_value =
{
	8, //<Enumeration key="8" value="unsigned 32-bit integer"/>
	0x00,
	0x3000,
	0x01,
	0x0000
};


BT_GATT_SERVICE_DEFINE 
(
service_stepper,
BT_GATT_PRIMARY_SERVICE (&service_stepper_uuid),
BT_GATT_CHARACTERISTIC  (BT_UUID_DECLARE_16(0x2A56), BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE, NULL, direction_recv, (void *)1),
BT_GATT_CUD             ("Pin DIR", BT_GATT_PERM_READ),
BT_GATT_CHARACTERISTIC  (BT_UUID_DECLARE_16(0x2A56), BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE, NULL, enable_recv, (void *)1),
BT_GATT_CUD             ("Pin EN", BT_GATT_PERM_READ),
BT_GATT_CHARACTERISTIC  (BT_UUID_DECLARE_16(0x2A56), BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE|BT_GATT_PERM_READ, read_u32, period_recv, &tmc.period),
BT_GATT_CPF(&cha_format_value),
BT_GATT_CUD             ("Pin STEP period", BT_GATT_PERM_READ),
);


BT_GATT_SERVICE_DEFINE 
(
service_helloworld,
BT_GATT_PRIMARY_SERVICE (&service_helloworld_uuid),
BT_GATT_CHARACTERISTIC  (BT_UUID_DECLARE_16(0x2A56),BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,BT_GATT_PERM_WRITE, NULL, led_recv, (void *)1),
BT_GATT_CUD             ("LED", BT_GATT_PERM_READ),
BT_GATT_CHARACTERISTIC  (BT_UUID_DECLARE_16(0x2A56), BT_GATT_CHRC_NOTIFY,BT_GATT_PERM_READ, NULL, NULL, &but_val),
BT_GATT_CCC             (mpu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
BT_GATT_CUD             ("Button", BT_GATT_PERM_READ),
);

//BT_GATT_CCC             (mpu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)





static void bt_ready (int err)
{
	if (err)
	{
		LOG_ERR ("Bluetooth init failed (err %d)", err);
		return;
	}
	LOG_INF ("Bluetooth initialized");
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




void main(void)
{
	int ret;
	tmc2130_init (&tmc);
	

	ret = button_init();
	if (ret)
	{
		LOG_ERR ("Button init error: (err %d)", ret);
	}

	led_init();
	bt_conn_cb_register(&conn_callbacks);
	ret = bt_enable(bt_ready);
	if (ret)
	{
		LOG_ERR ("Bluetooth init failed (err %d)", ret);
	}

	while (1)
	{
		//led_on_off(0);
		k_sleep(K_SECONDS(1));
		//led_on_off(1);
		//k_sleep(K_SECONDS(1));
		//gpio_pin_set (gpiodev, TMC2130_DIR_PIN, 0);
		//k_sleep(K_SECONDS(1));
		//gpio_pin_set (gpiodev, TMC2130_DIR_PIN, 1);
	}
}
