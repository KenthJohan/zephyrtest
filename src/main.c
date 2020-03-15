/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 * Copyright (c) 2019 Marcio Montenegro <mtuxpe@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

#include <zephyr.h>
#include <zephyr/types.h>

#include <sys/printk.h>
#include <sys/byteorder.h>
#include <sys/util.h>

#include <devicetree.h>

#include <device.h>

#include <logging/log.h>

#include <drivers/pwm.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>

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

//TMC2130 pin SDO --yellow--- (PA6,D12,21,MISO)
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
	struct device * dev_gpio_cs;
	struct device * dev_spi;
	struct spi_config spi_cfg;
	u32_t period;
	u32_t pulse;
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

#define WRITE_FLAG     (1<<7) //write flag
#define READ_FLAG      (0<<7) //read flag
#define REG_GCONF      0x00
#define REG_GSTAT      0x01
#define REG_IHOLD_IRUN 0x10
#define REG_CHOPCONF   0x6C
#define REG_COOLCONF   0x6D
#define REG_DCCTRL     0x6E
#define REG_DRVSTATUS  0x6F

#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

#define TMC2130_RESET_FLAG 0x01
#define TMC2130_DRIVER_ERROR 0x02
#define TMC2130_SG2 0x04
#define TMC2130_STANDSTILL 0x08

void tmc_info_status (u8_t s)
{
	printf ("%sreset|", (s & TMC2130_RESET_FLAG)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%serror|", (s & TMC2130_DRIVER_ERROR)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%ssg2|", (s & TMC2130_SG2)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%sstandstill", (s & TMC2130_STANDSTILL)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf (ANSI_COLOR_RESET"\n");
}

u8_t tmc_write (struct tmc2130 * dev, u8_t cmd, u32_t data)
{
	//TMC2130 MSB big-endian
	u8_t s;
	data = sys_cpu_to_be32 (data);
	gpio_pin_set (dev->dev_gpio_cs, TMC2130_CS_PIN, 0);
	struct spi_buf bufs_tx[] =
	{
	{
		.buf = &cmd,
		.len = sizeof(cmd)
	},
	{
		.buf = &data,
		.len = sizeof(data)
	}
};
	struct spi_buf bufs_rx[] =
	{
	{
		.buf = &s,
		.len = sizeof(s)
	}};
	struct spi_buf_set tx =
	{
		.buffers = bufs_tx,
		.count = 2
	};
	struct spi_buf_set rx =
	{
		.buffers = bufs_rx,
		.count = 1
	};
	spi_transceive (dev->dev_spi, &dev->spi_cfg, &tx, &rx);
	gpio_pin_set (dev->dev_gpio_cs, TMC2130_CS_PIN, 1);
	LOG_INF ("Write %08x %08x => %02x", cmd, data, s);
	tmc_info_status (s);
	return s;
}

u8_t tmc_read (struct tmc2130 * dev, u8_t cmd, u32_t * data)
{
	tmc_write (dev, cmd, 0);
	u8_t s;
	gpio_pin_set (dev->dev_gpio_cs, TMC2130_CS_PIN, 0);
	struct spi_buf bufs_tx[] =
	{
	{
		.buf = &cmd,
		.len = sizeof(cmd)
	}
};
	struct spi_buf bufs_rx[] =
	{
	{
		.buf = &s,
		.len = sizeof(s)
	},
	{
		.buf = &data,
		.len = sizeof(data)
	}
};
	struct spi_buf_set tx =
	{
		.buffers = bufs_tx,
		.count = 1
	};
	struct spi_buf_set rx =
	{
		.buffers = bufs_rx,
		.count = 2
	};
	spi_transceive (dev->dev_spi, &dev->spi_cfg, &tx, &rx);
	gpio_pin_set (dev->dev_gpio_cs, TMC2130_CS_PIN, 1);
	*data = sys_be32_to_cpu (*data);
	LOG_INF ("Read %08x => %08x %02x", cmd, *data, s);
	tmc_info_status (s);
	return s;
}

void tmc2130_init (struct tmc2130 * dev)
{
	int ret;
	dev->period = PERIOD;
	dev->pulse = PULSEWIDTH;
	dev->dev_pwm_step = tmc2130_get_dev (TMC2130_STEP_DEV);
	dev->dev_gpio_dir = tmc2130_get_dev (TMC2130_DIR_PORT);
	dev->dev_gpio_en = tmc2130_get_dev (TMC2130_EN_PORT);
	dev->dev_gpio_cs = tmc2130_get_dev (TMC2130_CS_PORT);
	dev->dev_spi = tmc2130_get_dev (TMC2130_SPI_DEV);
	if (pwm_pin_set_usec (dev->dev_pwm_step, 1, dev->period, dev->pulse, 0))
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
	ret = gpio_pin_configure (dev->dev_gpio_en, TMC2130_EN_PIN, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		LOG_ERR ("Error %d: failed to configure pin %s.%d\n", ret, TMC2130_EN_PORT, TMC2130_EN_PIN);
		return;
	}
	ret = gpio_pin_configure (dev->dev_gpio_cs, TMC2130_CS_PIN, GPIO_OUTPUT_ACTIVE);
	if (ret < 0)
	{
		LOG_ERR ("Error %d: failed to configure pin %s.%d\n", ret, TMC2130_CS_PORT, TMC2130_CS_PIN);
		return;
	}

	dev->spi_cfg.operation = SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB;
	dev->spi_cfg.frequency = 1000000;

	//uint32_t data = 0;
	//tmc_read (dev, REG_DRVSTATUS, &data);
	//tmc_read (dev, REG_GSTAT, &data);

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
	u32_t const * u32 = buf;
	tmc.period = sys_cpu_to_le32 (*u32);
	LOG_INF ("Changing period to %u", tmc.period);
	return 0;
}

static ssize_t reg_recv (struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	u32_t data;
	tmc_read (&tmc, REG_DRVSTATUS, &data);
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
	0x2703,//0x2703	time (second)	org.bluetooth.unit.time.second
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
BT_GATT_CPF             (&cha_format_value),
BT_GATT_CUD             ("Pin STEP period", BT_GATT_PERM_READ),
BT_GATT_CHARACTERISTIC  (BT_UUID_DECLARE_16(0x2A56), BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE, NULL, reg_recv, (void *)1),
BT_GATT_CUD             ("reg_recv", BT_GATT_PERM_READ),
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
	tmc_info_status (0x00);

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
