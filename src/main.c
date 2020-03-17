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
#include "tmc2130.h"


LOG_MODULE_REGISTER(main);



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







struct tmc2130 tmc;






static ssize_t recv_tmc_dir (struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	uint8_t const * v = buf;
	tmc2130_set_dir (&tmc, v[0]);
	LOG_INF ("Changing DIR to %i", tmc.flags & TMC2130_DIR_FLAG ? 1 : 0);
	return 0;
}

static ssize_t recv_tmc_en (struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	uint8_t const * v = buf;
	tmc2130_set_en (&tmc, v[0]);
	LOG_INF ("Changing EN to %i", tmc.flags & TMC2130_EN_FLAG ? 1 : 0);
	return 0;
}

static ssize_t recv_tmc_period (struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	u32_t const * u32 = buf;
	tmc2130_set_period (&tmc, sys_cpu_to_le32 (*u32));
	LOG_INF ("Changing period to %u", tmc.period);
	return 0;
}

static ssize_t recv_tmc_pulse (struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	u32_t const * u32 = buf;
	tmc2130_set_pulse (&tmc, sys_cpu_to_le32 (*u32));
	LOG_INF ("Changing pulse to %u", tmc.pulse);
	return 0;
}

static ssize_t recv_tmc_reg (struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	tmc2130_set_en (&tmc, 0);
	u32_t data1;
	u32_t data2;
	tmc2130_read (&tmc, REG_GSTAT, &data1);
	tmc2130_read (&tmc, REG_DRVSTATUS, &data2);
	tmc2130_info_DRVSTATUS (data2);
	tmc2130_write (&tmc, WRITE_FLAG|REG_CHOPCONF, 0x00008008UL);
	tmc2130_set_en (&tmc, 1);
	return 0;
}


static ssize_t read_u32 (struct bt_conn *conn, const struct bt_gatt_attr *attr,void *buf, u16_t len, u16_t offset)
{
	const u32_t *u32 = attr->user_data;
	u32_t value = sys_cpu_to_le32(*u32);
	return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
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
BT_GATT_CHARACTERISTIC  (BT_UUID_DECLARE_16(0x2A56), BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE, NULL, recv_tmc_dir, (void *)1),
BT_GATT_CUD             ("Pin DIR", BT_GATT_PERM_READ),
BT_GATT_CHARACTERISTIC  (BT_UUID_DECLARE_16(0x2A56), BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE, NULL, recv_tmc_en, (void *)1),
BT_GATT_CUD             ("Pin EN", BT_GATT_PERM_READ),
BT_GATT_CHARACTERISTIC  (BT_UUID_DECLARE_16(0x2A56), BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE|BT_GATT_PERM_READ, read_u32, recv_tmc_period, &tmc.period),
BT_GATT_CPF             (&cha_format_value),
BT_GATT_CUD             ("Pin STEP period", BT_GATT_PERM_READ),
BT_GATT_CHARACTERISTIC  (BT_UUID_DECLARE_16(0x2A56), BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE|BT_GATT_PERM_READ, read_u32, recv_tmc_pulse, &tmc.pulse),
BT_GATT_CPF             (&cha_format_value),
BT_GATT_CUD             ("Pin STEP pulse", BT_GATT_PERM_READ),
BT_GATT_CHARACTERISTIC  (BT_UUID_DECLARE_16(0x2A56), BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP, BT_GATT_PERM_WRITE, NULL, recv_tmc_reg, (void *)1),
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
	tmc2130_info_status (0x00);

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

	tmc2130_write (&tmc, WRITE_FLAG|REG_GCONF,      0x00000001UL); //voltage on AIN is current reference
	tmc2130_write (&tmc, WRITE_FLAG|REG_IHOLD_IRUN, 0x00001010UL); //IHOLD=0x10, IRUN=0x10
	tmc2130_write (&tmc, WRITE_FLAG|REG_CHOPCONF,   0x00008008UL); //native 256 microsteps, MRES=0, TBL=1=24, TOFF=8
	tmc2130_set_en (&tmc, 0);

	while (1)
	{
		k_sleep(K_SECONDS(2));
		tmc2130_set_en (&tmc, 1);
		u32_t data1;
		u32_t data2;
		u8_t s;
		s = tmc2130_read (&tmc, REG_GSTAT, &data1);
		printf ("REG_GSTAT     0x%02X 0x%08X\n", s, data1);
		s = tmc2130_read (&tmc, REG_DRVSTATUS, &data2);
		printf ("REG_DRVSTATUS 0x%02X 0x%08X\n", s, data2);
		tmc2130_info_DRVSTATUS (data2);
		tmc2130_set_en (&tmc, 0);

		//tmc2130_info_drv_status (data2);
		//tmc2130_write (&tmc, WRITE_FLAG|REG_CHOPCONF, 0x00008008UL);
		//tmc2130_set_en (&tmc, 1);
	}
}
