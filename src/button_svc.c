#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <logging/log.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include "button_svc.h"


#include <devicetree.h>

LOG_MODULE_REGISTER(button_svc);

#define BUT_PORT    DT_ALIAS_SW0_GPIOS_CONTROLLER
#define BUT_PIN     DT_ALIAS_SW0_GPIOS_PIN

extern struct bt_conn *conn;
extern struct bt_gatt_service_static service_helloworld[];
extern volatile bool notify_enable;

static struct device *button_dev;
static struct gpio_callback gpio_cb;
u16_t but_val;

void button_pressed (struct device *gpiob, struct gpio_callback *cb, u32_t pins)
{
	int err;
	if (conn == NULL)
	{
		LOG_INF("BLE not connected");
		return;
	}
	LOG_INF("Button SW1 pressed");
	if (notify_enable == false)
	{
		LOG_INF("Notify not enabled");
		return;
	}
	err = bt_gatt_notify(NULL, &service_helloworld->attrs[2], &but_val, sizeof(but_val));
	if (err)
	{
		LOG_ERR("Notify error: %d", err);
		return;
	}
	LOG_INF("Send notify ok");
	but_val = (but_val == 0) ? 0x100 : 0;
}

int button_init(void)
{
	int ret;
	button_dev = device_get_binding(BUT_PORT);
	if (!button_dev)
	{
		return (-EOPNOTSUPP);
	}

	ret = gpio_pin_configure(button_dev, BUT_PIN, DT_ALIAS_SW0_GPIOS_FLAGS | GPIO_INPUT);
	if (ret != 0)
	{
		LOG_ERR("Error %d: failed to configure pin %d '%s'\n",ret, BUT_PIN, DT_ALIAS_SW0_LABEL);
		return ret;
	}

	gpio_init_callback(&gpio_cb, button_pressed,BIT(BUT_PIN));
	gpio_add_callback(button_dev, &gpio_cb);
	ret = gpio_pin_interrupt_configure(button_dev, BUT_PIN,GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0)
	{
		LOG_ERR("Error %d: failed to configure interrupt on pin %d '%s'\n", ret, BUT_PIN, DT_ALIAS_SW0_LABEL);
		return ret;
	}
	but_val = 0;
	return 0;
}
