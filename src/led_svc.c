/** @file
 *  @brief Button Service sample
 */

/*
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
#include <zephyr.h>
#include <drivers/gpio.h>
#include <logging/log.h>

#include <devicetree.h>
#include "led_svc.h"

LOG_MODULE_REGISTER(led_svc);

#define LED_PORT DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED      DT_ALIAS_LED0_GPIOS_PIN

struct device *led_dev;


ssize_t led_recv (struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, u16_t len, u16_t offset, u8_t flags)
{
	if (led_dev == NULL)
	{
		LOG_ERR ("Error: no LED device");
	}
	gpio_pin_toggle (led_dev, LED);
	return 0;
}

int led_init(void)
{
	int ret;
	led_dev = device_get_binding (LED_PORT);
	if (led_dev == NULL)
	{
		LOG_ERR ("Error device_get_binding: failed to get device '%s'\n", LED_PORT);
		return (-EOPNOTSUPP);
	}
	ret = gpio_pin_configure (led_dev, LED,GPIO_OUTPUT_ACTIVE | DT_ALIAS_LED0_GPIOS_FLAGS);
	if (ret < 0)
	{
		LOG_ERR ("Error %d: failed to configure pin %d '%s'\n", ret, LED, DT_ALIAS_LED0_LABEL);
		return ret;
	}
	return 0;
}
