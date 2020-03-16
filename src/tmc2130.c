#include "tmc2130.h"

LOG_MODULE_REGISTER(tmc2130);

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


void tmc2130_info_status (u8_t s)
{
	printf ("%sreset|", (s & TMC2130_RESET_FLAG)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%serror|", (s & TMC2130_DRIVER_ERROR)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%ssg2|", (s & TMC2130_SG2)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%sstandstill", (s & TMC2130_STANDSTILL)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf (ANSI_COLOR_RESET"\n");
}


void tmc2130_info_drv_status (u32_t s)
{
	printf ("%s TMC2130_DRV_STATUS_STST|", (s & TMC2130_DRV_STATUS_STST)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%s TMC2130_DRV_STATUS_OLB|", (s & TMC2130_DRV_STATUS_OLB)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%s TMC2130_DRV_STATUS_OLA|", (s & TMC2130_DRV_STATUS_OLA)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%s TMC2130_DRV_STATUS_S2GB|", (s & TMC2130_DRV_STATUS_S2GB)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%s TMC2130_DRV_STATUS_S2GA|", (s & TMC2130_DRV_STATUS_S2GA)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%s TMC2130_DRV_STATUS_STALLGUARD|", (s & TMC2130_DRV_STATUS_STALLGUARD)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf (ANSI_COLOR_RESET"\n");
}


u8_t tmc2130_write (struct tmc2130 * dev, u8_t cmd, u32_t data)
{
	//TMC2130 MSB big-endian
	u8_t s;
	//data = sys_cpu_to_be32 (data);
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
	tmc2130_info_status (s);
	return s;
}

u8_t tmc2130_read (struct tmc2130 * dev, u8_t cmd, u32_t * data)
{
	tmc2130_write (dev, cmd, 0);
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
	tmc2130_info_status (s);
	return s;
}


void tmc2130_set_period (struct tmc2130 * dev, u32_t period)
{
	int r;
	r = pwm_pin_set_usec (dev->dev_pwm_step, 1, period, dev->pulse, 0);
	if (r)
	{
		LOG_ERR ("PWM pin set period (%u) failed", period);
	}
	else
	{
		dev->period = period;
	}
}


void tmc2130_set_pulse (struct tmc2130 * dev, u32_t pulse)
{
	int r;
	r = pwm_pin_set_usec (dev->dev_pwm_step, 1, dev->period, pulse, 0);
	if (r)
	{
		LOG_ERR ("PWM pin set pulse (%u) failed", pulse);
	}
	else
	{
		dev->pulse = pulse;
	}
}


void tmc2130_set_dir (struct tmc2130 * dev, int value)
{
	int r;
	if (value)
	{
		r = gpio_pin_set (dev->dev_gpio_dir, TMC2130_DIR_PIN, 1);
		if (r)
		{
			LOG_ERR ("DIR pin set fails");
		}
		else
		{
			dev->flags |= TMC2130_DIR_FLAG;
		}
	}
	else
	{
		r = gpio_pin_set (dev->dev_gpio_dir, TMC2130_DIR_PIN, 0);
		if (r)
		{
			LOG_ERR ("DIR pin set fails");
		}
		else
		{
			dev->flags &= ~TMC2130_DIR_FLAG;
		}
	}
}


void tmc2130_set_en (struct tmc2130 * dev, int value)
{
	int r;
	if (value)
	{
		r = gpio_pin_set (dev->dev_gpio_en, TMC2130_EN_PIN, 1);
		if (r)
		{
			LOG_ERR ("EN pin set fails");
		}
		else
		{
			dev->flags |= TMC2130_EN_FLAG;
		}
	}
	else
	{
		r = gpio_pin_set (dev->dev_gpio_en, TMC2130_EN_PIN, 0);
		if (r)
		{
			LOG_ERR ("EN pin set fails");
		}
		else
		{
			dev->flags &= ~TMC2130_EN_FLAG;
		}
	}
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








