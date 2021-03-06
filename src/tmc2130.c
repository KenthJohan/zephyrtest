#include <sys/__assert.h>
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


void tmc2130_info_DRVSTATUS (u32_t s)
{
	printf ("%sstst|", (s & TMC2130_DRVSTATUS_STST)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%solb|", (s & TMC2130_DRVSTATUS_OLB)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%sola|", (s & TMC2130_DRVSTATUS_OLA)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%ss2gb|", (s & TMC2130_DRVSTATUS_S2GB)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%ss2ga|", (s & TMC2130_DRVSTATUS_S2GA)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%sotpw|", (s & TMC2130_DRVSTATUS_OTPW)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%sStallGuard|", (s & TMC2130_DRVSTATUS_STALLGUARD)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf ("%sfsactive|", (s & TMC2130_DRVSTATUS_FSACTIVE)?ANSI_COLOR_GREEN:ANSI_COLOR_RED);
	printf (ANSI_COLOR_RESET);
	printf ("CSACTUAL%lu|", (s & TMC2130_DRVSTATUS_CSACTUAL_MASK) >> TMC2130_DRVSTATUS_CSACTUAL_BITPOS);
	printf ("SGRESULT%lu", (s & TMC2130_DRVSTATUS_SGRESULT_MASK) >> TMC2130_DRVSTATUS_SGRESULT_BITPOS);
	printf ("\n");
}


u8_t tmc2130_tansfer (struct tmc2130 * dev, u8_t data_tx)
{
	u8_t data_rx;
	struct spi_buf buf_tx[] = {{.buf = &data_tx,.len = sizeof(data_tx)}};
	struct spi_buf buf_rx[] = {{.buf = &data_rx,.len = sizeof(data_rx)}};
	struct spi_buf_set tx = {.buffers = buf_tx, .count = 1};
	struct spi_buf_set rx = {.buffers = buf_rx, .count = 1};
	spi_transceive (dev->dev_spi, &dev->spi_cfg, &tx, &rx);
	return data_rx;
}


u8_t tmc2130_write (struct tmc2130 * dev, u8_t cmd, u32_t data)
{
	//LOG_INF ("tmc2130_write 0x%02x 0x%08x", cmd, data);
	uint8_t s;
	gpio_pin_set (dev->dev_gpio_cs, TMC2130_CS_PIN, 0);
	s = tmc2130_tansfer (dev, cmd);
	tmc2130_tansfer (dev, (data >> 24UL) & 0xFF) & 0xFF;
	tmc2130_tansfer (dev, (data >> 16UL) & 0xFF) & 0xFF;
	tmc2130_tansfer (dev, (data >>  8UL) & 0xFF) & 0xFF;
	tmc2130_tansfer (dev, (data >>  0UL) & 0xFF) & 0xFF;
	gpio_pin_set (dev->dev_gpio_cs, TMC2130_CS_PIN, 1);
	return s;
}


u8_t tmc2130_read (struct tmc2130 * dev, u8_t cmd, u32_t * data)
{
	uint8_t s;
	tmc2130_write (dev, cmd, 0UL); //set read address
	gpio_pin_set (dev->dev_gpio_cs, TMC2130_CS_PIN, 0);
	s = tmc2130_tansfer (dev, cmd);
	*data  = tmc2130_tansfer (dev, 0x00) & 0xFF;
	*data <<= 8;
	*data |= tmc2130_tansfer (dev, 0x00) & 0xFF;
	*data <<= 8;
	*data |= tmc2130_tansfer (dev, 0x00) & 0xFF;
	*data <<= 8;
	*data |= tmc2130_tansfer (dev, 0x00) & 0xFF;
	gpio_pin_set (dev->dev_gpio_cs, TMC2130_CS_PIN, 1);
	return s;
}



/*
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
*/

void tmc2130_set_period (struct tmc2130 * dev, u32_t period)
{
	int r;
	r = pwm_pin_set_usec (dev->dev_pwm_step, 1, period, dev->pulse, 0);
	__ASSERT (r == 0, "pwm_pin_set_usec period (%u), pulse (%u) failed", period, dev->pulse);
	dev->period = period;
}


void tmc2130_set_pulse (struct tmc2130 * dev, u32_t pulse)
{
	int r;
	r = pwm_pin_set_usec (dev->dev_pwm_step, 1, dev->period, pulse, 0);
	__ASSERT (r == 0, "pwm_pin_set_usec pulse (%u), period (%u) failed", pulse, dev->period);
	dev->pulse = pulse;
}


void tmc2130_set_dir (struct tmc2130 * dev, int value)
{
	int r;
	if (value)
	{
		r = gpio_pin_set (dev->dev_gpio_dir, TMC2130_DIR_PIN, 1);
		__ASSERT (r == 0, "gpio_pin_set failed (err %u)", r);
		dev->flags |= TMC2130_DIR_FLAG;
	}
	else
	{
		r = gpio_pin_set (dev->dev_gpio_dir, TMC2130_DIR_PIN, 0);
		__ASSERT (r == 0, "gpio_pin_set failed (err %u)", r);
		dev->flags &= ~TMC2130_DIR_FLAG;
	}
}


void tmc2130_set_en (struct tmc2130 * dev, int value)
{
	int r;
	if (value)
	{
		r = gpio_pin_set (dev->dev_gpio_en, TMC2130_EN_PIN, 1);
		__ASSERT (r == 0, "gpio_pin_set failed (err %u)", r);
		dev->flags |= TMC2130_EN_FLAG;
	}
	else
	{
		r = gpio_pin_set (dev->dev_gpio_en, TMC2130_EN_PIN, 0);
		__ASSERT (r == 0, "gpio_pin_set failed (err %u)", r);
		dev->flags &= ~TMC2130_EN_FLAG;

	}
}


void tmc2130_init (struct tmc2130 * dev)
{
	int r;
	dev->period = PERIOD;
	dev->pulse = PULSEWIDTH;
	dev->dev_pwm_step = device_get_binding (TMC2130_STEP_DEV);
	__ASSERT (dev->dev_pwm_step, "device_get_binding failed");
	dev->dev_gpio_dir = device_get_binding (TMC2130_DIR_PORT);
	__ASSERT (dev->dev_gpio_dir, "device_get_binding failed");
	dev->dev_gpio_en = device_get_binding (TMC2130_EN_PORT);
	__ASSERT (dev->dev_gpio_en, "device_get_binding failed");
	dev->dev_gpio_cs = device_get_binding (TMC2130_CS_PORT);
	__ASSERT (dev->dev_gpio_cs, "device_get_binding failed");
	dev->dev_spi = device_get_binding (TMC2130_SPI_DEV);
	__ASSERT (dev->dev_spi, "device_get_binding failed");
	r = pwm_pin_set_usec (dev->dev_pwm_step, 1, dev->period, dev->pulse, 0);
	__ASSERT (r == 0, "pwm_pin_set_usec failed (err %u)", r);
	r = gpio_pin_configure (dev->dev_gpio_dir, TMC2130_DIR_PIN, GPIO_OUTPUT_INACTIVE);
	__ASSERT (r == 0, "gpio_pin_configure failed (err %u)", r);
	r = gpio_pin_configure (dev->dev_gpio_en, TMC2130_EN_PIN, GPIO_OUTPUT_ACTIVE);
	__ASSERT (r == 0, "gpio_pin_configure failed (err %u)", r);
	r = gpio_pin_configure (dev->dev_gpio_cs, TMC2130_CS_PIN, GPIO_OUTPUT_ACTIVE);
	__ASSERT (r == 0, "gpio_pin_configure failed (err %u)", r);
	dev->spi_cfg.operation = SPI_WORD_SET(8) | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_TRANSFER_MSB;
	dev->spi_cfg.frequency = 1000000;
	//uint32_t data = 0;
	//tmc_read (dev, REG_DRVSTATUS, &data);
	//tmc_read (dev, REG_GSTAT, &data);
}








