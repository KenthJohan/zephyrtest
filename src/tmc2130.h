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

//TMC2130 pin STEP ---yellow--- (PA0,A3,15)
#define TMC2130_STEP_DEV "PWM_2"
#define TMC2130_STEP_PIN 0
#define TMC2130_STEP_PORT "GPIOA"

//#define PERIOD (USEC_PER_SEC / 50U)
#define PERIOD 100
#define PULSEWIDTH 50

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

#define TMC2130_DRVSTATUS_STST       (1U << 31)
#define TMC2130_DRVSTATUS_OLB        (1U << 30)
#define TMC2130_DRVSTATUS_OLA        (1U << 29)
#define TMC2130_DRVSTATUS_S2GB       (1U << 28)
#define TMC2130_DRVSTATUS_S2GA       (1U << 27)
#define TMC2130_DRVSTATUS_OTPW       (1U << 26)
#define TMC2130_DRVSTATUS_OT         (1U << 25)
#define TMC2130_DRVSTATUS_STALLGUARD (1U << 24)
#define TMC2130_DRVSTATUS_FSACTIVE   (1U << 15)
#define TMC2130_DRVSTATUS_SGRESULT_BITPOS 0
#define TMC2130_DRVSTATUS_SGRESULT_MASK 0x3FFUL
#define TMC2130_DRVSTATUS_CSACTUAL_BITPOS 16
#define TMC2130_DRVSTATUS_CSACTUAL_MASK 0x3FFUL

#define TMC2130_COOLCONF_SFILT (1U << 24)
#define TMC2130_COOLCONF_SGT_BITPOS 16
#define TMC2130_COOLCONF_SGT_MASK 0x7F0000UL





#define TMC2130_EN_FLAG 0x01
#define TMC2130_DIR_FLAG 0x02

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
	u8_t flags;
};

struct device * tmc2130_get_dev (char const * name);



void tmc2130_info_status (u8_t s);
void tmc2130_info_DRVSTATUS (u32_t s);

u8_t tmc2130_write (struct tmc2130 * dev, u8_t cmd, u32_t data);

u8_t tmc2130_read (struct tmc2130 * dev, u8_t cmd, u32_t * data);

void tmc2130_init (struct tmc2130 * dev);

void tmc2130_set_period (struct tmc2130 * dev, u32_t period);
void tmc2130_set_pulse (struct tmc2130 * dev, u32_t pulse);
void tmc2130_set_en (struct tmc2130 * dev, int value);
void tmc2130_set_dir (struct tmc2130 * dev, int value);




