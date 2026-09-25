#include <stdio.h>
#ifdef RUN_IN_SRAM
#include "system_common.h"
#endif

#include "mmio.h"
#include "gpio.h"

static uint32_t gpio_base_from_pin(int pin)
{
	switch ((pin >> 8) & 0xF) {
	case 0xB:
		return CVI_GPIOB_BASE;
	case 0xC:
		return CVI_GPIOC_BASE;
	case 0xD:
		return CVI_GPIOD_BASE;
	case 0xA:
		return CVI_GPIOA_BASE;
	default:
		return 0;
	}
}

int gpio_is_valid(int pin)
{
	return gpio_base_from_pin(pin) != 0;
}

void gpio_direction_output(int pin, int val)
{
	uint32_t gpio_base = gpio_base_from_pin(pin);
	uint32_t bit;

	if (!gpio_base)
		return;

	bit = 1U << (pin & 0xff);
	mmio_setbits_32(gpio_base + GPIO_SWPORTA_DDR, bit);
	if (val)
		mmio_setbits_32(gpio_base + GPIO_SWPORTA_DR, bit);
	else
		mmio_clrbits_32(gpio_base + GPIO_SWPORTA_DR, bit);
}

void gpio_direction_input(int pin)
{
	uint32_t gpio_base = gpio_base_from_pin(pin);
	uint32_t bit;

	if (!gpio_base)
		return;

	bit = 1U << (pin & 0xff);
	mmio_clrbits_32(gpio_base + GPIO_SWPORTA_DDR, bit);
}

void gpio_set_value(int pin, int val)
{
	uint32_t gpio_base = gpio_base_from_pin(pin);
	uint32_t bit;

	if (!gpio_base)
		return;

	bit = 1U << (pin & 0xff);
	if (val)
		mmio_setbits_32(gpio_base + GPIO_SWPORTA_DR, bit);
	else
		mmio_clrbits_32(gpio_base + GPIO_SWPORTA_DR, bit);
}

int gpio_get_value(int pin)
{
	uint32_t gpio_base = gpio_base_from_pin(pin);
	uint32_t bit;

	if (!gpio_base)
		return 0;

	bit = 1U << (pin & 0xff);
	return (mmio_read_32(gpio_base + GPIO_EXT_PORTA) & bit) ? 1 : 0;
}
