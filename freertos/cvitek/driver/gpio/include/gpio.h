#ifndef __GPIO_H__
#define __GPIO_H__

#include <stddef.h>
#include <stdint.h>

#define CVI_GPIOA_BASE		0x03020000
#define CVI_GPIOB_BASE		0x03021000
#define CVI_GPIOC_BASE		0x03022000
#define CVI_GPIOD_BASE		0x03023000

/* DesignWare APB GPIO register offsets (single port per bank). */
#define GPIO_SWPORTA_DR		0x00
#define GPIO_SWPORTA_DDR	0x04
#define GPIO_EXT_PORTA		0x50

/*
 * Pin encoding used by FreeRTOS cvitek GPIO helpers:
 *   high byte = bank (0xA / 0xB / 0xC / 0xD)
 *   low byte  = bit within bank
 * Example: GPIOA2 = 0xA02, GPIOB21 = 0xB15
 */
#define GPIO_PIN(bank, bit)	((((uint32_t)(bank) & 0xF) << 8) | ((bit) & 0xFF))
#define GPIOA_PIN(bit)		GPIO_PIN(0xA, bit)
#define GPIOB_PIN(bit)		GPIO_PIN(0xB, bit)
#define GPIOC_PIN(bit)		GPIO_PIN(0xC, bit)
#define GPIOD_PIN(bit)		GPIO_PIN(0xD, bit)

enum of_gpio_flags {
	OF_GPIO_ACTIVE_LOW  = 0x1
};

int gpio_is_valid(int pin);
void gpio_direction_output(int pin, int val);
void gpio_direction_input(int pin);
void gpio_set_value(int pin, int val);
int gpio_get_value(int pin);

#endif
