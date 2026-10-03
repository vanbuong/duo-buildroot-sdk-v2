#include <string.h>
#include "fake_hw.h"
#include "gpio.h"
#include "poll_i2c.h"

#define MAX_PINS 0x1000
static uint8_t g_pin_level[MAX_PINS];

void fake_gpio_reset(void) { memset(g_pin_level, 0, sizeof(g_pin_level)); }
void fake_gpio_set(int pin, int level) { g_pin_level[pin & 0xfff] = !!level; }
int fake_gpio_get(int pin) { return g_pin_level[pin & 0xfff]; }

int gpio_is_valid(int pin) { (void)pin; return 1; }
void gpio_direction_output(int pin, int val) { fake_gpio_set(pin, val); }
void gpio_direction_input(int pin) { (void)pin; }
void gpio_set_value(int pin, int val) { fake_gpio_set(pin, val); }
int gpio_get_value(int pin) { return fake_gpio_get(pin); }

static uint8_t g_regs[256];
static int g_fail_reads;
static int g_writes;
static void (*g_read_hook)(void);
static uint8_t g_ignore[256];
void fake_i2c_ignore_writes(uint8_t reg, int ignore) { g_ignore[reg] = (uint8_t)!!ignore; }
void fake_i2c_set_read_hook(void (*hook)(void)) { g_read_hook = hook; }

void fake_i2c_reset(void)
{
	memset(g_regs, 0, sizeof(g_regs));
	g_fail_reads = 0;
	g_writes = 0;
	g_read_hook = 0;
	memset(g_ignore, 0, sizeof(g_ignore));
}
void fake_i2c_set_reg(uint8_t reg, uint8_t val) { g_regs[reg] = val; }
uint8_t fake_i2c_get_reg(uint8_t reg) { return g_regs[reg]; }
void fake_i2c_fail_reads(int fail) { g_fail_reads = fail; }
int fake_i2c_write_count(void) { return g_writes; }

static void put16(int reg, int16_t v)
{
	g_regs[reg] = (uint8_t)((uint16_t)v >> 8);
	g_regs[reg + 1] = (uint8_t)((uint16_t)v & 0xff);
}

void fake_i2c_set_accel_gyro(int16_t ax, int16_t ay, int16_t az,
			     int16_t gx, int16_t gy, int16_t gz)
{
	put16(0x3B, ax); put16(0x3D, ay); put16(0x3F, az);
	put16(0x43, gx); put16(0x45, gy); put16(0x47, gz);
}

int poll_i2c_init(uint8_t bus_id) { (void)bus_id; return 0; }

int poll_i2c_write(uint8_t bus_id, uint8_t addr, uint8_t reg,
		   const uint8_t *data, uint16_t len)
{
	uint16_t i;

	(void)bus_id; (void)addr;
	for (i = 0; i < len; i++)
		if (!g_ignore[(uint8_t)(reg + i)])
			g_regs[(uint8_t)(reg + i)] = data[i];
	g_writes++;
	return 0;
}

int poll_i2c_read(uint8_t bus_id, uint8_t addr, uint8_t reg,
		  uint8_t *data, uint16_t len)
{
	uint16_t i;

	(void)bus_id; (void)addr;
	if (g_read_hook)
		g_read_hook();
	if (g_fail_reads)
		return -1;
	for (i = 0; i < len; i++)
		data[i] = g_regs[(uint8_t)(reg + i)];
	return 0;
}
