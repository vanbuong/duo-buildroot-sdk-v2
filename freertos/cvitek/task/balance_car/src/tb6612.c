#include "tb6612.h"
#include "board_pins.h"
#include "hw_pwm.h"
#include "gpio.h"
#include "printf.h"

static int clamp_speed(int speed)
{
	if (speed > 100)
		return 100;
	if (speed < -100)
		return -100;
	return speed;
}

int tb6612_init(tb6612_t *drv)
{
	if (!drv)
		return -1;

	drv->pin_ain1 = BC_PIN_AIN1;
	drv->pin_ain2 = BC_PIN_AIN2;
	drv->pin_bin1 = BC_PIN_BIN1;
	drv->pin_bin2 = BC_PIN_BIN2;
	drv->pin_stby = BC_PIN_STBY;
	drv->pwma_chip = BC_PWMA_CHIP;
	drv->pwma_ch = BC_PWMA_CH;
	drv->pwmb_chip = BC_PWMB_CHIP;
	drv->pwmb_ch = BC_PWMB_CH;
	drv->pwm_period_ns = BC_PWM_PERIOD_NS;

	gpio_direction_output(drv->pin_ain1, 0);
	gpio_direction_output(drv->pin_ain2, 0);
	gpio_direction_output(drv->pin_bin1, 0);
	gpio_direction_output(drv->pin_bin2, 0);
	gpio_direction_output(drv->pin_stby, 0);

	hw_pwm_init();
	hw_pwm_set_percent(drv->pwma_chip, drv->pwma_ch, drv->pwm_period_ns, 0);
	hw_pwm_set_percent(drv->pwmb_chip, drv->pwmb_ch, drv->pwm_period_ns, 0);

	tb6612_enable(drv, 0);
	printf("[balance] TB6612 ready (HW PWM %u ns period, STBY off)\n",
	       (unsigned)drv->pwm_period_ns);
	return 0;
}

void tb6612_enable(tb6612_t *drv, int on)
{
	gpio_set_value(drv->pin_stby, on ? 1 : 0);
}

static void set_side(tb6612_t *drv, uint8_t chip, uint8_t ch,
		     int pin_in1, int pin_in2, int speed)
{
	uint8_t duty;

	speed = clamp_speed(speed);
	duty = (uint8_t)(speed < 0 ? -speed : speed);

	if (speed > 0) {
		gpio_set_value(pin_in1, 1);
		gpio_set_value(pin_in2, 0);
	} else if (speed < 0) {
		gpio_set_value(pin_in1, 0);
		gpio_set_value(pin_in2, 1);
	} else {
		gpio_set_value(pin_in1, 0);
		gpio_set_value(pin_in2, 0);
		duty = 0;
	}

	hw_pwm_set_percent(chip, ch, drv->pwm_period_ns, duty);
}

void tb6612_set_speed(tb6612_t *drv, tb6612_side_t side, int speed)
{
	if (side == TB6612_LEFT)
		set_side(drv, drv->pwma_chip, drv->pwma_ch,
			 drv->pin_ain1, drv->pin_ain2, speed);
	else
		set_side(drv, drv->pwmb_chip, drv->pwmb_ch,
			 drv->pin_bin1, drv->pin_bin2, speed);
}

void tb6612_coast(tb6612_t *drv)
{
	tb6612_set_speed(drv, TB6612_LEFT, 0);
	tb6612_set_speed(drv, TB6612_RIGHT, 0);
}

void tb6612_brake(tb6612_t *drv)
{
	gpio_set_value(drv->pin_ain1, 1);
	gpio_set_value(drv->pin_ain2, 1);
	gpio_set_value(drv->pin_bin1, 1);
	gpio_set_value(drv->pin_bin2, 1);
	hw_pwm_set_percent(drv->pwma_chip, drv->pwma_ch, drv->pwm_period_ns, 100);
	hw_pwm_set_percent(drv->pwmb_chip, drv->pwmb_ch, drv->pwm_period_ns, 100);
}
