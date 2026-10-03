#include "encoder.h"
#include "gpio.h"

/* 4x quadrature decode LUT indexed by (prev<<2)|curr, values -1/0/+1. */
static const int8_t qdec_lut[16] = {
	 0, +1, -1,  0,
	-1,  0,  0, +1,
	+1,  0,  0, -1,
	 0, -1, +1,  0
};

void encoder_init(encoder_t *enc, int pin_a, int pin_b)
{
	enc->pin_a = pin_a;
	enc->pin_b = pin_b;
	enc->count = 0;
	gpio_direction_input(pin_a);
	gpio_direction_input(pin_b);
	enc->prev = (uint8_t)((gpio_get_value(pin_a) << 1) | gpio_get_value(pin_b));
}

void encoder_poll(encoder_t *enc)
{
	uint8_t curr = (uint8_t)((gpio_get_value(enc->pin_a) << 1) |
				 gpio_get_value(enc->pin_b));
	uint8_t idx = (uint8_t)((enc->prev << 2) | curr);

	enc->count += qdec_lut[idx];
	enc->prev = curr;
}

int32_t encoder_get_count(encoder_t *enc)
{
	return enc->count;
}

void encoder_reset(encoder_t *enc)
{
	enc->count = 0;
}
