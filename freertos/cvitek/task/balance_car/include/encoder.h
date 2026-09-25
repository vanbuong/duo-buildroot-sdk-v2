#ifndef BALANCE_ENCODER_H
#define BALANCE_ENCODER_H

#include <stdint.h>

typedef struct {
	int pin_a;
	int pin_b;
	volatile int32_t count;
	uint8_t prev;
} encoder_t;

void encoder_init(encoder_t *enc, int pin_a, int pin_b);
void encoder_poll(encoder_t *enc);
int32_t encoder_get_count(encoder_t *enc);
void encoder_reset(encoder_t *enc);

#endif /* BALANCE_ENCODER_H */
