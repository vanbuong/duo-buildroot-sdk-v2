#ifndef BALANCE_TB6612_H
#define BALANCE_TB6612_H

#include <stdint.h>

typedef enum {
	TB6612_LEFT = 0,
	TB6612_RIGHT = 1,
} tb6612_side_t;

typedef struct {
	int pin_ain1;
	int pin_ain2;
	int pin_bin1;
	int pin_bin2;
	int pin_stby;
	uint8_t pwma_chip;
	uint8_t pwma_ch;
	uint8_t pwmb_chip;
	uint8_t pwmb_ch;
	uint32_t pwm_period_ns;
} tb6612_t;

int tb6612_init(tb6612_t *drv);
void tb6612_enable(tb6612_t *drv, int on);
/* speed: -100 .. +100 (percent of full PWM). */
void tb6612_set_speed(tb6612_t *drv, tb6612_side_t side, int speed);
void tb6612_coast(tb6612_t *drv);
void tb6612_brake(tb6612_t *drv);

#endif /* BALANCE_TB6612_H */
