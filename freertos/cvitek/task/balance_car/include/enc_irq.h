#ifndef BALANCE_ENC_IRQ_H
#define BALANCE_ENC_IRQ_H

#include "encoder.h"

/*
 * Decode both wheel encoders from GPIOB both-edge interrupts. The ISR re-reads
 * all four lines and applies the quadrature LUT, so no edge is lost while the
 * control task is busy in the I2C transfer. Returns 0 on success.
 */
int enc_irq_start(encoder_t *left, encoder_t *right);
/* Number of ISR invocations (diagnostics). */
unsigned enc_irq_count(void);

#endif
