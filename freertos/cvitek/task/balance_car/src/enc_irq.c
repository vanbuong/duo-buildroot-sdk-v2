/*
 * GPIO edge interrupts for the wheel encoders (DesignWare APB GPIO, bank B).
 * Register layout is the one used by Linux's gpio-dwapb driver.
 */
#include "enc_irq.h"
#include "board_pins.h"
#include "gpio.h"
#include "mmio.h"
#include "intr_conf.h"
#include "printf.h"

#define GPIO_INTEN		0x30
#define GPIO_INTMASK		0x34
#define GPIO_INTTYPE_LEVEL	0x38	/* 1 = edge   */
#define GPIO_INT_POLARITY	0x3c
#define GPIO_INTSTATUS		0x40
#define GPIO_PORTA_EOI		0x4c
#define GPIO_INT_BOTHEDGE	0x68

#define ENC_MASK  ((1U << (BC_PIN_ENC_L_A & 0xff)) | (1U << (BC_PIN_ENC_L_B & 0xff)) | \
		   (1U << (BC_PIN_ENC_R_A & 0xff)) | (1U << (BC_PIN_ENC_R_B & 0xff)))

static encoder_t *g_l, *g_r;
static volatile unsigned g_isr_count;

static int enc_isr(int irqn, void *priv)
{
	uint32_t st;

	(void)irqn;
	(void)priv;
	st = mmio_read_32(CVI_GPIOB_BASE + GPIO_INTSTATUS);
	mmio_write_32(CVI_GPIOB_BASE + GPIO_PORTA_EOI, st & ENC_MASK);
	if (st & ENC_MASK) {
		encoder_poll(g_l);
		encoder_poll(g_r);
		g_isr_count++;
	}
	return 0;
}

unsigned enc_irq_count(void)
{
	return g_isr_count;
}

int enc_irq_start(encoder_t *left, encoder_t *right)
{
	uint32_t base = CVI_GPIOB_BASE;

	g_l = left;
	g_r = right;
	/* edge sensitive, both edges, unmasked, only our four pins */
	mmio_clrbits_32(base + GPIO_INTEN, ENC_MASK);
	mmio_setbits_32(base + GPIO_INTTYPE_LEVEL, ENC_MASK);
	mmio_setbits_32(base + GPIO_INT_BOTHEDGE, ENC_MASK);
	mmio_clrbits_32(base + GPIO_INTMASK, ENC_MASK);
	mmio_write_32(base + GPIO_PORTA_EOI, ENC_MASK);
	mmio_setbits_32(base + GPIO_INTEN, ENC_MASK);
	if (request_irq(BC_ENC_IRQ_NUM, enc_isr, 0, "bc_enc", NULL)) {
		printf("[balance] request_irq(%d) failed\n", BC_ENC_IRQ_NUM);
		return -1;
	}
	printf("[balance] encoder edge IRQ %d armed (mask 0x%x)\n", BC_ENC_IRQ_NUM,
	       (unsigned)ENC_MASK);
	return 0;
}
