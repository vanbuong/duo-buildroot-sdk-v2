#include "unity_lite.h"
#include "encoder.h"
#include "fake_hw.h"

#define PA 0x10
#define PB 0x11

static void set_ab(int a, int b)
{
	fake_gpio_set(PA, a);
	fake_gpio_set(PB, b);
}

/* One full forward quadrature cycle: 00 -> 01 -> 11 -> 10 -> 00. */
static void cycle_fwd(encoder_t *e)
{
	static const int seq[4][2] = { {0, 1}, {1, 1}, {1, 0}, {0, 0} };
	int i;

	for (i = 0; i < 4; i++) {
		set_ab(seq[i][0], seq[i][1]);
		encoder_poll(e);
	}
}

static void cycle_rev(encoder_t *e)
{
	static const int seq[4][2] = { {1, 0}, {1, 1}, {0, 1}, {0, 0} };
	int i;

	for (i = 0; i < 4; i++) {
		set_ab(seq[i][0], seq[i][1]);
		encoder_poll(e);
	}
}

static void encoder_counts_4x_per_cycle_forward(void)
{
	encoder_t e;

	fake_gpio_reset();
	set_ab(0, 0);
	encoder_init(&e, PA, PB);
	cycle_fwd(&e);
	/* Sign convention: A-leading-B reads positive with this LUT. */
	CHECK_EQ(encoder_get_count(&e), 4);
}

static void encoder_direction_is_signed(void)
{
	encoder_t e;
	int i;

	fake_gpio_reset();
	set_ab(0, 0);
	encoder_init(&e, PA, PB);
	for (i = 0; i < 10; i++)
		cycle_rev(&e);
	CHECK_EQ(encoder_get_count(&e), -40);
	for (i = 0; i < 25; i++)
		cycle_fwd(&e);
	CHECK_EQ(encoder_get_count(&e), -40 + 100);
}

static void encoder_no_motion_no_counts(void)
{
	encoder_t e;
	int i;

	fake_gpio_reset();
	set_ab(1, 0);
	encoder_init(&e, PA, PB);
	for (i = 0; i < 1000; i++)
		encoder_poll(&e);
	CHECK_EQ(encoder_get_count(&e), 0);
}

static void encoder_jitter_is_cancelled(void)
{
	/* Contact bounce on one edge: 00->01->00->01 nets exactly one count. */
	encoder_t e;

	fake_gpio_reset();
	set_ab(0, 0);
	encoder_init(&e, PA, PB);
	set_ab(0, 1); encoder_poll(&e);
	set_ab(0, 0); encoder_poll(&e);
	set_ab(0, 1); encoder_poll(&e);
	CHECK_EQ(encoder_get_count(&e), 1);
}

static void encoder_skipped_state_is_dropped(void)
{
	/* Polling too slowly: 00 -> 11 is an illegal transition, counts 0. */
	encoder_t e;

	fake_gpio_reset();
	set_ab(0, 0);
	encoder_init(&e, PA, PB);
	set_ab(1, 1); encoder_poll(&e);
	CHECK_EQ(encoder_get_count(&e), 0);
}

static void encoder_reset_zeroes_count(void)
{
	encoder_t e;

	fake_gpio_reset();
	set_ab(0, 0);
	encoder_init(&e, PA, PB);
	cycle_rev(&e);
	encoder_reset(&e);
	CHECK_EQ(encoder_get_count(&e), 0);
}

void suite_encoder(void)
{
	printf("suite encoder\n");
	RUN(encoder_counts_4x_per_cycle_forward);
	RUN(encoder_direction_is_signed);
	RUN(encoder_no_motion_no_counts);
	RUN(encoder_jitter_is_cancelled);
	RUN(encoder_skipped_state_is_dropped);
	RUN(encoder_reset_zeroes_count);
}
