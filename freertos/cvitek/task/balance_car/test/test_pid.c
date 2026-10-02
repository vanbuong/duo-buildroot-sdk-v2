#include "unity_lite.h"
#include "pid.h"

static void pid_p_only_is_proportional(void)
{
	bc_pid_t p;

	pid_init(&p, 2.0f, 0.0f, 0.0f, -100.0f, 100.0f);
	CHECK_NEAR(pid_update(&p, 10.0f, 4.0f, 0.005f), 12.0f, 1e-4);
	CHECK_NEAR(pid_update(&p, 0.0f, 4.0f, 0.005f), -8.0f, 1e-4);
}

static void pid_output_is_clamped(void)
{
	bc_pid_t p;

	pid_init(&p, 100.0f, 0.0f, 0.0f, -50.0f, 60.0f);
	CHECK_NEAR(pid_update(&p, 10.0f, 0.0f, 0.005f), 60.0f, 1e-4);
	CHECK_NEAR(pid_update(&p, -10.0f, 0.0f, 0.005f), -50.0f, 1e-4);
}

static void pid_integral_accumulates_and_is_clamped(void)
{
	bc_pid_t p;
	int i;
	float out = 0;

	pid_init(&p, 0.0f, 1.0f, 0.0f, -10.0f, 10.0f);
	/* err=1 for 1 s at 200 Hz -> integral ~ 1.0 -> out ~ 1.0 */
	for (i = 0; i < 200; i++)
		out = pid_update(&p, 1.0f, 0.0f, 0.005f);
	CHECK_NEAR(out, 1.0f, 0.02);
	/* Keep pushing: integral must saturate at i_max, not wind up forever. */
	for (i = 0; i < 100000; i++)
		out = pid_update(&p, 1.0f, 0.0f, 0.005f);
	CHECK_NEAR(p.integral, 10.0f, 1e-3);
	CHECK_NEAR(out, 10.0f, 1e-3);
	/* Anti-windup: reversing the error unwinds within a bounded time. */
	for (i = 0; i < 200 * 11; i++)
		out = pid_update(&p, -1.0f, 0.0f, 0.005f);
	CHECK(out < 0.0f);
}

static void pid_derivative_acts_on_error_change(void)
{
	bc_pid_t p;

	pid_init(&p, 0.0f, 0.0f, 1.0f, -1000.0f, 1000.0f);
	pid_update(&p, 0.0f, 0.0f, 0.01f);
	/* error jumps 0 -> 1 in 10 ms -> d = 100 */
	CHECK_NEAR(pid_update(&p, 1.0f, 0.0f, 0.01f), 100.0f, 1e-3);
	/* steady error -> derivative returns to 0 */
	CHECK_NEAR(pid_update(&p, 1.0f, 0.0f, 0.01f), 0.0f, 1e-3);
}

/*
 * Characterisation of a known limitation (docs/balance_car/04): derivative is
 * taken on error, so a setpoint step produces a derivative kick. When the
 * firmware switches to derivative-on-measurement this test must be updated.
 */
static void pid_setpoint_step_causes_derivative_kick(void)
{
	bc_pid_t p;
	float out;

	pid_init(&p, 0.0f, 0.0f, 1.0f, -1000.0f, 1000.0f);
	pid_update(&p, 0.0f, 0.0f, 0.005f);
	out = pid_update(&p, 5.0f, 0.0f, 0.005f);
	CHECK(out > 900.0f);
}

static void pid_reset_clears_state(void)
{
	bc_pid_t p;

	pid_init(&p, 1.0f, 1.0f, 1.0f, -100.0f, 100.0f);
	pid_update(&p, 5.0f, 0.0f, 0.01f);
	pid_reset(&p);
	CHECK_NEAR(p.integral, 0.0f, 1e-9);
	CHECK_NEAR(p.prev_err, 0.0f, 1e-9);
}

static void pid_nonpositive_dt_is_safe(void)
{
	bc_pid_t p;
	float out;

	pid_init(&p, 1.0f, 1.0f, 1.0f, -100.0f, 100.0f);
	out = pid_update(&p, 1.0f, 0.0f, 0.0f);
	CHECK(out == out);		/* not NaN */
	CHECK(fabsf(out) <= 100.0f);
	out = pid_update(&p, 1.0f, 0.0f, -1.0f);
	CHECK(out == out);
}

void suite_pid(void)
{
	printf("suite pid\n");
	RUN(pid_p_only_is_proportional);
	RUN(pid_output_is_clamped);
	RUN(pid_integral_accumulates_and_is_clamped);
	RUN(pid_derivative_acts_on_error_change);
	RUN(pid_setpoint_step_causes_derivative_kick);
	RUN(pid_reset_clears_state);
	RUN(pid_nonpositive_dt_is_safe);
}
