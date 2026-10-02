#include <math.h>
#include "unity_lite.h"
#include "bc_state.h"

static bc_params_t P;

static bc_state_in_t in0(void)
{
	bc_state_in_t i = { 0 };

	i.dt_ms = 5.0f;
	return i;
}

/* drive the machine for ms milliseconds with fixed inputs */
static bc_state_id_t run(bc_state_t *s, bc_state_in_t i, float ms)
{
	int n = (int)(ms / i.dt_ms), k;
	uint32_t ev = i.events;

	for (k = 0; k < n; k++) {
		i.events = (k == 0) ? ev : 0;
		bc_state_step(s, &i, &P);
	}
	return s->state;
}

static void to_idle(bc_state_t *s)
{
	bc_state_in_t i = in0();

	bc_state_init(s);
	i.events = BC_EV_CALIB_REQ; bc_state_step(s, &i, &P);
	i.events = BC_EV_CALIB_OK;  bc_state_step(s, &i, &P);
}

static void to_balancing(bc_state_t *s)
{
	bc_state_in_t i = in0();

	to_idle(s);
	i.events = BC_EV_ARM;
	run(s, i, 1100.0f);
}

static void st_boot_calibrate_idle(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	bc_state_init(&s);
	CHECK_EQ(s.state, BC_ST_BOOT);
	i.events = BC_EV_CALIB_REQ; bc_state_step(&s, &i, &P);
	CHECK_EQ(s.state, BC_ST_CALIBRATING);
	i.events = BC_EV_CALIB_OK; bc_state_step(&s, &i, &P);
	CHECK_EQ(s.state, BC_ST_IDLE);
}

static void st_calibration_failure_is_a_fault(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	bc_state_init(&s);
	i.events = BC_EV_CALIB_REQ; bc_state_step(&s, &i, &P);
	i.events = BC_EV_CALIB_FAIL; bc_state_step(&s, &i, &P);
	CHECK_EQ(s.state, BC_ST_FAULT);
	CHECK_EQ(s.fault, BC_FAULT_CALIB);
}

static void st_imu_init_failure_in_boot(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	bc_state_init(&s);
	i.events = BC_EV_IMU_INIT_FAIL; bc_state_step(&s, &i, &P);
	CHECK_EQ(s.state, BC_ST_FAULT);
	CHECK_EQ(s.fault, BC_FAULT_IMU_INIT);
}

static void st_no_balancing_without_arm(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_idle(&s);
	CHECK_EQ(run(&s, i, 5000.0f), BC_ST_IDLE);	/* upright but never armed */
}

static void st_arming_needs_one_second_upright(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_idle(&s);
	i.events = BC_EV_ARM;
	CHECK_EQ(run(&s, i, 900.0f), BC_ST_ARMING);	/* ST-02 */
	i.events = 0;
	CHECK_EQ(run(&s, i, 200.0f), BC_ST_BALANCING);
}

static void st_arming_window_restarts_when_disturbed(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_idle(&s);
	i.events = BC_EV_ARM;
	run(&s, i, 800.0f);
	i.events = 0; i.omega = 50.0f;			/* moved */
	run(&s, i, 5.0f);
	i.omega = 0.0f;
	CHECK_EQ(run(&s, i, 800.0f), BC_ST_ARMING);	/* restarted: not yet */
	CHECK_EQ(run(&s, i, 400.0f), BC_ST_BALANCING);
}

static void st_arming_times_out_when_not_upright(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_idle(&s);
	i.events = BC_EV_ARM; i.theta = 20.0f;
	CHECK_EQ(run(&s, i, 4000.0f), BC_ST_ARMING);
	i.events = 0;
	CHECK_EQ(run(&s, i, 1500.0f), BC_ST_IDLE);
}

static void st_arm_refused_with_imu_fault(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_idle(&s);
	i.events = BC_EV_ARM; i.imu_bus_fail = 1;
	CHECK_EQ(run(&s, i, 100.0f), BC_ST_IDLE);
}

static void st_fall_detection(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_balancing(&s);
	CHECK_EQ(s.state, BC_ST_BALANCING);
	i.theta = 46.0f;
	CHECK_EQ(run(&s, i, 5.0f), BC_ST_FALLEN);	/* SAF-01 */
	CHECK_EQ(s.fault, BC_FAULT_FALL);
}

static void st_fallen_recovers_only_when_upright_and_still(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_balancing(&s);
	i.theta = 60.0f; run(&s, i, 5.0f);
	CHECK_EQ(run(&s, i, 5000.0f), BC_ST_FALLEN);	/* still lying down */
	i.theta = 0.0f;
	CHECK_EQ(run(&s, i, 1900.0f), BC_ST_FALLEN);
	CHECK_EQ(run(&s, i, 200.0f), BC_ST_IDLE);
}

static void st_imu_faults_stop_balancing(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();
	int k;
	struct { int bus, stuck, range; bc_fault_t f; } cases[3] = {
		{ 1, 0, 0, BC_FAULT_IMU_BUS },
		{ 0, 1, 0, BC_FAULT_IMU_STUCK },
		{ 0, 0, 1, BC_FAULT_IMU_RANGE },
	};

	bc_params_default(&P);
	for (k = 0; k < 3; k++) {
		to_balancing(&s);
		i.imu_bus_fail = cases[k].bus;
		i.imu_stuck = cases[k].stuck;
		i.imu_range = cases[k].range;
		CHECK_EQ(run(&s, i, 5.0f), BC_ST_FAULT);
		CHECK_EQ(s.fault, cases[k].f);
		i.imu_bus_fail = i.imu_stuck = i.imu_range = 0;
	}
}

static void st_fault_needs_disarm_and_clean_sensor(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_balancing(&s);
	i.imu_bus_fail = 1; run(&s, i, 5.0f);
	CHECK_EQ(s.state, BC_ST_FAULT);
	i.events = BC_EV_DISARM;			/* sensor still bad */
	CHECK_EQ(run(&s, i, 5.0f), BC_ST_FAULT);
	i.imu_bus_fail = 0; i.events = BC_EV_DISARM;
	CHECK_EQ(run(&s, i, 5.0f), BC_ST_IDLE);
	CHECK_EQ(s.fault, BC_FAULT_NONE);
}

static void st_saturation_for_too_long_is_a_fault(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_balancing(&s);
	i.saturated = 1;
	CHECK_EQ(run(&s, i, 250.0f), BC_ST_BALANCING);
	CHECK_EQ(run(&s, i, 100.0f), BC_ST_FAULT);	/* SAF-05, sat_ms = 300 */
	CHECK_EQ(s.fault, BC_FAULT_SATURATION);
}

static void st_short_saturation_is_tolerated(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();
	int k;

	bc_params_default(&P);
	to_balancing(&s);
	for (k = 0; k < 20; k++) {
		i.saturated = 1; run(&s, i, 200.0f);
		i.saturated = 0; run(&s, i, 5.0f);
	}
	CHECK_EQ(s.state, BC_ST_BALANCING);
}

static void st_lift_detection(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_balancing(&s);
	i.v_f = 2.0f;
	CHECK_EQ(run(&s, i, 150.0f), BC_ST_BALANCING);
	CHECK_EQ(run(&s, i, 100.0f), BC_ST_FAULT);	/* SAF-04 */
	CHECK_EQ(s.fault, BC_FAULT_LIFT);
}

static void st_fast_wheels_while_leaning_is_not_lift(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_balancing(&s);
	i.v_f = 2.0f; i.theta = 15.0f;			/* catching a fall */
	CHECK_EQ(run(&s, i, 1000.0f), BC_ST_BALANCING);
}

static void st_timing_fault(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_balancing(&s);
	i.timing_fault = 1;
	CHECK_EQ(run(&s, i, 5.0f), BC_ST_FAULT);
	CHECK_EQ(s.fault, BC_FAULT_TIMING);
}

static void st_estop_from_every_state_and_latches(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_balancing(&s);
	i.events = BC_EV_ESTOP;
	CHECK_EQ(run(&s, i, 5.0f), BC_ST_ESTOP);	/* SAF-07 */
	i.events = BC_EV_ARM;
	CHECK_EQ(run(&s, i, 3000.0f), BC_ST_ESTOP);	/* ARM cannot clear it */
	i.events = BC_EV_DISARM;
	CHECK_EQ(run(&s, i, 5.0f), BC_ST_IDLE);
	bc_state_init(&s);
	i.events = BC_EV_ESTOP;
	CHECK_EQ(run(&s, i, 5.0f), BC_ST_ESTOP);	/* even from BOOT */
}

static void st_disarm_returns_to_idle_from_balancing(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	to_balancing(&s);
	i.events = BC_EV_DISARM;
	CHECK_EQ(run(&s, i, 5.0f), BC_ST_IDLE);
}

static void st_motors_only_in_balancing(void)
{
	bc_state_t s;
	bc_state_in_t i = in0();

	bc_params_default(&P);
	bc_state_init(&s);
	CHECK(!bc_state_is_balancing(&s));
	to_idle(&s);
	CHECK(!bc_state_is_balancing(&s));
	i.events = BC_EV_ARM; run(&s, i, 100.0f);
	CHECK(!bc_state_is_balancing(&s));		/* ARMING */
	i.events = 0; run(&s, i, 1100.0f);
	CHECK(bc_state_is_balancing(&s));
}

static void st_names_exist_for_every_value(void)
{
	int k;

	for (k = 0; k < BC_FAULT_COUNT; k++)
		CHECK(bc_fault_name((bc_fault_t)k)[0] != '?');
	for (k = 0; k <= BC_ST_ESTOP; k++)
		CHECK(bc_state_name((bc_state_id_t)k)[0] != '?');
}

void suite_state(void)
{
	printf("suite state machine\n");
	RUN(st_boot_calibrate_idle);
	RUN(st_calibration_failure_is_a_fault);
	RUN(st_imu_init_failure_in_boot);
	RUN(st_no_balancing_without_arm);
	RUN(st_arming_needs_one_second_upright);
	RUN(st_arming_window_restarts_when_disturbed);
	RUN(st_arming_times_out_when_not_upright);
	RUN(st_arm_refused_with_imu_fault);
	RUN(st_fall_detection);
	RUN(st_fallen_recovers_only_when_upright_and_still);
	RUN(st_imu_faults_stop_balancing);
	RUN(st_fault_needs_disarm_and_clean_sensor);
	RUN(st_saturation_for_too_long_is_a_fault);
	RUN(st_short_saturation_is_tolerated);
	RUN(st_lift_detection);
	RUN(st_fast_wheels_while_leaning_is_not_lift);
	RUN(st_timing_fault);
	RUN(st_estop_from_every_state_and_latches);
	RUN(st_disarm_returns_to_idle_from_balancing);
	RUN(st_motors_only_in_balancing);
	RUN(st_names_exist_for_every_value);
}
