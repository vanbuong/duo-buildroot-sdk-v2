#include <math.h>
#include "bc_state.h"

#define ARM_ANGLE_DEG	3.0f
#define ARM_RATE_DPS	10.0f
#define ARM_HOLD_MS	1000.0f
#define ARM_TIMEOUT_MS	5000.0f
#define FALLEN_HOLD_MS	2000.0f
#define LIFT_ANGLE_DEG	10.0f
#define LIFT_HOLD_MS	200.0f

void bc_state_init(bc_state_t *s)
{
	s->state = BC_ST_BOOT;
	s->fault = BC_FAULT_NONE;
	s->t_in_state_ms = s->still_ms = s->sat_ms = s->lift_ms = 0.0f;
}

static void enter(bc_state_t *s, bc_state_id_t st, bc_fault_t f)
{
	s->state = st;
	s->fault = f;
	s->t_in_state_ms = 0.0f;
	s->still_ms = s->sat_ms = s->lift_ms = 0.0f;
}

static int imu_fault(const bc_state_in_t *in, bc_fault_t *f)
{
	if (in->imu_bus_fail) { *f = BC_FAULT_IMU_BUS; return 1; }
	if (in->imu_stuck)    { *f = BC_FAULT_IMU_STUCK; return 1; }
	if (in->imu_range)    { *f = BC_FAULT_IMU_RANGE; return 1; }
	return 0;
}

bc_state_id_t bc_state_step(bc_state_t *s, const bc_state_in_t *in,
			    const bc_params_t *p)
{
	float dev = fabsf(in->theta - in->trim);
	int upright = dev < ARM_ANGLE_DEG && fabsf(in->omega) < ARM_RATE_DPS;
	bc_fault_t f = BC_FAULT_NONE;
	int imu_bad = imu_fault(in, &f);

	s->t_in_state_ms += in->dt_ms;

	/* highest priority: emergency stop */
	if (in->events & BC_EV_ESTOP) {
		enter(s, BC_ST_ESTOP, BC_FAULT_ESTOP);
		return s->state;
	}

	/* boot / calibration */
	if (in->events & BC_EV_IMU_INIT_FAIL) {
		if (s->state == BC_ST_BOOT || s->state == BC_ST_CALIBRATING)
			enter(s, BC_ST_FAULT, BC_FAULT_IMU_INIT);
		return s->state;
	}
	if (in->events & BC_EV_CALIB_REQ) {
		if (s->state == BC_ST_BOOT || s->state == BC_ST_IDLE)
			enter(s, BC_ST_CALIBRATING, BC_FAULT_NONE);
		return s->state;
	}
	if (s->state == BC_ST_CALIBRATING) {
		if (in->events & BC_EV_CALIB_OK)
			enter(s, BC_ST_IDLE, BC_FAULT_NONE);
		else if (in->events & BC_EV_CALIB_FAIL)
			enter(s, BC_ST_FAULT, BC_FAULT_CALIB);
		return s->state;
	}

	/* explicit disarm */
	if (in->events & BC_EV_DISARM) {
		if (s->state == BC_ST_FAULT && imu_bad)
			return s->state;	/* cannot clear while still faulty */
		if (s->state != BC_ST_BOOT && s->state != BC_ST_CALIBRATING)
			enter(s, BC_ST_IDLE, BC_FAULT_NONE);
		return s->state;
	}

	switch (s->state) {
	case BC_ST_IDLE:
		if ((in->events & BC_EV_ARM) && !imu_bad)
			enter(s, BC_ST_ARMING, BC_FAULT_NONE);
		break;

	case BC_ST_ARMING:
		if (imu_bad) {
			enter(s, BC_ST_FAULT, f);
		} else if (upright) {
			s->still_ms += in->dt_ms;
			if (s->still_ms >= ARM_HOLD_MS)
				enter(s, BC_ST_BALANCING, BC_FAULT_NONE);
		} else {
			s->still_ms = 0.0f;
			if (s->t_in_state_ms >= ARM_TIMEOUT_MS)
				enter(s, BC_ST_IDLE, BC_FAULT_NONE);
		}
		break;

	case BC_ST_BALANCING:
		if (imu_bad) {
			enter(s, BC_ST_FAULT, f);
		} else if (in->timing_fault) {
			enter(s, BC_ST_FAULT, BC_FAULT_TIMING);
		} else if (fabsf(in->theta) > p->fall_deg) {
			enter(s, BC_ST_FALLEN, BC_FAULT_FALL);
		} else {
			/* lift: wheels run away while the body is upright */
			if (fabsf(in->v_f) > p->lift_speed_mps && dev < LIFT_ANGLE_DEG)
				s->lift_ms += in->dt_ms;
			else
				s->lift_ms = 0.0f;
			/* saturation: loop not coping */
			if (in->saturated)
				s->sat_ms += in->dt_ms;
			else
				s->sat_ms = 0.0f;
			if (s->lift_ms >= LIFT_HOLD_MS)
				enter(s, BC_ST_FAULT, BC_FAULT_LIFT);
			else if (s->sat_ms >= p->sat_ms)
				enter(s, BC_ST_FAULT, BC_FAULT_SATURATION);
		}
		break;

	case BC_ST_FALLEN:
		if (upright) {
			s->still_ms += in->dt_ms;
			if (s->still_ms >= FALLEN_HOLD_MS)
				enter(s, BC_ST_IDLE, BC_FAULT_NONE);
		} else {
			s->still_ms = 0.0f;
		}
		break;

	case BC_ST_BOOT:
	case BC_ST_FAULT:
	case BC_ST_ESTOP:
	default:
		break;
	}
	return s->state;
}

const char *bc_state_name(bc_state_id_t s)
{
	static const char *const n[] = { "BOOT", "CALIBRATING", "IDLE", "ARMING",
					 "BALANCING", "FALLEN", "FAULT", "ESTOP" };

	return (unsigned)s < sizeof(n) / sizeof(n[0]) ? n[s] : "?";
}

const char *bc_fault_name(bc_fault_t f)
{
	static const char *const n[] = { "NONE", "FALL", "LIFT", "IMU_BUS",
		"IMU_STUCK", "IMU_RANGE", "SATURATION", "TIMING", "ESTOP",
		"PARAM", "IMU_INIT", "CALIB" };

	return (unsigned)f < sizeof(n) / sizeof(n[0]) ? n[f] : "?";
}
