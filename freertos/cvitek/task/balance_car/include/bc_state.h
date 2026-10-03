/* Balance safety state machine - docs/balance_car/01 section 7. Pure logic. */
#ifndef BC_STATE_H
#define BC_STATE_H

#include <stdint.h>
#include "bc_params.h"

typedef enum {
	BC_ST_BOOT = 0,
	BC_ST_CALIBRATING,
	BC_ST_IDLE,
	BC_ST_ARMING,
	BC_ST_BALANCING,
	BC_ST_FALLEN,
	BC_ST_FAULT,
	BC_ST_ESTOP,
} bc_state_id_t;

typedef enum {
	BC_FAULT_NONE = 0,
	BC_FAULT_FALL,
	BC_FAULT_LIFT,
	BC_FAULT_IMU_BUS,
	BC_FAULT_IMU_STUCK,
	BC_FAULT_IMU_RANGE,
	BC_FAULT_SATURATION,
	BC_FAULT_TIMING,
	BC_FAULT_ESTOP,
	BC_FAULT_PARAM,
	BC_FAULT_IMU_INIT,
	BC_FAULT_CALIB,
	BC_FAULT_COUNT
} bc_fault_t;

/* events (bit mask, consumed by one bc_state_step call) */
#define BC_EV_ARM		(1u << 0)
#define BC_EV_DISARM		(1u << 1)
#define BC_EV_ESTOP		(1u << 2)
#define BC_EV_CALIB_REQ		(1u << 3)
#define BC_EV_CALIB_OK		(1u << 4)
#define BC_EV_CALIB_FAIL	(1u << 5)
#define BC_EV_IMU_INIT_FAIL	(1u << 6)

typedef struct {
	bc_state_id_t state;
	bc_fault_t fault;
	float t_in_state_ms;
	float still_ms;
	float sat_ms;
	float lift_ms;
} bc_state_t;

typedef struct {
	float theta;		/* deg */
	float trim;		/* deg */
	float omega;		/* deg/s */
	float v_f;		/* m/s */
	float dt_ms;
	int saturated;
	int imu_bus_fail;	/* >= 3 consecutive read errors */
	int imu_stuck;
	int imu_range;
	int timing_fault;
	uint32_t events;
} bc_state_in_t;

void bc_state_init(bc_state_t *s);
bc_state_id_t bc_state_step(bc_state_t *s, const bc_state_in_t *in,
			    const bc_params_t *p);
static inline int bc_state_is_balancing(const bc_state_t *s)
{
	return s->state == BC_ST_BALANCING;
}
const char *bc_state_name(bc_state_id_t s);
const char *bc_fault_name(bc_fault_t f);

#endif
