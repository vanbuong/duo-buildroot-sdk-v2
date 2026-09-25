#ifndef BALANCE_PID_H
#define BALANCE_PID_H

typedef struct {
	float kp;
	float ki;
	float kd;
	float integral;
	float prev_err;
	float out_min;
	float out_max;
	float i_min;
	float i_max;
} bc_pid_t;

void pid_init(bc_pid_t *pid, float kp, float ki, float kd,
	      float out_min, float out_max);
void pid_reset(bc_pid_t *pid);
float pid_update(bc_pid_t *pid, float setpoint, float measurement, float dt);

#endif /* BALANCE_PID_H */
