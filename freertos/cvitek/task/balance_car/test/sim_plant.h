/*
 * Wheeled inverted pendulum + DC-motor plant used for closed-loop tests.
 * All parameters are ASSUMED values for a ~0.5 kg, 12 V, 65 mm-wheel chassis
 * (see docs/balance_car/05-test-plan.md); they are not measurements.
 *
 * Sign convention required by the firmware control law
 * (motor = Kp * (setpoint - angle), so angle > 0 must give motor < 0):
 *   motor % > 0 : wheels drive "forward".
 *   theta > 0   : body leaning "back", i.e. toward the side that NEGATIVE
 *                 motor drive moves the wheels under. Positive drive
 *                 therefore tips the body further (non-minimum phase).
 * Whether this holds on the real car depends on IMU mounting and motor
 * wiring; docs/balance_car/04 section 7 describes the bring-up check.
 */
#ifndef SIM_PLANT_H
#define SIM_PLANT_H

typedef struct {
	/* parameters */
	double g, l;		/* gravity [m/s^2], CoM height [m] */
	double ku;		/* wheel accel per motor percent [m/s^2/%] */
	double deadband_pct;	/* static friction: |u| below this does nothing */
	double wheel_circ_m;	/* wheel circumference [m] */
	double counts_per_rev;
	double actuator_tau_s;	/* first-order motor lag */
	/* state */
	double theta;		/* [rad] */
	double omega;		/* [rad/s] */
	double x, v;		/* wheel position [m] / speed [m/s] */
	double u_applied;	/* lagged motor drive, percent */
} plant_t;

void plant_init(plant_t *p);
/* Advance by dt with commanded motor percent u (>0 forward). */
void plant_step(plant_t *p, double u_cmd, double dt);
double plant_theta_deg(const plant_t *p);
double plant_encoder_counts(const plant_t *p);
#endif
