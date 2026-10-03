#include "unity_lite.h"

int t_checks, t_failures;

void suite_pid(void);
void suite_encoder(void);
void suite_mpu(void);
void suite_sim(void);
void suite_params(void);
void suite_estimator(void);
void suite_control(void);
void suite_state(void);
void suite_health_cmd(void);
void suite_shm(void);
void suite_i2c_recover(void);
void suite_dmp(void);

int main(void)
{
	suite_pid();
	suite_encoder();
	suite_mpu();
	suite_params();
	suite_estimator();
	suite_control();
	suite_state();
	suite_health_cmd();
	suite_shm();
	suite_i2c_recover();
	suite_dmp();
	suite_sim();
	printf("\n%d checks, %d failures\n", t_checks, t_failures);
	return t_failures ? 1 : 0;
}
