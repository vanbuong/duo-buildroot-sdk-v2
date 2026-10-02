#include "unity_lite.h"

int t_checks, t_failures;

void suite_pid(void);
void suite_encoder(void);
void suite_mpu(void);
void suite_sim(void);

int main(void)
{
	suite_pid();
	suite_encoder();
	suite_mpu();
	suite_sim();
	printf("\n%d checks, %d failures\n", t_checks, t_failures);
	return t_failures ? 1 : 0;
}
