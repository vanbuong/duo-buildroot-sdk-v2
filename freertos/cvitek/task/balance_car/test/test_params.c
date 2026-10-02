#include <math.h>
#include <string.h>
#include "unity_lite.h"
#include "bc_params.h"

static void params_defaults_are_valid(void)
{
	bc_params_t p;

	bc_params_default(&p);
	CHECK_EQ(bc_params_validate(&p), 0);
	CHECK_NEAR(p.a_kp, 15.0, 1e-6);
	CHECK_NEAR(p.v_kp, 1.94, 1e-5);
	CHECK_EQ(p.imu_sign, 1);
}

static void params_every_field_rejects_out_of_range(void)
{
	unsigned i;

	for (i = 0; i < bc_param_count; i++) {
		bc_params_t p;
		const bc_param_desc_t *d = &bc_param_table[i];

		bc_params_default(&p);
		CHECK_EQ(bc_param_set(&p, i, (double)d->max + 1000.0), -2);
		CHECK_EQ(bc_param_set(&p, i, (double)d->min - 1000.0), -2);
		CHECK_EQ(bc_param_set(&p, i, NAN), -2);
		CHECK_EQ(bc_params_validate(&p), 0);	/* failed set changes nothing */
	}
}

static void params_set_by_name_roundtrip(void)
{
	bc_params_t p;
	int i;

	bc_params_default(&p);
	i = bc_param_index("a_kd");
	CHECK(i >= 0);
	CHECK_EQ(bc_param_set(&p, (unsigned)i, 1.25), 0);
	CHECK_NEAR(p.a_kd, 1.25, 1e-6);
	CHECK_NEAR(bc_param_get(&p, (unsigned)i), 1.25, 1e-6);
	CHECK_EQ(bc_param_index("nope"), -1);
	CHECK_EQ(bc_param_set(&p, bc_param_count, 1.0), -1);
}

static void params_sign_fields_accept_only_plus_minus_one(void)
{
	bc_params_t p;
	int i = bc_param_index("enc_sign_l");

	bc_params_default(&p);
	CHECK_EQ(bc_param_set(&p, (unsigned)i, -1), 0);
	CHECK_EQ(p.enc_sign_l, -1);
	CHECK_EQ(bc_param_set(&p, (unsigned)i, 0), -2);
	CHECK_EQ(bc_param_set(&p, (unsigned)i, 0.5), -2);
}

static void params_validate_reports_first_bad_field(void)
{
	bc_params_t p;

	bc_params_default(&p);
	p.a_kp = NAN;
	CHECK_EQ(bc_params_validate(&p), bc_param_index("a_kp") + 1);
	bc_params_default(&p);
	p.motor_sign_l = 0;
	CHECK_EQ(bc_params_validate(&p), bc_param_index("motor_sign_l") + 1);
}

static void params_layout_is_dense_32bit(void)
{
	CHECK_EQ(sizeof(bc_params_t), bc_param_count * 4);
	CHECK_EQ(bc_param_table[0].offset, 0);
	CHECK_EQ(bc_param_table[bc_param_count - 1].offset, (bc_param_count - 1) * 4);
}

static void crc32_known_vector(void)
{
	CHECK_EQ(bc_crc32("123456789", 9), 0xCBF43926u);
	CHECK_EQ(bc_crc32("", 0), 0u);
}

void suite_params(void)
{
	printf("suite params\n");
	RUN(params_defaults_are_valid);
	RUN(params_every_field_rejects_out_of_range);
	RUN(params_set_by_name_roundtrip);
	RUN(params_sign_fields_accept_only_plus_minus_one);
	RUN(params_validate_reports_first_bad_field);
	RUN(params_layout_is_dense_32bit);
	RUN(crc32_known_vector);
}
