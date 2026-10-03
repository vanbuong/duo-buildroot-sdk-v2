#include <math.h>
#include <string.h>
#include "bc_params.h"

const bc_param_desc_t bc_param_table[] = {
#define BC_X_DESC(n, t, d, lo, hi) \
	{ #n, (uint16_t)offsetof(bc_params_t, n), #t[0], (float)(lo), (float)(hi) },
	BC_PARAM_LIST(BC_X_DESC)
#undef BC_X_DESC
};
const unsigned bc_param_count = sizeof(bc_param_table) / sizeof(bc_param_table[0]);

void bc_params_default(bc_params_t *p)
{
#define BC_X_DEF(n, t, d, lo, hi) p->n = (BC_T_##t)(d);
	BC_PARAM_LIST(BC_X_DEF)
#undef BC_X_DEF
}

double bc_param_get(const bc_params_t *p, unsigned idx)
{
	const bc_param_desc_t *d;
	const char *base = (const char *)p;

	if (idx >= bc_param_count)
		return NAN;
	d = &bc_param_table[idx];
	if (d->type == 'F') {
		float f;
		memcpy(&f, base + d->offset, sizeof(f));
		return f;
	} else {
		int32_t i;
		memcpy(&i, base + d->offset, sizeof(i));
		return i;
	}
}

static int in_range(const bc_param_desc_t *d, double v)
{
	if (!isfinite(v))
		return 0;
	if (v < d->min || v > d->max)
		return 0;
	if (d->type == 'S' && v != 1.0 && v != -1.0)
		return 0;
	if (d->type != 'F' && v != floor(v))
		return 0;
	return 1;
}

int bc_params_validate(const bc_params_t *p)
{
	unsigned i;

	for (i = 0; i < bc_param_count; i++)
		if (!in_range(&bc_param_table[i], bc_param_get(p, i)))
			return (int)i + 1;
	return 0;
}

int bc_param_index(const char *name)
{
	unsigned i;

	for (i = 0; i < bc_param_count; i++)
		if (strcmp(bc_param_table[i].name, name) == 0)
			return (int)i;
	return -1;
}

int bc_param_set(bc_params_t *p, unsigned idx, double value)
{
	const bc_param_desc_t *d;
	char *base = (char *)p;

	if (idx >= bc_param_count)
		return -1;
	d = &bc_param_table[idx];
	if (!in_range(d, value))
		return -2;
	if (d->type == 'F') {
		float f = (float)value;
		memcpy(base + d->offset, &f, sizeof(f));
	} else {
		int32_t i = (int32_t)value;
		memcpy(base + d->offset, &i, sizeof(i));
	}
	return 0;
}

uint32_t bc_crc32(const void *data, size_t len)
{
	const uint8_t *b = data;
	uint32_t crc = 0xFFFFFFFFu;
	size_t i;
	int k;

	for (i = 0; i < len; i++) {
		crc ^= b[i];
		for (k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)-(int32_t)(crc & 1u));
	}
	return ~crc;
}
