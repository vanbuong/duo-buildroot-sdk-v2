/* Minimal assert framework: no external dependencies, CI-friendly output. */
#ifndef UNITY_LITE_H
#define UNITY_LITE_H
#include <math.h>
#include <stdio.h>

extern int t_checks, t_failures;

#define CHECK(cond) do { \
	t_checks++; \
	if (!(cond)) { \
		t_failures++; \
		printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
	} \
} while (0)

#define CHECK_NEAR(a, b, tol) do { \
	double _a = (a), _b = (b), _t = (tol); \
	t_checks++; \
	if (!(fabs(_a - _b) <= _t)) { \
		t_failures++; \
		printf("    FAIL %s:%d: %s=%g expected %s=%g (+/-%g)\n", \
		       __FILE__, __LINE__, #a, _a, #b, _b, _t); \
	} \
} while (0)

#define CHECK_EQ(a, b) do { \
	long _a = (long)(a), _b = (long)(b); \
	t_checks++; \
	if (_a != _b) { \
		t_failures++; \
		printf("    FAIL %s:%d: %s=%ld expected %s=%ld\n", \
		       __FILE__, __LINE__, #a, _a, #b, _b); \
	} \
} while (0)

typedef void (*test_fn)(void);
#define RUN(fn) do { \
	int _before = t_failures; \
	fn(); \
	printf("  [%s] %s\n", t_failures == _before ? " OK " : "FAIL", #fn); \
} while (0)
#endif
