/* Host stub: delays are no-ops in unit tests. */
#ifndef STUB_DELAY_H
#define STUB_DELAY_H
static inline void mdelay(unsigned ms) { (void)ms; }
static inline void udelay(unsigned us) { (void)us; }
#endif
