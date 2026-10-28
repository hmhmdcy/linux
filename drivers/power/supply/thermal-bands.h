/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SAMURAI_THERMAL_BANDS_H
#define SAMURAI_THERMAL_BANDS_H
/* Included after the exact 19781 sm_oem_bands profile.
 * OEM oplus_charger.c 4174-4337: direction is relative to pre_shake, updated
 * on status changes, not on every same-band sample. The unported normal-child
 * initialization's assignment to the parent pre_shake is not reproduced.
 */
#define SM_POLICY_STATIC_BAND (-3)
#define SM_THERMAL_REMOVED (-2)
#define SM_THERMAL_LOW (-1)
#define SM_THERMAL_HIGH 6
#define SM_THERMAL_PRE_INVALID 999
enum sm_thermal_hold { SM_THERMAL_NONE, SM_THERMAL_COLD, SM_THERMAL_HOT };
struct sm_thermal_state {
	enum sm_thermal_hold hold;
	int initialized, band, pre_shake, bounds[7];
};
static inline void sm_thermal_base_bounds(struct sm_thermal_state *s)
{
	unsigned int i;
	for (i = 0; i < 6; i++) s->bounds[i] = sm_oem_bands[i].low;
	s->bounds[6] = sm_oem_bands[5].high;
}
static inline void sm_thermal_init(struct sm_thermal_state *s)
{
	*s = (struct sm_thermal_state){ .initialized = 1, .band = 4,
		.pre_shake = SM_THERMAL_PRE_INVALID };
	sm_thermal_base_bounds(s);
}
static inline int sm_thermal_gate(struct sm_thermal_state *s, int temp, int eligible)
{
	int band, low = 0, high = 0, low0 = 0, high0 = 0;
	if (!eligible) return s->hold != SM_THERMAL_NONE;
	if (!s->initialized) sm_thermal_init(s);
	/* OEM compares >dynamic hot; preserve its status/boundary history, then
	 * enforce r6's immutable absolute >=hot stop separately below.
	 */
	if (temp > s->bounds[6] || temp < s->bounds[0])
		band = temp <= -190 ? SM_THERMAL_REMOVED :
			temp > s->bounds[6] ? SM_THERMAL_HIGH : SM_THERMAL_LOW;
	else {
		for (band = 5; band > 0 && temp < s->bounds[band]; band--) { }
	}
	if (s->pre_shake == SM_THERMAL_PRE_INVALID) s->pre_shake = temp;
	if (band != s->band) {
		if (temp > s->pre_shake) { low = -20; low0 = -5; }
		else if (temp < s->pre_shake) { high = 20; high0 = 5; }
		sm_thermal_base_bounds(s);
		switch (band) {
		case SM_THERMAL_HIGH: s->bounds[6] += low; break;
		case SM_THERMAL_LOW: s->bounds[0] += high; break;
		case 0: s->bounds[0] += low; s->bounds[1] += high0; break;
		case 1: s->bounds[1] += low0; s->bounds[2] += high; break;
		case 2: s->bounds[2] += low; s->bounds[3] += high; break;
		case 3: s->bounds[3] += low; s->bounds[4] += high; break;
		case 5: s->bounds[5] += low; s->bounds[6] += high; break;
		default: break; /* NORMAL and REMOVED restore base boundaries. */
		}
		s->band = band;
		s->pre_shake = temp;
	}
	/* Explicit safer difference from OEM shifted cold/hot limits: raw
	 * <-2 C or >=53 C always stops. No shifted band can authorize outside.
	 * Recovery within that absolute range follows the same dynamic state;
	 * there is no independent fixed 2 C recovery latch.
	 */
	s->hold = temp < sm_oem_bands[0].low || band < 0 ? SM_THERMAL_COLD :
		temp >= sm_oem_bands[5].high || band >= 6 ? SM_THERMAL_HOT : SM_THERMAL_NONE;
	return s->hold != SM_THERMAL_NONE;
}
#endif
