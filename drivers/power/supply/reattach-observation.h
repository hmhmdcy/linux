/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SAMURAI_REATTACH_OBSERVATION_H
#define SAMURAI_REATTACH_OBSERVATION_H
#ifdef __KERNEL__
#include <linux/types.h>
typedef u64 sm_observe_ns;
#else
#include <stdint.h>
typedef uint64_t sm_observe_ns;
#endif

#define SM_REATTACH_WINDOW_NS 3000000000ULL
#define SM_REATTACH_FAST_MS 250U
#define SM_REATTACH_NORMAL_MS 1000U

/* All fields belong to the existing charger mutex. A zeroed state has no
 * offline evidence; initial online and unknown cannot start a short window.
 */
struct sm_reattach_observation {
	sm_observe_ns last_ns, until_ns;
	int have_time, off_ready, active;
};

static inline int sm_reattach_time(struct sm_reattach_observation *s,
				 sm_observe_ns now)
{
	if (s->have_time && now < s->last_ns) {
		s->last_ns = now;
		s->active = s->off_ready = 0;
		s->until_ns = 0;
		return 0;
	}
	s->last_ns = now;
	s->have_time = 1;
	if (s->active && now >= s->until_ns) {
		s->active = s->off_ready = 0;
		s->until_ns = 0;
	}
	return 1;
}

/* source has the driver's existing meaning: -1 unknown, 0 agreed physical
 * off, 1 source+ACOK agreed on even if the subsequent budget is unavailable.
 * This is an observation hint only, never charging authorization.
 */
static inline void sm_reattach_source(struct sm_reattach_observation *s,
				      sm_observe_ns now, int source)
{
	if (!sm_reattach_time(s, now) || s->active)
		return;
	if (source == 0) {
		s->off_ready = 1;
	} else if (source == 1) {
		int off = s->off_ready;

		s->off_ready = 0;
		if (off && now <= ~(sm_observe_ns)0 - SM_REATTACH_WINDOW_NS) {
			s->active = 1;
			s->until_ns = now + SM_REATTACH_WINDOW_NS;
		}
	}
}

static inline unsigned int sm_reattach_delay_ms(struct sm_reattach_observation *s,
					       sm_observe_ns now)
{
	sm_observe_ns remaining_ms;

	if (!sm_reattach_time(s, now) || !s->active)
		return SM_REATTACH_NORMAL_MS;
	/* At most one last shortened interval reaches the absolute deadline.
	 * There is no catch-up loop and no new work owner. Workqueue/I2C completion
	 * is not a hard real-time bound; an expired observation returns to 1000ms.
	 */
	remaining_ms = (s->until_ns - now + 999999ULL) / 1000000ULL;
	return remaining_ms < SM_REATTACH_FAST_MS ?
		(unsigned int)remaining_ms : SM_REATTACH_FAST_MS;
}
#endif
