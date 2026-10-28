/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SAMURAI_CHARGE_CYCLE_H
#define SAMURAI_CHARGE_CYCLE_H
#ifdef __KERNEL__
#include <linux/types.h>
typedef u64 sm_cycle_ns;
#else
#include <stdint.h>
typedef uint64_t sm_cycle_ns;
#endif

/* OEM 19781 max_chg_time_sec. State is intentionally volatile like the OEM
 * driver; do not translate this into an immutable ten-hour cable attachment.
 * The caller supplies CLOCK_BOOTTIME/ktime_get_boottime_ns(), not wall-clock.
 */
#define SM_CYCLE_LIMIT_SECONDS 36000U
#define SM_CYCLE_LIMIT_NS (36000ULL * 1000000000ULL)
struct sm_charge_cycle {
	sm_cycle_ns anchor_ns, last_ns;
	unsigned int seconds;
	int attached, timed_out;
};

static inline void sm_cycle_init(struct sm_charge_cycle *c, sm_cycle_ns now)
{
	*c = (struct sm_charge_cycle){ .last_ns = now };
}

/* Run actual policy with timer zeroed unless an old timeout already exists:
 * only a wholly valid, in-range sample can establish a healthy hardware or
 * validated software Full-period boundary (or a normal recharge start).
 * This helper never clears the charger's independent IO/electrical latch.
 */
static inline int sm_cycle_full_allowed(int hardware_full, int fault_latched,
					int validation_error)
{
	return hardware_full && !fault_latched && !validation_error;
}

/* source=-1: unknown/error; 0: independently agreed physical detach;
 * 1: independently agreed attachment. Unknown never clears any cycle state.
 * full_good is allowed only after fresh complete telemetry/policy checks,
 * confirmed MP2650 REG13 CHG_STAT==3 or accepted software-Full/recharge state,
 * and no existing fault latch/timeout. SOC100,
 * zero net current, or an unvalidated cached Full flag is not sufficient.
 * Full resets the current charge period (OEM full_action); subsequent recharge
 * gets a fresh period without requiring a physical unplug. A timeout already
 * established on an earlier sample cannot be erased by Full or AUTO.
 */
static inline unsigned int sm_cycle_update(struct sm_charge_cycle *c,
					  sm_cycle_ns now, int source, int full_good)
{
	sm_cycle_ns elapsed;

	/* No wall-clock/RTC rollback arithmetic; bad input must not underflow. */
	if (now < c->last_ns) {
		c->timed_out = 1;
		c->seconds = SM_CYCLE_LIMIT_SECONDS;
		return c->seconds;
	}
	c->last_ns = now;
	if (source == 0) {
		sm_cycle_init(c, now);
		return 0;
	}
	if (source == 1 && !c->attached) {
		c->attached = 1;
		c->anchor_ns = now;
	}
	if (!c->attached)
		return c->seconds;
	if (c->timed_out)
		return SM_CYCLE_LIMIT_SECONDS;
	if (source == 1 && full_good) {
		c->anchor_ns = now;
		c->seconds = 0;
		return 0;
	}
	elapsed = now - c->anchor_ns;
	if (elapsed >= SM_CYCLE_LIMIT_NS) {
		c->timed_out = 1;
		c->seconds = SM_CYCLE_LIMIT_SECONDS;
	} else {
		c->seconds = elapsed / 1000000000ULL;
	}
	return c->seconds;
}
#endif
