/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef BQ27XXX_STATUS_POLICY_H
#define BQ27XXX_STATUS_POLICY_H

#define BQ27XXX_FULL_STATUS_TTL_MS 4000

/*
 * The caller refreshes a Full override on every successful charger poll.
 * Keeping the deadline wrap-safe avoids an indefinitely sticky battery state
 * if detach handling or that polling worker is delayed.
 */
static inline int bq27xxx_full_status_active(int override,
					     unsigned long deadline,
					     unsigned long now)
{
	return override && (long)(now - deadline) < 0;
}

static inline int bq27xxx_full_status_confirmed(int override,
						unsigned long deadline,
						unsigned long now,
						unsigned int generation,
						unsigned int expected)
{
	return generation == expected &&
		bq27xxx_full_status_active(override, deadline, now);
}

static inline int bq27xxx_full_status_changed(int override,
					      unsigned long deadline,
					      unsigned long now,
					      int next_override)
{
	return bq27xxx_full_status_active(override, deadline, now) !=
			!!next_override ||
		!!override != !!next_override;
}

/*
 * Raw CHARGING always wins. Other raw states are replaced only while the
 * charger-owned Full hold lease is active.
 */
static inline int bq27xxx_full_status_apply(int status, int active,
					    int status_charging,
					    int status_full)
{
	if (active && status != status_charging)
		return status_full;
	return status;
}

#endif
