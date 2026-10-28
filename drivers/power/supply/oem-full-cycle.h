/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SAMURAI_OEM_FULL_CYCLE_H
#define SAMURAI_OEM_FULL_CYCLE_H
#ifdef __KERNEL__
#include <linux/errno.h>
#include <linux/types.h>
typedef u64 sm_full_ns;
#else
#include <errno.h>
#include <stdint.h>
typedef uint64_t sm_full_ns;
#endif

#define SM_FULL_TICK_NS (5ULL * 1000000000ULL)
#define SM_FULL_OCA 0x8000U
#define SM_FULL_TCA 0x4000U
enum sm_full_cause { SM_FULL_NONE, SM_FULL_SOFTWARE, SM_FULL_VOLTAGE, SM_FULL_HARDWARE };
enum sm_full_block { SM_FULL_OK, SM_FULL_NO_SOURCE, SM_FULL_BAD_SAMPLE,
	SM_FULL_HARD_FAULT, SM_FULL_TCA_HOLD, SM_FULL_BAD_CLOCK };

struct sm_full_band {
	int low, high, sw_uv, oem_cv_uv, recharge_drop_uv;
};
/* Exact 19781 normal profile. Do not use these reference thresholds as a
 * request to raise the actual inherited voltage or to enter VOOC/FFC modes.
 */
static const struct sm_full_band sm_full_bands[] = {
	{ -20, 0, 3930000, 3980000, 300000 },
	{ 0, 50, 4350000, 4400000, 200000 },
	{ 50, 120, 4350000, 4400000, 100000 },
	{ 120, 160, 4350000, 4400000, 100000 },
	{ 160, 440, 4350000, 4400000, 100000 },
	{ 440, 530, 4080000, 4130000, 100000 },
};

struct sm_oem_full_sample {
	/* valid: current, cells, flags, MAC protection and policy prerequisites
	 * are fresh/complete. source_known is independently agreed PMIC/ACOK;
	 * unknown must not be represented as a real physical detach.
	 */
	int valid, source_known, online, protection_good, policy_good;
	int hard_latched, timeout, hardware_full, recharge_forbidden;
	int temp_decic, max_cell_uv, gauge_current_ua, cell_target_uv;
	int policy_band_valid, policy_band;
	unsigned int gauge_flags;
};
struct sm_oem_full_state {
	sm_full_ns last_seen_ns, last_tick_ns;
	unsigned int sw_count, voltage_count, recharge_count;
	int have_seen, have_tick, band, full;
	enum sm_full_cause cause;
};
struct sm_oem_full_result {
	/* stop_all has priority over every other output. Existing policy remains
	 * the authority for all charger writes and hard fault/timeout latching.
	 * A normal Full uses charge-only stop; retain input converter/USB budget.
	 */
	int stop_all, hold_charge_only, full, entered_full, recharge, tick;
	int sw_threshold_uv, voltage_threshold_uv, recharge_threshold_uv;
	enum sm_full_block block;
	enum sm_full_cause cause;
};

static inline void sm_oem_full_init(struct sm_oem_full_state *s)
{
	*s = (struct sm_oem_full_state){ .band = -1 };
}
static inline void sm_oem_full_clear_streak(struct sm_oem_full_state *s)
{
	s->sw_count = s->voltage_count = s->recharge_count = 0;
	s->have_tick = 0;
}

/* Explicit operator INHIBIT is neither bad telemetry nor physical detach.
 * Keep an established Full fact/cause, discard all incomplete evidence and
 * produce no new Full/recharge/hold request. Real safety checks still run.
 */
static inline struct sm_oem_full_result sm_oem_full_skip(struct sm_oem_full_state *s)
{
	sm_oem_full_clear_streak(s);
	return (struct sm_oem_full_result){ .full = s->full, .cause = s->cause };
}
static inline int sm_oem_full_band(int temp)
{
	unsigned int i;
	for (i = 0; i < sizeof(sm_full_bands) / sizeof(sm_full_bands[0]); i++)
		if (temp >= sm_full_bands[i].low && temp < sm_full_bands[i].high)
			return i;
	return -1;
}

/* OEM FFA5 get_average_current is actually mapped to 0x0c (instantaneous).
 * Positive raw gauge current means charging; OEM negates it at the API edge.
 * Never substitute a cached 0x14 average, input current, or current cap here.
 */
static inline int sm_oem_gauge_current_ua(unsigned int word, int *ua)
{
	int ma;
	if (word > 0xffffU || !ua) return -EINVAL;
	ma = word & 0x8000U ? (int)word - 65536 : (int)word;
	*ua = ma * 1000;
	return 0;
}

static inline int sm_oem_full_current_qualifies(int gauge_current_ua)
{
	return gauge_current_ua >= -110000 && gauge_current_ua <= 110000;
}

static inline struct sm_oem_full_result sm_oem_full_step(struct sm_oem_full_state *s,
				 sm_full_ns now, const struct sm_oem_full_sample *in)
{
	struct sm_oem_full_result out = { 0 };
	const struct sm_full_band *b;
	int band, current_qualifies, due, cv;

	out.full = s->full;
	out.cause = s->cause;
	if (s->have_seen && now < s->last_seen_ns) {
		sm_oem_full_clear_streak(s);
		out.stop_all = 1;
		out.block = SM_FULL_BAD_CLOCK;
		return out;
	}
	/* Never backfill unobserved five-second samples after suspend/starvation.
	 * A missing >=2-period observation gap resets streaks. This explicit
	 * continuity choice permits normal 5.01s scheduling jitter, unlike a
	 * strict >5.000s reset which would make the OEM counts unreachable.
	 */
	if (s->have_seen && now - s->last_seen_ns >= 2 * SM_FULL_TICK_NS)
		sm_oem_full_clear_streak(s);
	s->last_seen_ns = now;
	s->have_seen = 1;
	if (!in->source_known) {
		out.stop_all = 1; out.block = SM_FULL_NO_SOURCE;
		goto blocked;
	}
	if (!in->online) {
		sm_oem_full_init(s);
		out = (struct sm_oem_full_result){ .stop_all = 1,
			.block = SM_FULL_NO_SOURCE };
		return out;
	}
	/* Legacy callers may use the raw lookup. The live driver supplies the
	 * same effective band used by its already validated policy target.
	 */
	band = in->policy_band_valid ? in->policy_band : sm_oem_full_band(in->temp_decic);
	if (!in->valid || !in->policy_good || !in->protection_good ||
	    sm_oem_full_band(in->temp_decic) < 0 || band < 0 || band >= 6 ||
	    in->max_cell_uv < 2000000 || in->max_cell_uv > 4600000 ||
	    in->gauge_current_ua < -32768000 || in->gauge_current_ua > 32767000 ||
	    in->cell_target_uv < 3712500 || in->cell_target_uv > 4500000) {
		out.stop_all = 1; out.block = SM_FULL_BAD_SAMPLE;
		goto blocked;
	}
	if (in->hard_latched || in->timeout || (in->gauge_flags & SM_FULL_OCA)) {
		out.stop_all = 1; out.block = SM_FULL_HARD_FAULT;
		goto blocked;
	}
	/* A normal SafetyStatus alone cannot distinguish TCA's TC condition
	 * from pre-trip SafetyAlert OCC/COV/OTC on this factory 2719 pack.
	 * TCA never authorizes Full/recharge, clears faults, or resets a timer.
	 */
	if (in->gauge_flags & SM_FULL_TCA) {
		out.stop_all = 1; out.block = SM_FULL_TCA_HOLD;
		goto blocked;
	}
	b = &sm_full_bands[band];
	out.sw_threshold_uv = b->sw_uv;
	out.voltage_threshold_uv = b->oem_cv_uv + 18000;
	/* Safer adaptation for the inherited, quantized lower CV: recharge
	 * cannot use a threshold above the voltage this Linux policy can reach.
	 * This changes only a state threshold; it never changes actual CV.
	 */
	cv = in->cell_target_uv < b->oem_cv_uv ? in->cell_target_uv : b->oem_cv_uv;
	if (cv > 4400000) cv = 4400000;
	out.recharge_threshold_uv = cv - b->recharge_drop_uv;
	if (s->band != band) {
		sm_oem_full_clear_streak(s);
		s->band = band;
	}
	current_qualifies = sm_oem_full_current_qualifies(in->gauge_current_ua);
	/* A fresh hardware termination is already an actual completion event,
	 * not a software counter sample. Handle it immediately, even between
	 * software 5s ticks, so a concurrent limit change cannot re-enable CHG.
	 * A heavily discharging pack can retain termination bits from the prior
	 * charge, so current is still a mandatory completion predicate.
	 */
	if (!s->full && in->hardware_full && current_qualifies) {
		s->full = 1; s->cause = SM_FULL_HARDWARE;
		sm_oem_full_clear_streak(s);
		s->last_tick_ns = now; s->have_tick = 1;
		out.full = out.entered_full = out.hold_charge_only = 1;
		out.cause = s->cause;
		return out;
	}
	due = !s->have_tick || now - s->last_tick_ns >= SM_FULL_TICK_NS;
	if (s->full) {
		out.hold_charge_only = 1;
		/* Even a non-tick fresh sample above the threshold disproves a
		 * continuous low-voltage run. Short-c prohibition cannot leave an
		 * already accumulated ready-to-recharge count behind.
		 */
		if (in->max_cell_uv > out.recharge_threshold_uv || in->recharge_forbidden)
			s->recharge_count = 0;
		if (!due) return out;
		s->last_tick_ns = now; s->have_tick = 1; out.tick = 1;
		if (in->max_cell_uv <= out.recharge_threshold_uv && !in->recharge_forbidden)
			s->recharge_count++;
		if (s->recharge_count > 5) {
			s->full = 0; s->cause = SM_FULL_NONE;
			sm_oem_full_clear_streak(s);
			/* No second count at the same timestamp after a transition. */
			s->last_tick_ns = now; s->have_tick = 1;
			out.full = 0; out.hold_charge_only = 0;
			out.recharge = 1; out.cause = SM_FULL_NONE;
		}
		return out;
	}
	if (in->max_cell_uv <= out.sw_threshold_uv || !current_qualifies)
		s->sw_count = 0;
	if (in->max_cell_uv < out.voltage_threshold_uv || !current_qualifies)
		s->voltage_count = 0;
	if (!due) return out;
	s->last_tick_ns = now; s->have_tick = 1; out.tick = 1;
	if (in->max_cell_uv > out.sw_threshold_uv && current_qualifies)
		s->sw_count++;
	if (in->max_cell_uv >= out.voltage_threshold_uv && current_qualifies)
		s->voltage_count++;
	if (s->sw_count > (in->gauge_current_ua > 0 ? 5U : 10U))
		s->cause = SM_FULL_SOFTWARE;
	else if (s->voltage_count >= 4)
		s->cause = SM_FULL_VOLTAGE;
	if (s->cause != SM_FULL_NONE) {
		s->full = 1;
		sm_oem_full_clear_streak(s);
		s->last_tick_ns = now; s->have_tick = 1;
		out.full = out.entered_full = out.hold_charge_only = 1;
		out.cause = s->cause;
	}
	return out;
blocked:
	sm_oem_full_clear_streak(s);
	return out;
}
#endif
