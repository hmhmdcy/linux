/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SAMURAI_MP2650_CORE_H
#define SAMURAI_MP2650_CORE_H
#ifdef __KERNEL__
#include <linux/errno.h>
#else
#include <errno.h>
#endif

/* All currents/voltages are uA/uV. Temperatures are tenths of a degree C. */
struct sm_band {
	int low, high, current_ua, cell_target_uv, cell_ovp_uv;
};
#include "oem-profile.h"

struct sm_sample {
	int valid, online, protection_ok, budget_ua, temp_decic;
	int pack_uv, cell1_uv, cell2_uv, gauge_flags, fault;
	int gauge_current_ua;
	unsigned int session_seconds;
};

struct sm_target {
	int enable, input_ua, current_ua, cell_uv;
};

struct sm_io {
	void *ctx;
	int (*read)(void *ctx, unsigned int reg);
	int (*write)(void *ctx, unsigned int reg, unsigned int value);
};

static inline int sm_min(int a, int b) { return a < b ? a : b; }

/* Source transitions inhibit immediately but cannot erase an electrical fault.
 * The caller separately resets the latch on real detach or explicit AUTO.
 */
static inline int sm_sample_latch(int latched, int error, int source_inhibit, int fault)
{
	return latched || fault || (error && !source_inhibit);
}

enum sm_policy_reason {
	SM_POLICY_REASON_NONE,
	SM_POLICY_REASON_TEMPERATURE,
	SM_POLICY_REASON_BAND_OVP,
	SM_POLICY_REASON_GAUGE_CHARGE_STOP,
};

static inline int sm_policy_reason_is_thermal(enum sm_policy_reason reason)
{
	return reason == SM_POLICY_REASON_TEMPERATURE ||
		reason == SM_POLICY_REASON_BAND_OVP;
}

#include "thermal-bands.h"

/* The existing cell target is an upper ceiling, never an inferred safe target. */
static inline int sm_policy_with_band_reason(const struct sm_sample *s, int cell_ceiling_uv,
			   int current_cap_ua, int input_cap_ua, int band,
			   struct sm_target *out, enum sm_policy_reason *reason)
{
	unsigned int i;
	int cell_max, sum, delta;
	const struct sm_band *b = 0;
	enum sm_policy_reason unused;

	if (!reason) reason = &unused;
	*reason = SM_POLICY_REASON_NONE;
	*out = (struct sm_target){ 0 };
	if (!s->valid || !s->protection_ok)
		return -ENODATA;
	if (!s->online)
		return 0;
	if (s->fault || (s->gauge_flags & 0x8000))
		return -EIO;
	if (s->session_seconds >= SM_OEM_MAX_SECONDS)
		return -ETIME;
	if (s->budget_ua < 50000 || input_cap_ua < 50000 ||
	    current_cap_ua < 200000)
		return -ENODATA;
	if (s->cell1_uv < 2000000 || s->cell1_uv > 4600000 ||
	    s->cell2_uv < 2000000 || s->cell2_uv > 4600000 ||
	    s->pack_uv < 4000000 || s->pack_uv > 9200000)
		return -ERANGE;
	sum = s->cell1_uv + s->cell2_uv;
	delta = sum - s->pack_uv;
	/* Candidate acquisition consistency bound, not an OEM calibration value. */
	if (delta < -200000 || delta > 200000)
		return -ENODATA;
	/* A globally invalid target ceiling must not be hidden by a
	 * simultaneous temperature or band-voltage inhibit. */
	if (cell_ceiling_uv < 3712500)
		return -ERANGE;
	/* TCA alone is a stop request, not proven normal Full or a permanent
	 * electrical fault. Preserve its raw bit and check all hard predicates
	 * above first; unknown protection, OCA, timeout and bad data still win.
	 */
	if (s->gauge_flags & 0x4000) {
		*reason = SM_POLICY_REASON_GAUGE_CHARGE_STOP;
		return -EAGAIN;
	}
	for (i = 0; i < sizeof(sm_oem_bands) / sizeof(sm_oem_bands[0]); i++)
		if (s->temp_decic >= sm_oem_bands[i].low &&
		    s->temp_decic < sm_oem_bands[i].high) {
			b = &sm_oem_bands[i];
			break;
		}
	if (!b) {
		*reason = SM_POLICY_REASON_TEMPERATURE;
		return -ERANGE;
	}
	/* One OEM effective status selects CC/CV/OVP and Full/recharge. The
	 * immutable original temperature stop still has priority above this.
	 */
	if (band != SM_POLICY_STATIC_BAND) {
		if (band < 0 || band >= 6) {
			*reason = SM_POLICY_REASON_TEMPERATURE;
			return -ERANGE;
		}
		b = &sm_oem_bands[band];
	}
	cell_max = s->cell1_uv > s->cell2_uv ? s->cell1_uv : s->cell2_uv;
	if (cell_max > b->cell_ovp_uv) {
		*reason = SM_POLICY_REASON_BAND_OVP;
		return -EOVERFLOW;
	}
	out->cell_uv = sm_min(b->cell_target_uv, cell_ceiling_uv);
	if (out->cell_uv < 3712500 || out->cell_uv > 4500000)
		return -ERANGE;
	out->cell_uv = 3712500 + (out->cell_uv - 3712500) / 12500 * 12500;
	out->current_ua = sm_min(b->current_ua, current_cap_ua) / 50000 * 50000;
	out->input_ua = sm_min(sm_min(s->budget_ua, input_cap_ua),
			       SM_OEM_INPUT_MAX_UA) / 50000 * 50000;
	out->enable = 1;
	return 0;
}

/* Stateless compatibility entry point for existing callers/tests. The
 * driver uses sm_policy_with_band_reason with its single live thermal state.
 */
static inline int sm_policy_with_reason(const struct sm_sample *s, int cell_ceiling_uv,
			   int current_cap_ua, int input_cap_ua,
			   struct sm_target *out, enum sm_policy_reason *reason)
{
	return sm_policy_with_band_reason(s, cell_ceiling_uv, current_cap_ua,
			input_cap_ua, SM_POLICY_STATIC_BAND, out, reason);
}

/* Compatibility wrapper: existing callers retain their return codes. */
static inline int sm_policy(const struct sm_sample *s, int cell_ceiling_uv,
			   int current_cap_ua, int input_cap_ua,
			   struct sm_target *out)
{
	return sm_policy_with_reason(s, cell_ceiling_uv, current_cap_ua,
				     input_cap_ua, out, 0);
}

static inline int sm_update(const struct sm_io *io, unsigned int reg,
			    unsigned int mask, unsigned int value)
{
	int old, after, ret;
	unsigned int next;

	/* No raw register interface, reset, watchdog kick, OTG or BATTFET writes. */
	if (!((reg == 0x08 && !(mask & ~0x18U)) ||
	      (reg == 0x07 && mask == 0x30) ||
	      ((reg == 0x00 || reg == 0x0f || reg == 0x02) && mask == 0x7f) ||
	      (reg == 0x03 && mask == 0xff) ||
	      (reg == 0x04 && mask == 0x7f)) || !mask)
		return -EINVAL;
	old = io->read(io->ctx, reg);
	if (old < 0)
		return old;
	if (reg == 0x08 && (old & 0xc0))
		return -EBUSY;
	next = (old & ~mask) | (value & mask);
	if (next != (unsigned int)old) {
		ret = io->write(io->ctx, reg, next);
		if (ret)
			return ret;
	}
	after = io->read(io->ctx, reg);
	if (after < 0)
		return after;
	/* Also verify unrelated bits, rather than silently clobbering them. */
	return after == (int)next ? 0 : -EIO;
}

/* Pause normal charge and the input converter; never switch the protection FET.
 * Failure is reported: a dead I2C bus cannot prove the hardware has stopped.
 */
static inline int sm_stop(const struct sm_io *io)
{
	return sm_update(io, 0x08, 0x18, 0x08);
}

static inline int sm_preconditions(const struct sm_io *io)
{
	int cell = io->read(io->ctx, 0x07);
	int control = io->read(io->ctx, 0x08);
	int timers = io->read(io->ctx, 0x09);
	int compensation = io->read(io->ctx, 0x05);

	if (cell < 0) return cell;
	if (control < 0) return control;
	if (timers < 0) return timers;
	if (compensation < 0) return compensation;
	if ((cell & 0xc0) || (control & 0xe0) || (control & 0x26) != 0x06 ||
	    (compensation & 0xfc))
		return -EOPNOTSUPP;
	/* Do not adopt a live watchdog or disable termination/safety timers. */
	if ((timers & 0x30) || (timers & 0x48) != 0x48)
		return -EOPNOTSUPP;
	return 0;
}

/* Apply while suspended, verify both input limits and then resume. A failed
 * transaction never rolls back to a larger old input allowance or enables
 * charging. The caller latches the primary and stop errors separately.
 */
static inline int sm_apply(const struct sm_io *io, const struct sm_target *t,
			   int *stop_error)
{
	int ret, cc, cv, first, second, control, preterm, cell;

	*stop_error = 0;
	if (!t->enable) {
		*stop_error = sm_stop(io);
		return *stop_error;
	}
	if (t->input_ua < 50000 || t->input_ua > SM_OEM_INPUT_MAX_UA ||
	    t->input_ua % 50000 || t->current_ua < 200000 ||
	    t->current_ua > 1100000 || t->current_ua % 50000 ||
	    t->cell_uv < 3712500 || t->cell_uv > 4500000 ||
	    (t->cell_uv - 3712500) % 12500) {
		ret = -EINVAL;
		goto fail;
	}
	ret = sm_preconditions(io);
	if (ret)
		goto fail;
	/* Do not toggle CHG_EN on every poll: it resets the hardware safety timer. */
	first = io->read(io->ctx, 0x00);
	second = io->read(io->ctx, 0x0f);
	cc = io->read(io->ctx, 0x02);
	cv = io->read(io->ctx, 0x04);
	control = io->read(io->ctx, 0x08);
	preterm = io->read(io->ctx, 0x03);
	cell = io->read(io->ctx, 0x07);
	if (first < 0 || second < 0 || cc < 0 || cv < 0 || control < 0 ||
	    preterm < 0 || cell < 0) {
		ret = first < 0 ? first : second < 0 ? second : cc < 0 ? cc :
			cv < 0 ? cv : control < 0 ? control : preterm < 0 ? preterm : cell;
		goto fail;
	}
	if ((first & 0x7f) == t->input_ua / 50000 &&
	    (second & 0x7f) == t->input_ua / 50000 &&
	    (cc & 0x7f) == t->current_ua / 50000 &&
	    (cv & 0x7f) == (t->cell_uv - 3712500) / 12500 * 2 &&
	    preterm == 0x31 &&
	    !(cell & 0x30) &&
	    (control & 0x18) == 0x10)
		return 0;
	ret = sm_stop(io);
	if (ret) goto fail;
	/* OEM 2-cell profile: 6600mV precharge-to-fast threshold. */
	ret = sm_update(io, 0x07, 0x30, 0);
	if (ret) goto fail;
	ret = sm_update(io, 0x0f, 0x7f, t->input_ua / 50000);
	if (ret) goto fail;
	ret = sm_update(io, 0x00, 0x7f, t->input_ua / 50000);
	if (ret) goto fail;
	ret = sm_update(io, 0x02, 0x7f, t->current_ua / 50000);
	if (ret) goto fail;
	/* Conservative verified precharge: code 3 = 180mA with the OEM 110mA
	 * termination request rounded down to the 100mA hardware step.
	 */
	ret = sm_update(io, 0x03, 0xff, 0x31);
	if (ret) goto fail;
	/* OEM recharge offset 100mV/cell is encoded by bit 0 = 0. */
	ret = sm_update(io, 0x04, 0x7f, (t->cell_uv - 3712500) / 12500 * 2);
	if (ret) goto fail;
	ret = sm_update(io, 0x08, 0x18, 0x10);
	if (!ret) return 0;
fail:
	*stop_error = sm_stop(io);
	return ret;
}

/* Normal Full: never enable CHG even transiently, retain the input converter
 * only after current source limits and all inherited configuration are proven.
 * Unlike sm_stop(), stable normal Full is not an input-converter inhibit.
 */
static inline int sm_apply_charge_hold(const struct sm_io *io,
				       const struct sm_target *t, int *stop_error)
{
	int ret, first, second, cc, cv, preterm, control, cell;

	*stop_error = 0;
	/* First hardware action clears CHG; do not call sm_apply() on this path. */
	ret = sm_update(io, 0x08, 0x10, 0);
	if (ret) goto fail;
	if (t->input_ua < 50000 || t->input_ua > SM_OEM_INPUT_MAX_UA ||
	    t->input_ua % 50000 || t->current_ua < 200000 ||
	    t->current_ua > 1100000 || t->current_ua % 50000 ||
	    t->cell_uv < 3712500 || t->cell_uv > 4500000 ||
	    (t->cell_uv - 3712500) % 12500) {
		ret = -EINVAL;
		goto fail;
	}
	ret = sm_preconditions(io);
	if (ret) goto fail;
	first = io->read(io->ctx, 0x00);
	second = io->read(io->ctx, 0x0f);
	cc = io->read(io->ctx, 0x02);
	cv = io->read(io->ctx, 0x04);
	control = io->read(io->ctx, 0x08);
	preterm = io->read(io->ctx, 0x03);
	cell = io->read(io->ctx, 0x07);
	if (first < 0 || second < 0 || cc < 0 || cv < 0 || control < 0 ||
	    preterm < 0 || cell < 0) {
		ret = first < 0 ? first : second < 0 ? second : cc < 0 ? cc :
			cv < 0 ? cv : control < 0 ? control : preterm < 0 ? preterm : cell;
		goto fail;
	}
	if ((first & 0x7f) == t->input_ua / 50000 &&
	    (second & 0x7f) == t->input_ua / 50000 &&
	    (cc & 0x7f) == t->current_ua / 50000 &&
	    (cv & 0x7f) == (t->cell_uv - 3712500) / 12500 * 2 &&
	    preterm == 0x31 && !(cell & 0x30) && !(control & 0x18))
		return 0;
	/* Any discrete limit/configuration change is made with input paused. */
	ret = sm_stop(io);
	if (ret) goto fail;
	ret = sm_update(io, 0x07, 0x30, 0);
	if (ret) goto fail;
	ret = sm_update(io, 0x0f, 0x7f, t->input_ua / 50000);
	if (ret) goto fail;
	ret = sm_update(io, 0x00, 0x7f, t->input_ua / 50000);
	if (ret) goto fail;
	ret = sm_update(io, 0x02, 0x7f, t->current_ua / 50000);
	if (ret) goto fail;
	ret = sm_update(io, 0x03, 0xff, 0x31);
	if (ret) goto fail;
	ret = sm_update(io, 0x04, 0x7f, (t->cell_uv - 3712500) / 12500 * 2);
	if (ret) goto fail;
	/* Unpause only the input converter; CHG_EN stays zero. */
	ret = sm_update(io, 0x08, 0x18, 0);
	if (!ret) return 0;
fail:
	*stop_error = sm_stop(io);
	return ret;
}
#endif
