/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SAMURAI_STATUS_H
#define SAMURAI_STATUS_H

/* Complete evidence required before translating a charge-only hold into Full.
 * The raw MP2650 register remains authoritative whenever it reports charging.
 */
struct sm_full_status_evidence {
	int control, auto_requested, online, sample_valid, protection_ok;
	int cycle_timeout, latched, last_error, stop_error, source_inhibit;
	int gauge_charge_stop, logical_full, hold_requested, full_holding;
};

static inline int sm_full_status_safety_valid(
	const struct sm_full_status_evidence *e)
{
	if (!e)
		return 0;
	return e->online && e->sample_valid && e->protection_ok &&
		!e->cycle_timeout && !e->latched && !e->last_error &&
		!e->stop_error && !e->source_inhibit && !e->gauge_charge_stop;
}

static inline int sm_full_status_evidence_valid(
	const struct sm_full_status_evidence *e)
{
	return sm_full_status_safety_valid(e) && e->auto_requested &&
		e->control && e->logical_full && e->hold_requested &&
		e->full_holding;
}

/* REG13 bit1 is ACOK and bits3:2 are CHG_STAT. A stopped converter is made
 * visible as Full only for complete, fault-free Full-hold evidence. Any raw
 * charging indication still wins, and detach remains Not charging.
 */
static inline int sm_reg13_full(const int reg,
				const struct sm_full_status_evidence *e)
{
	if (!(reg & 0x02))
		return 0;
	if ((reg & 0x0c) == 0x0c)
		return sm_full_status_safety_valid(e);
	return !(reg & 0x0c) && sm_full_status_evidence_valid(e);
}

/* REG13 bits 3:2 encode CHG_STAT. Only fast charge and precharge are raw
 * charging states; the hardware-done value must not be relabelled as
 * charging when the safety gate rejects Full.
 */
static inline int sm_reg13_charging(const int reg)
{
	return (reg & 0x0c) == 0x04 || (reg & 0x0c) == 0x08;
}

/* Gauge status remains untouched except while the charger has independently
 * proved the same normal Full hold. Raw Charging is never replaced.
 */
static inline int sm_gauge_full_override(const int raw_charging,
					 const struct sm_full_status_evidence *e)
{
	return !raw_charging && sm_full_status_evidence_valid(e);
}

#endif
