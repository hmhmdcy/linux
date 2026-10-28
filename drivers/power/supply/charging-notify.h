/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SAMURAI_CHARGING_NOTIFY_H
#define SAMURAI_CHARGING_NOTIFY_H

/* Discrete notification state. Never include ADC noise, age or session time.
 * Read failures have their own fields, so an errno cannot become a valid
 * hardware/battery status and recovery cannot disappear behind cached data.
 */
struct sm_notify_state {
	int charger_status, charger_error;
	int battery_status, battery_error;
	int health, error, stop_error;
	int latched, source_inhibit, auto_requested;
	int valid, online, protection_ok, fault;
	int target_enable, input_ua, current_ua, cell_uv;
	int logical_full, full_holding, hold_requested, gauge_charge_stop;
};

static inline int sm_notify_changed(const struct sm_notify_state *old,
				    const struct sm_notify_state *new)
{
	return old->charger_status != new->charger_status ||
		old->charger_error != new->charger_error ||
		old->battery_status != new->battery_status ||
		old->battery_error != new->battery_error ||
		old->health != new->health || old->error != new->error ||
		old->stop_error != new->stop_error ||
		old->latched != new->latched ||
		old->source_inhibit != new->source_inhibit ||
		old->auto_requested != new->auto_requested ||
		old->valid != new->valid || old->online != new->online ||
		old->protection_ok != new->protection_ok ||
		old->fault != new->fault ||
		old->target_enable != new->target_enable ||
		old->input_ua != new->input_ua ||
		old->current_ua != new->current_ua ||
		old->cell_uv != new->cell_uv ||
		old->logical_full != new->logical_full ||
		old->full_holding != new->full_holding ||
		old->hold_requested != new->hold_requested ||
		old->gauge_charge_stop != new->gauge_charge_stop;
}

#endif
