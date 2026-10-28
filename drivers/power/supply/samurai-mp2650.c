// SPDX-License-Identifier: GPL-2.0-only
/* Deferred managed-charging driver for RMX1931.
 * The installed mps,mp2650 node can bind before optional providers appear;
 * control remains disabled until a complete, accepted sample exists.
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kconfig.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/power/bq27xxx_battery.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/workqueue.h>

#include "mp2650-core.h"
#include "charge-cycle.h"
#include "charge-cycle-hold.h"
#include "negotiated-input.h"
#include "oem-full-cycle.h"
#include "gauge-i2c.h"
#include "charging-notify.h"
#include "reattach-observation.h"
#include "samurai-status.h"

#define SM_DEFAULT_INPUT_CAP_UA	500000
#define SM_DEFAULT_CURRENT_CAP_UA	500000

#if !IS_REACHABLE(CONFIG_BATTERY_BQ27XXX)
#error "samurai Full-status bridge requires reachable CONFIG_BATTERY_BQ27XXX"
#endif

static bool control;
module_param(control, bool, 0444);
MODULE_PARM_DESC(control, "Enable managed writes; default is shadow mode");
static bool runtime_board;
module_param(runtime_board, bool, 0444);
MODULE_PARM_DESC(runtime_board, "Use the observed RMX1931 board wiring without changing DT");

struct sm_charger {
	struct i2c_client *client, *gauge;
	struct power_supply *input, *protection, *psy, *battery;
	struct power_supply *typec;
	char *battery_name;
	struct mutex lock;
	struct delayed_work work;
	struct sm_reattach_observation reattach;
	struct sm_io io;
	struct sm_sample sample;
	struct sm_target target;
	int cell_ceiling_uv, current_cap_ua, input_cap_ua;
	int last_error, stop_error;
	bool auto_requested, latched, paused, sample_full;
	bool timer_hold_confirmed;
	u64 timer_hold_since_ns;
	bool source_inhibit;
	ktime_t sampled_at;
	struct sm_charge_cycle cycle;
	struct sm_oem_full_state full_state;
	struct sm_oem_full_result full_result;
	bool full_holding, hold_requested, gauge_charge_stop;
	struct sm_thermal_state thermal;
	enum sm_policy_reason thermal_reason;
	struct notifier_block supply_notifier;
	bool notifier_registered;
	struct sm_notify_state notified;
	bool notified_valid;
	bool battery_status_bridge;
	bool controls_charger;
	bool providers_ready;
};

static void sm_status_evidence(const struct sm_charger *c,
			       struct sm_full_status_evidence *e)
{
	*e = (struct sm_full_status_evidence){
		.control = c->controls_charger,
		.auto_requested = c->auto_requested,
		.online = c->sample.online,
		.sample_valid = c->sample.valid,
		.protection_ok = c->sample.protection_ok,
		.cycle_timeout = c->cycle.timed_out,
		.latched = c->latched,
		.last_error = c->last_error,
		.stop_error = c->stop_error,
		.source_inhibit = c->source_inhibit,
		.gauge_charge_stop = c->gauge_charge_stop,
		.logical_full = c->full_state.full,
		.hold_requested = c->hold_requested,
		.full_holding = c->full_holding,
	};
}

/* Keep UPower's Battery state coherent with the charger-owned Full hold.
 * The exported bq27xxx API is part of the kernel Image ABI. This driver is not
 * deployable as a module-only update against an Image without that symbol.
 * The bq27xxx opt-in rejects non-samurai gauges; failure only disables this
 * presentation bridge and never changes charging safety or control flow.
 */
static void sm_battery_status_locked(struct sm_charger *c)
{
	struct sm_full_status_evidence e;
	int ret;

	if (!c->battery_status_bridge || !c->battery)
		return;
	sm_status_evidence(c, &e);
	/* bq27xxx preserves raw CHARGING and applies its own short lease. */
	ret = bq27xxx_battery_set_status_override(
		c->battery, sm_gauge_full_override(false, &e));
	if (ret && ret != -EOPNOTSUPP)
		dev_warn_ratelimited(&c->client->dev,
			"battery Full-status bridge failed: %d\n", ret);
}

static void sm_clear_full_status_locked(struct sm_charger *c)
{
	c->full_holding = false;
	c->hold_requested = false;
	c->notified_valid = false;
	sm_battery_status_locked(c);
}

static int sm_read(void *ctx, unsigned int reg)
{
	struct sm_charger *c = ctx;

	return i2c_smbus_read_byte_data(c->client, reg);
}

static int sm_write(void *ctx, unsigned int reg, unsigned int value)
{
	struct sm_charger *c = ctx;

	return i2c_smbus_write_byte_data(c->client, reg, value);
}

static int sm_gauge_sample(struct sm_charger *c, struct sm_sample *s)
{
	struct i2c_client *g = c->gauge;
	unsigned char packet[36];
	int temp, pack, flags, current_word, ret;

	i2c_lock_bus(g->adapter, I2C_LOCK_SEGMENT);
	temp = sm_gauge_word_locked(g, 0x06, false, 0);
	pack = sm_gauge_word_locked(g, 0x08, false, 0);
	flags = sm_gauge_word_locked(g, 0x0a, false, 0);
	/* OEM FFA5 maps its misleading get_average_current name to 0x0c.
	 * Acquire the actual signed instantaneous net current in the same bus
	 * segment; do not substitute mainline current_now's 0x14 average.
	 */
	current_word = sm_gauge_word_locked(g, 0x0c, false, 0);
	if (temp < 0 || pack < 0 || flags < 0 || current_word < 0) {
		ret = temp < 0 ? temp : pack < 0 ? pack : flags < 0 ? flags : current_word;
		goto out;
	}
	ret = sm_gauge_mac_locked(g, 0x0071, packet);
	if (ret) goto out;
	s->temp_decic = temp - 2731;
	s->pack_uv = pack * 1000;
	s->gauge_flags = flags;
	ret = sm_oem_gauge_current_ua(current_word, &s->gauge_current_ua);
	if (ret) goto out;
	s->cell1_uv = sm_le16(packet + 2) * 1000;
	s->cell2_uv = sm_le16(packet + 4) * 1000;
out:
	i2c_unlock_bus(g->adapter, I2C_LOCK_SEGMENT);
	return ret;
}

static int sm_psy_int(struct power_supply *psy,
		      enum power_supply_property property, int *value)
{
	union power_supply_propval v;
	int ret = power_supply_get_property(psy, property, &v);

	if (!ret) *value = v.intval;
	return ret;
}

/* There is no native->TCPM->native cycle: native supplies the default-Rp
 * allowance; TCPM supplies the committed operating allowance to this charger.
 * An opted-in but unavailable/transitioning supplier never falls back to a
 * stale BC1.2 result. Physical online remains the independent native input.
 */
static int sm_read_typec_allowance(struct power_supply *psy,
				 struct sm_input_allowance *a)
{
	int ret;

	*a = (struct sm_input_allowance){ 0 };
	ret = sm_psy_int(psy, POWER_SUPPLY_PROP_ONLINE, &a->online);
	if (ret) return ret;
	if (!a->online) return -ENODATA;
	ret = sm_psy_int(psy, POWER_SUPPLY_PROP_USB_TYPE, &a->usb_type);
	if (ret) return ret;
	ret = sm_psy_int(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &a->operating_ua);
	if (ret) return ret;
	ret = sm_psy_int(psy, POWER_SUPPLY_PROP_CURRENT_MAX, &a->maximum_ua);
	if (ret) return ret;
	return sm_psy_int(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &a->voltage_uv);
}

static struct power_supply *sm_get_optional_supply(struct device *dev,
						   const char *name);
static struct power_supply *sm_get_typec_supply(struct device *dev);

/* Resolve providers that appeared after the initial probe. A provider is only
 * accepted once, and references remain owned by devres for the client's life.
 */
static void sm_resolve_providers(struct sm_charger *c)
{
	struct power_supply *psy;

	if (!c->input)
		c->input = sm_get_optional_supply(&c->client->dev,
						 "samurai-usb-input");
	if (IS_ERR(c->input)) {
		psy = c->input;
		c->input = NULL;
		dev_warn_ratelimited(&c->client->dev,
			"input provider unavailable: %ld\n", PTR_ERR(psy));
	}
	if (!c->protection)
		c->protection = sm_get_optional_supply(&c->client->dev,
						       "samurai-gauge-protection");
	if (IS_ERR(c->protection)) {
		psy = c->protection;
		c->protection = NULL;
		dev_warn_ratelimited(&c->client->dev,
			"protection provider unavailable: %ld\n", PTR_ERR(psy));
	}
	if (!c->typec)
		c->typec = sm_get_typec_supply(&c->client->dev);
	if (IS_ERR(c->typec)) {
		psy = c->typec;
		c->typec = NULL;
		dev_warn_ratelimited(&c->client->dev,
			"Type-C provider unavailable: %ld\n", PTR_ERR(psy));
	}
	c->providers_ready = c->input && c->protection && c->typec;
}

static int sm_input_budget(struct sm_charger *c, struct sm_input_allowance *a)
{
	struct sm_input_allowance after;
	int ret;

	*a = (struct sm_input_allowance){ 0 };
	if (!c->typec)
		return sm_psy_int(c->input, POWER_SUPPLY_PROP_CURRENT_MAX, &a->operating_ua);
	ret = sm_read_typec_allowance(c->typec, a);
	if (ret) return ret;
	ret = sm_read_typec_allowance(c->typec, &after);
	if (ret) return ret;
	if (!sm_input_allowance_equal(a, &after)) return -EAGAIN;
	ret = sm_typec_allowance_current(a);
	return ret < 0 ? ret : 0;
}

static int sm_collect(struct sm_charger *c, struct sm_sample *s)
{
	int online, status, present, health, ret, online_after;
	struct sm_input_allowance budget, budget_after;

	*s = (struct sm_sample){ 0 };
	c->source_inhibit = false;
	c->sample_full = false;
	if (!c->input) { c->source_inhibit = true; return -ENODATA; }
	ret = sm_psy_int(c->input, POWER_SUPPLY_PROP_ONLINE, &online);
	if (ret) { c->source_inhibit = true; return ret; }
	status = sm_read(c, 0x13);
	if (status < 0) return status;
	/* This board's MPS Rev1.0 REG13 bit1 is an active-high power-good
	 * indication. The generic input driver and live REG13/online evidence
	 * agree on the polarity; the OEM header's POWER_NG name is reversed.
	 */
	if (!!online != !!(status & 0x02)) {
		c->source_inhibit = true;
		return -EAGAIN;
	}
	s->online = !!online;
	if (!s->online) {
		/* Only an agreed physical detach resets the session and fault latch. */
		sm_cycle_update(&c->cycle, ktime_get_boottime_ns(), 0, 0);
		c->latched = false;
		c->thermal = (struct sm_thermal_state){ 0 };
		sm_oem_full_init(&c->full_state);
		s->valid = s->protection_ok = 1;
		return 0;
	}
	ret = sm_input_budget(c, &budget);
	s->budget_ua = budget.operating_ua;
	if (ret) { c->source_inhibit = true; return ret; }
	if (s->budget_ua < 50000) { c->source_inhibit = true; return -ENODATA; }
	if (c->protection) {
		ret = sm_psy_int(c->protection, POWER_SUPPLY_PROP_PRESENT, &present);
		if (ret) return ret;
		ret = sm_psy_int(c->protection, POWER_SUPPLY_PROP_HEALTH, &health);
		if (ret) return ret;
		s->protection_ok = present == 1 && health == POWER_SUPPLY_HEALTH_GOOD;
	}
	ret = sm_gauge_sample(c, s);
	if (ret) return ret;
	/* One owning read per poll, cached for sysfs; never discard a prior fault. */
	ret = sm_read(c, 0x14);
	if (ret < 0) return ret;
	s->fault = ret;
	/* A changing source budget/attachment cannot authorize this sample. */
	ret = sm_psy_int(c->input, POWER_SUPPLY_PROP_ONLINE, &online_after);
	if (ret) { c->source_inhibit = true; return ret; }
	ret = sm_input_budget(c, &budget_after);
	if (ret) { c->source_inhibit = true; return ret; }
	status = sm_read(c, 0x13);
	if (status < 0) return status;
	if (!online_after || !(status & 0x02) ||
	    !sm_input_allowance_equal(&budget, &budget_after)) {
		c->source_inhibit = true;
		return -EAGAIN;
	}
	c->sample_full = (status & 0x0c) == 0x0c &&
		sm_oem_full_current_qualifies(s->gauge_current_ua);
	s->valid = 1;
	return 0;
}

/* Caller holds c->lock. Sysfs limits are applied synchronously, too. */
static int sm_step_locked(struct sm_charger *c)
{
	struct sm_target target = { 0 }, validation_target;
	struct sm_oem_full_sample full_sample;
	struct sm_charge_cycle next_cycle;
	enum sm_policy_reason reason = SM_POLICY_REASON_NONE;
	int ret, validation, source, thermal_hold = 0;
	bool healthy_full, previously_held = c->timer_hold_confirmed;
	bool cycle_reset_requested, cycle_reset_committed;
	u64 now;

	ret = sm_collect(c, &c->sample);
	c->sampled_at = ktime_get_boottime();
	now = ktime_get_boottime_ns();
	c->full_holding = false;
	c->hold_requested = false;
	/* Established timeout is checked before any new Full interpretation. */
	c->sample.session_seconds = c->cycle.timed_out ? SM_OEM_MAX_SECONDS : 0;
	validation = ret;
	if (!validation)
		validation = sm_policy_with_reason(&c->sample, c->cell_ceiling_uv,
				c->current_cap_ua, c->input_cap_ua, &validation_target, &reason);
	if (!c->controls_charger)
		validation = 0;
	else if (c->sample.online && c->sample.valid &&
	    (!validation || sm_policy_reason_is_thermal(reason))) {
		thermal_hold = sm_thermal_gate(&c->thermal, c->sample.temp_decic, 1);
		/* Re-evaluate the complete policy against that same effective band.
		 * Only fresh non-hard-failing samples can advance this history.
		 */
		validation = sm_policy_with_band_reason(&c->sample, c->cell_ceiling_uv,
			c->current_cap_ua, c->input_cap_ua, c->thermal.band,
			&validation_target, &reason);
		if (!validation && thermal_hold)
			validation = -ERANGE;
	}
	source = c->sample.online ? 1 : !ret && c->sample.valid ? 0 : -1;
	sm_reattach_source(&c->reattach, now, source);
	/* OEM Full handling precedes this period's time accumulation. A newly
	 * completed sixth software sample may finish a not-yet-timed-out cycle
	 * at its boundary. Existing timed_out was validated above and cannot reset.
	 */
	full_sample = (struct sm_oem_full_sample){
		.valid = c->sample.valid,
		.source_known = source >= 0,
		.online = c->sample.online,
		.protection_good = c->sample.protection_ok,
		.policy_good = !validation,
		.hard_latched = c->latched,
		.timeout = c->cycle.timed_out,
		.hardware_full = c->sample_full,
		.temp_decic = c->sample.temp_decic,
		.policy_band_valid = 1,
		.policy_band = c->thermal.initialized ? c->thermal.band : SM_POLICY_STATIC_BAND,
		.max_cell_uv = c->sample.cell1_uv > c->sample.cell2_uv ?
			c->sample.cell1_uv : c->sample.cell2_uv,
		.gauge_current_ua = c->sample.gauge_current_ua,
		.cell_target_uv = !validation ? validation_target.cell_uv : 0,
		.gauge_flags = c->sample.gauge_flags,
	};
	if (c->auto_requested)
		c->full_result = sm_oem_full_step(&c->full_state, now, &full_sample);
	else
		c->full_result = sm_oem_full_skip(&c->full_state);
	/* Physical detach is a normal stop/reset, never a new telemetry fault. */
	if (c->full_result.stop_all && !validation && !c->latched && source != 0) {
		ret = -ENODATA;
		validation = ret;
		reason = SM_POLICY_REASON_NONE;
	}
	healthy_full = c->auto_requested &&
		sm_cycle_full_allowed(c->sample_full ||
			((c->full_result.full || c->full_result.recharge) &&
			 !c->full_result.stop_all),
			c->latched, validation);
	cycle_reset_requested = source == 1 && healthy_full;
	next_cycle = c->cycle;
	c->sample.session_seconds = sm_cycle_update_held(&next_cycle, now, source, healthy_full,
		c->timer_hold_confirmed && c->sample.valid && c->sample.protection_ok &&
		!c->sample.fault && !c->latched && !c->stop_error &&
		(!validation || reason == SM_POLICY_REASON_GAUGE_CHARGE_STOP ||
		 sm_policy_reason_is_thermal(reason)), c->timer_hold_since_ns);
	if (!ret && c->controls_charger)
		ret = sm_policy_with_band_reason(&c->sample, c->cell_ceiling_uv,
			c->current_cap_ua, c->input_cap_ua,
			c->thermal.initialized ? c->thermal.band : SM_POLICY_STATIC_BAND,
			&target, &reason);
	if (!ret && thermal_hold) {
		ret = -ERANGE;
		reason = SM_POLICY_REASON_TEMPERATURE;
		target.enable = 0;
	}
	c->thermal_reason = sm_policy_reason_is_thermal(reason) ? reason : SM_POLICY_REASON_NONE;
	c->gauge_charge_stop = reason == SM_POLICY_REASON_GAUGE_CHARGE_STOP;
	/* TCA is an immediate stop request; it never masks a prior hard latch,
	 * becomes normal Full, erases raw flags, or bypasses a failed bus stop.
	 */
	c->latched = sm_sample_latch(c->latched, ret,
		c->source_inhibit || sm_policy_reason_is_thermal(reason) || c->gauge_charge_stop,
		c->sample.fault);
	c->hold_requested = !ret && !c->latched && c->auto_requested &&
		c->full_result.hold_charge_only;
	if (c->hold_requested)
		target.enable = 0; /* Desired CHG, distinct from healthy converter hold. */
	c->timer_hold_confirmed = false;
	if (c->controls_charger) {
		if (ret || c->latched || !c->auto_requested ||
		    (!target.enable && !c->hold_requested)) {
			c->stop_error = sm_stop(&c->io);
			if (c->stop_error) c->latched = true;
			c->timer_hold_confirmed = source == 1 && !c->stop_error && !c->latched &&
				(!c->auto_requested || c->gauge_charge_stop || sm_policy_reason_is_thermal(reason));
			if (c->timer_hold_confirmed && !previously_held)
				c->timer_hold_since_ns = ktime_get_boottime_ns();
			if (!ret) ret = c->stop_error;
			target.enable = 0;
		} else {
			if (c->hold_requested) {
				ret = sm_apply_charge_hold(&c->io, &target, &c->stop_error);
				c->full_holding = !ret;
			} else {
				ret = sm_apply(&c->io, &target, &c->stop_error);
			}
			if (ret) {
				c->latched = true;
				target.enable = 0;
			}
		}
	} else {
		/* Shadow mode stays read-only; an unavailable optional provider
		 * must not overwrite charging limits inherited from firmware. */
		ret = 0;
		target = (struct sm_target){ 0 };
	}
	cycle_reset_committed = !cycle_reset_requested || !c->controls_charger ||
		(!ret && !c->stop_error);
	if (cycle_reset_committed)
		c->cycle = next_cycle;
	else
		/* A failed hold/restart never earns a fresh ten-hour period. The
		 * conservative timer still advances and preserves any timeout.
		 */
		c->sample.session_seconds = sm_cycle_update_held(&c->cycle, now, source, 0,
			c->timer_hold_confirmed && c->sample.valid && c->sample.protection_ok &&
			!c->sample.fault && !c->latched && !c->stop_error &&
			(!validation || reason == SM_POLICY_REASON_GAUGE_CHARGE_STOP ||
			 sm_policy_reason_is_thermal(reason)), c->timer_hold_since_ns);
	c->target = target;
	c->last_error = ret;
	sm_battery_status_locked(c);
	if (ret || c->stop_error)
		dev_warn_ratelimited(&c->client->dev,
			"policy/io error=%d stop error=%d latched=%d shadow=%d\n",
			ret, c->stop_error, c->latched, !c->controls_charger);
	return ret;
}

/* OEM suspend_after_full is disabled on this board: proven active charging
 * remains hardware-autonomous through system suspend. Every unknown, unsafe,
 * latched, timed-out, Full-hold, or already-disabled target still stops.
 * The caller computes this under c->lock after cancelling delayed work.
 */
static bool sm_suspend_needs_stop(const struct sm_charger *c)
{
	return !c->auto_requested || c->latched || c->last_error ||
		c->stop_error || c->source_inhibit || !c->sample.valid ||
		!c->sample.online || !c->sample.protection_ok ||
		c->sample.fault || c->cycle.timed_out || !c->target.enable;
}

/* Called after the safety step, with c->lock held and no I2C bus lock held.
 * bq27xxx STATUS reads its own signed current/FC flags; do not infer net battery
 * charging from CHG_EN or the charger's switching state.
 */
static bool sm_notification_locked(struct sm_charger *c)
{
	struct sm_notify_state state = {
		.health = c->last_error || c->latched || c->stop_error ?
			POWER_SUPPLY_HEALTH_UNSPEC_FAILURE : POWER_SUPPLY_HEALTH_GOOD,
		.error = c->last_error,
		.stop_error = c->stop_error,
		.latched = c->latched,
		.source_inhibit = c->source_inhibit,
		.auto_requested = c->auto_requested,
		.valid = c->sample.valid,
		.online = c->sample.online,
		.protection_ok = c->sample.protection_ok,
		.fault = c->sample.fault,
		.target_enable = c->target.enable,
		.input_ua = c->target.input_ua,
		.current_ua = c->target.current_ua,
		.cell_uv = c->target.cell_uv,
		.logical_full = c->full_state.full,
		.full_holding = c->full_holding,
		.hold_requested = c->hold_requested,
		.gauge_charge_stop = c->gauge_charge_stop,
	};
	bool changed;
	int reg;

	/* Sample after sm_step_locked() has completed any hardware writes. */
	reg = sm_read(c, 0x13);
	if (reg < 0)
		state.charger_error = reg;
	else
		state.charger_status = reg & 0x0e; /* ONLINE and CHG_STAT only */
	state.battery_error = sm_psy_int(c->battery, POWER_SUPPLY_PROP_STATUS,
				       &state.battery_status);
	changed = !c->notified_valid || sm_notify_changed(&c->notified, &state);
	c->notified = state;
	c->notified_valid = true;
	return changed;
}

static void sm_poll(struct work_struct *work)
{
	struct sm_charger *c = container_of(to_delayed_work(work),
					 struct sm_charger, work);
	bool notify = false;

	mutex_lock(&c->lock);
	if (c->paused) goto out;
	sm_resolve_providers(c);
	sm_step_locked(c);
	notify = sm_notification_locked(c);
	queue_delayed_work(system_dfl_wq, &c->work, msecs_to_jiffies(
		sm_reattach_delay_ms(&c->reattach, ktime_get_boottime_ns())));
out:
	mutex_unlock(&c->lock);
	if (notify)
		power_supply_changed(c->psy);
}

static int sm_supply_changed(struct notifier_block *nb, unsigned long event, void *data)
{
	struct sm_charger *c = container_of(nb, struct sm_charger, supply_notifier);
	if (event == PSY_EVENT_PROP_CHANGED &&
	    (data == c->input || data == c->protection || data == c->typec)) {
		/* Invalidate the battery presentation before the deferred poll.
		 * This closes the window where a detach could retain an old Full.
		 */
		mutex_lock(&c->lock);
		if (!c->paused) {
			sm_clear_full_status_locked(c);
			/* Keep pause and scheduling serialized with suspend/cleanup. */
			mod_delayed_work(system_dfl_wq, &c->work, 0);
		}
		mutex_unlock(&c->lock);
	}
	return NOTIFY_DONE;
}

/* Preserve the installed observer's stable-high/low/high ADC read protocol. */
static int sm_adc(struct sm_charger *c, unsigned int low_reg, int *code)
{
	int high, low, after;

	for (int attempt = 0; attempt < 3; attempt++) {
		high = sm_read(c, low_reg + 1);
		if (high < 0) return high;
		low = sm_read(c, low_reg);
		if (low < 0) return low;
		after = sm_read(c, low_reg + 1);
		if (after < 0) return after;
		if (high == after) {
			*code = (high << 2) | (low >> 6);
			return 0;
		}
	}
	return -ENODATA;
}

static int sm_get_property(struct power_supply *psy,
			   enum power_supply_property prop,
			   union power_supply_propval *v)
{
	struct sm_charger *c = power_supply_get_drvdata(psy);
	struct sm_full_status_evidence evidence;
	int value, after, code, ret = 0;

	mutex_lock(&c->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (!c->controls_charger) { ret = -ENODATA; break; }
		v->intval = c->auto_requested ? POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO :
			POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		v->intval = c->last_error || c->latched || c->stop_error ?
			POWER_SUPPLY_HEALTH_UNSPEC_FAILURE : POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		v->strval = c->controls_charger ? "samurai-mp2650-managed" :
			"samurai-mp2650-deferred";
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		value = sm_read(c, 0x00);
		if (value < 0) { ret = value; break; }
		v->intval = (value & 0x7f) * 50000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		value = sm_read(c, 0x02);
		if (value < 0) { ret = value; break; }
		v->intval = (value & 0x7f) * 50000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		value = sm_read(c, 0x04);
		if (value < 0) { ret = value; break; }
		v->intval = 2 * (3712500 + ((value & 0x7e) >> 1) * 12500);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		value = sm_read(c, 0x13);
		if (value < 0) { ret = value; break; }
		if (!(value & 2)) { ret = -ENODATA; break; }
		ret = sm_adc(c, prop == POWER_SUPPLY_PROP_VOLTAGE_NOW ? 0x1c : 0x1e, &code);
		if (ret) break;
		after = sm_read(c, 0x13);
		if (after < 0) { ret = after; break; }
		if (!(after & 2) || ((after ^ value) & 0x0c)) { ret = -ENODATA; break; }
		v->intval = code * (prop == POWER_SUPPLY_PROP_VOLTAGE_NOW ? 25000 : 6250);
		break;
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_STATUS:
		value = sm_read(c, 0x13);
		if (value < 0) { ret = value; break; }
		if (prop == POWER_SUPPLY_PROP_ONLINE) {
			v->intval = c->sample.online;
			break;
		}
		v->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		if (value & 0x02) {
			sm_status_evidence(c, &evidence);
			if (c->sample_full && sm_reg13_full(value, &evidence))
				v->intval = POWER_SUPPLY_STATUS_FULL;
			else if (sm_reg13_charging(value))
				v->intval = POWER_SUPPLY_STATUS_CHARGING;
		}
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&c->lock);
	return ret;
}

static int sm_set_property(struct power_supply *psy,
			   enum power_supply_property prop,
			   const union power_supply_propval *v)
{
	struct sm_charger *c = power_supply_get_drvdata(psy);
	int ret = 0;

	if (!c->controls_charger) return -EROFS;
	mutex_lock(&c->lock);
	if (c->paused) { ret = -EBUSY; goto out; }
	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (v->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
		    v->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE) {
			ret = -EINVAL;
			break;
		}
		if (v->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
		    (!c->providers_ready || c->stop_error)) {
			ret = -EIO;
			break;
		}
		c->auto_requested = v->intval == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
		/* Explicit operator recovery; the next poll must obtain fresh inputs. */
		if (c->auto_requested) c->latched = false;
		else {
			c->full_holding = false;
			c->hold_requested = false;
			c->stop_error = sm_stop(&c->io);
			ret = c->stop_error;
		}
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (v->intval < 50000 || v->intval > SM_OEM_INPUT_MAX_UA)
			ret = -ERANGE;
		else c->input_cap_ua = v->intval;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (v->intval < 200000 || v->intval > 1100000) ret = -ERANGE;
		else c->current_cap_ua = v->intval;
		break;
	default:
		ret = -EINVAL;
	}
	if (!ret) {
		ret = sm_step_locked(c);
		mod_delayed_work(system_dfl_wq, &c->work, msecs_to_jiffies(1000));
	}
	sm_battery_status_locked(c);
out:
	mutex_unlock(&c->lock);
	power_supply_changed(c->psy);
	return ret;
}

static int sm_writeable(struct power_supply *psy, enum power_supply_property p)
{
	struct sm_charger *c = power_supply_get_drvdata(psy);

	return c->controls_charger &&
		(p == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR ||
		p == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT ||
		p == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT);
}

static const enum power_supply_property sm_properties[] = {
	POWER_SUPPLY_PROP_ONLINE, POWER_SUPPLY_PROP_STATUS, POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR, POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW, POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static const struct power_supply_desc sm_desc = {
	.name = "mp2650-input", .type = POWER_SUPPLY_TYPE_USB,
	.properties = sm_properties, .num_properties = ARRAY_SIZE(sm_properties),
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
	.get_property = sm_get_property, .set_property = sm_set_property,
	.property_is_writeable = sm_writeable,
};

static void sm_put_gauge(void *data) { put_device(data); }
static void sm_put_supply(void *data) { power_supply_put(data); }
static void sm_put_module(void *data) { module_put(data); }

static struct power_supply *sm_get_optional_supply(struct device *dev,
						   const char *name)
{
	struct power_supply *psy = power_supply_get_by_name(name);
	struct module *owner;
	struct device_link *link;
	int ret;

	if (!psy)
		return NULL;
	ret = devm_add_action_or_reset(dev, sm_put_supply, psy);
	if (ret) return ERR_PTR(ret);
	/* Prevent a manually loaded provider from unloading while referenced. */
	owner = psy->dev.parent && psy->dev.parent->driver ?
		psy->dev.parent->driver->owner : NULL;
	if (owner) {
		if (!try_module_get(owner)) {
			power_supply_put(psy);
			return ERR_PTR(-ENODEV);
		}
		ret = devm_add_action_or_reset(dev, sm_put_module, owner);
		if (ret) return ERR_PTR(ret);
	}
	/* Order system sleep after real suppliers are available. The device link
	 * is removed by the driver core on consumer unbind or failed probe.
	 */
	if (!psy->dev.parent)
		return psy;
	link = device_link_add(dev, psy->dev.parent, DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!link)
		return psy;
	return psy;
}

/* Optional exact firmware reference to the real TCPM source PSY. This is
 * not a lookup by a constructed supply name. The managed link and provider
 * module reference use the same lifetime rules as the existing input path.
 */
static struct power_supply *sm_get_typec_supply(struct device *dev)
{
	struct power_supply *psy;
	char *name;
	struct device_node *node;
	struct platform_device *pdev;
	struct module *owner;
	struct device_link *link;
	int ret;

	node = of_find_compatible_node(NULL, NULL, "qcom,pm8150b-typec");
	if (!node)
		return NULL;
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return NULL;
	dev_info(dev, "Type-C device parent=%s\n", dev_name(&pdev->dev));
	name = kasprintf(GFP_KERNEL, "tcpm-source-psy-%s", dev_name(&pdev->dev));
	put_device(&pdev->dev);
	if (!name)
		return ERR_PTR(-ENOMEM);
	psy = power_supply_get_by_name(name);
	kfree(name);
	if (!psy)
		return NULL;
	if (!psy->dev.parent || psy->dev.parent == dev ||
	    psy->desc->type != POWER_SUPPLY_TYPE_USB) {
		power_supply_put(psy);
		return NULL;
	}
	owner = psy->dev.parent->driver ? psy->dev.parent->driver->owner : NULL;
	if (owner) {
		if (!try_module_get(owner)) {
			power_supply_put(psy);
			return ERR_PTR(-ENODEV);
		}
		ret = devm_add_action_or_reset(dev, sm_put_module, owner);
		if (ret) return ERR_PTR(ret);
	}
	link = device_link_add(dev, psy->dev.parent, DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!link)
		return psy;
	return psy;
}

/* Root-readable diagnostic snapshot for the future shadow validation window.
 * Fields are an acquired sample, never a claim of independent protection.
 */
static ssize_t sm_telemetry_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sm_charger *c = dev_get_drvdata(dev);
	ssize_t length;

	mutex_lock(&c->lock);
	length = sysfs_emit(buf,
		"shadow=%d managed=%d providers=%d valid=%d online=%d "
		"protection_ok=%d budget_ua=%d "
		"temp_decic=%d pack_uv=%d cell1_uv=%d cell2_uv=%d flags=%u "
		"fault=%u session_seconds=%u desired_enable=%d error=%d "
		"stop_error=%d latched=%d source_inhibit=%d thermal_inhibit=%d thermal_band=%d band_pre_shake=%d cycle_timeout=%d "
		"gauge_current_ua=%d logical_full=%d full_holding=%d hold_requested=%d "
		"full_cause=%d sw_full_count=%u recharge_count=%u gauge_charge_stop=%d age_ms=%lld\n",
		!c->controls_charger, c->controls_charger, c->providers_ready,
		c->sample.valid, c->sample.online, c->sample.protection_ok,
		c->sample.budget_ua, c->sample.temp_decic, c->sample.pack_uv,
		c->sample.cell1_uv, c->sample.cell2_uv, c->sample.gauge_flags,
		c->sample.fault, c->sample.session_seconds, c->target.enable,
		c->last_error, c->stop_error, c->latched,
		c->source_inhibit, c->thermal_reason,
		c->thermal.initialized ? c->thermal.band : SM_POLICY_STATIC_BAND,
		c->thermal.initialized ? c->thermal.pre_shake : SM_THERMAL_PRE_INVALID,
		c->cycle.timed_out,
		c->sample.gauge_current_ua, c->full_state.full, c->full_holding,
		c->hold_requested, c->full_state.cause, c->full_state.sw_count,
		c->full_state.recharge_count, c->gauge_charge_stop,
		c->sample.valid ? ktime_to_ms(ktime_sub(ktime_get_boottime(), c->sampled_at)) : -1LL);
	mutex_unlock(&c->lock);
	return length;
}

static DEVICE_ATTR(telemetry, 0400, sm_telemetry_show, NULL);
static struct attribute *sm_attributes[] = { &dev_attr_telemetry.attr, NULL };
static const struct attribute_group sm_attribute_group = { .attrs = sm_attributes };

static void sm_cleanup(void *data)
{
	struct sm_charger *c = data;
	int ret;

	if (c->notifier_registered) {
		power_supply_unreg_notifier(&c->supply_notifier);
		c->notifier_registered = false;
	}
	mutex_lock(&c->lock);
	c->paused = true;
	mutex_unlock(&c->lock);
	cancel_delayed_work_sync(&c->work);
	if (c->controls_charger) {
		ret = sm_stop(&c->io);
		if (ret) dev_err(&c->client->dev, "remove could not prove stop: %d\n", ret);
	}
	mutex_lock(&c->lock);
	sm_clear_full_status_locked(c);
	mutex_unlock(&c->lock);
}

static int sm_battery_notify_target(struct power_supply *psy, void *data)
{
	struct sm_charger *c = data;

	if (psy->desc->type != POWER_SUPPLY_TYPE_BATTERY ||
	    psy->dev.parent != &c->gauge->dev)
		return 0;
	if (c->battery_name)
		return -EEXIST;
	c->battery_name = devm_kstrdup(&c->client->dev, psy->desc->name, GFP_KERNEL);
	return c->battery_name ? 0 : -ENOMEM;
}

static int sm_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config config = { 0 };
	struct device_node *node;
	struct sm_charger *c;
	int reg, ret;

	if (!of_machine_is_compatible("realme,samurai") || client->addr != 0x5c)
		return -ENODEV;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EOPNOTSUPP;
	/* Explicitly opt into the OEM MAC telemetry protocol after compatibility
	 * review. The observed extended 2719 identity alone is insufficient.
	 */
	if (!device_property_read_bool(dev, "realme,bq28z610-cells-0071"))
		return dev_err_probe(dev, -EINVAL, "cell telemetry protocol not selected\n");
	c = devm_kzalloc(dev, sizeof(*c), GFP_KERNEL);
	if (!c) return -ENOMEM;
	c->client = client;
	c->io = (struct sm_io){ c, sm_read, sm_write };
	mutex_init(&c->lock);
	sm_cycle_init(&c->cycle, ktime_get_boottime_ns());
	sm_oem_full_init(&c->full_state);
	INIT_DELAYED_WORK(&c->work, sm_poll);
	i2c_set_clientdata(client, c);
	/* The DT contains real firmware-node links. The module parameter remains
	 * only for old images whose board description lacks those properties.
	 */
	c->input = runtime_board ? sm_get_optional_supply(dev, "samurai-usb-input") :
		devm_power_supply_get_by_reference(dev, "input-supply");
	if (IS_ERR(c->input))
		return dev_err_probe(dev, PTR_ERR(c->input), "input supply\n");
	c->protection = runtime_board ?
		sm_get_optional_supply(dev, "samurai-gauge-protection") :
		devm_power_supply_get_by_reference(dev, "protection-supply");
	if (IS_ERR(c->protection))
		return dev_err_probe(dev, PTR_ERR(c->protection), "protection supply\n");
	c->typec = sm_get_typec_supply(dev);
	if (IS_ERR(c->typec)) {
		dev_warn(dev, "Type-C provider unavailable: %ld\n",
			 PTR_ERR(c->typec));
		c->typec = NULL;
	}
	if (IS_ERR(c->input) || IS_ERR(c->protection)) {
		dev_warn(dev, "optional provider lookup failed; shadow only\n");
		c->input = IS_ERR(c->input) ? NULL : c->input;
		c->protection = IS_ERR(c->protection) ? NULL : c->protection;
	}
	dev_info(dev, "provider discovery input=%d protection=%d typec=%d\n",
		 !!c->input, !!c->protection, !!c->typec);
	c->providers_ready = c->input && c->protection && c->typec;
	if (control && !c->providers_ready)
		dev_dbg(dev, "managed providers not ready; binding shadow only\n");
	node = of_parse_phandle(dev->of_node, "battery-gauge", 0);
	if (!node && runtime_board)
		node = of_find_node_by_path("/soc@0/geniqup@cc0000/i2c@c94000/fuel-gauge@55");
	if (!node) return -EINVAL;
	c->gauge = of_find_i2c_device_by_node(node);
	of_node_put(node);
	if (!c->gauge) return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(dev, sm_put_gauge, &c->gauge->dev);
	if (ret) return ret;
	if (c->gauge == client || c->gauge->addr != 0x55 ||
	    IS_ENABLED(CONFIG_BATTERY_BQ27XXX_DT_UPDATES_NVM) ||
	    !i2c_check_functionality(c->gauge->adapter,
		I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_I2C))
		return -EOPNOTSUPP;
	if (!device_link_add(dev, &c->gauge->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		return -ENOMEM;
	ret = sm_preconditions(&c->io);
	if (ret) return dev_err_probe(dev, ret, "unsupported inherited charger state\n");
	reg = sm_read(c, 0x04);
	if (reg < 0) return reg;
	c->cell_ceiling_uv = 3712500 + ((reg & 0x7e) >> 1) * 12500;
	c->current_cap_ua = SM_DEFAULT_CURRENT_CAP_UA;
	c->input_cap_ua = SM_DEFAULT_INPUT_CAP_UA;
	c->last_error = -ENODATA;
	config.drv_data = c;
	config.fwnode = dev_fwnode(dev);
	/* Route external-power changes to the existing gauge's standard callback.
	 * Without this supplier relation bq27xxx can wait for its 360s poll, while
	 * the desktop continues to display the previous charging state. Match the
	 * actual gauge parent, never a guessed global battery name.
	 */
	ret = power_supply_for_each_psy(c, sm_battery_notify_target);
	if (ret)
		return ret;
	if (!c->battery_name)
		return dev_err_probe(dev, -ENODEV, "battery supply not found\n");
	c->battery = power_supply_get_by_name(c->battery_name);
	if (!c->battery)
		return dev_err_probe(dev, -ENODEV, "battery supply vanished\n");
	ret = devm_add_action_or_reset(dev, sm_put_supply, c->battery);
	if (ret)
		return ret;
	if (c->battery->desc->type != POWER_SUPPLY_TYPE_BATTERY ||
	    c->battery->dev.parent != &c->gauge->dev)
		return -ENODEV;
	ret = bq27xxx_battery_set_status_override(c->battery, false);
	if (ret)
		dev_warn(dev, "battery Full-status bridge unavailable: %d\n", ret);
	c->battery_status_bridge = !ret;
	config.supplied_to = &c->battery_name;
	config.num_supplicants = 1;
	/* SYSFS attribute permissions are sampled while the supply is registered.
	 * Set the final ownership mode before registration, not after it.
	 */
	c->controls_charger = control && c->providers_ready;
	c->psy = devm_power_supply_register(dev, &sm_desc, &config);
	if (IS_ERR(c->psy)) return PTR_ERR(c->psy);
	ret = devm_device_add_group(dev, &sm_attribute_group);
	if (ret) return ret;
	c->supply_notifier.notifier_call = sm_supply_changed;
	ret = power_supply_reg_notifier(&c->supply_notifier);
	if (ret) return ret;
	c->notifier_registered = true;
	ret = devm_add_action_or_reset(dev, sm_cleanup, c);
	if (ret) return ret;
	/* Cold boot is fail-safe: the integrated service must explicitly request
	 * AUTO only after limits and providers have been verified.
	 */
	c->auto_requested = false;
	if (control)
		c->stop_error = sm_stop(&c->io);
	c->latched = !!c->stop_error;
	/* A late optional provider may leave the probe read-only. Never advertise
	 * writable controls in that state; the board service removes and reloads
	 * the module after all providers are present.
	 */
	if (!c->controls_charger && control)
		dev_info(dev, "control requested but providers incomplete\n");
	queue_delayed_work(system_dfl_wq, &c->work, 0);
	dev_info(dev, "deferred managed charger bound in %s mode\n",
		 c->controls_charger ? "control" : "shadow");
	return 0;
}

static int sm_suspend(struct device *dev)
{
	struct sm_charger *c = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&c->lock);
	c->paused = true;
	mutex_unlock(&c->lock);
	cancel_delayed_work_sync(&c->work);
	mutex_lock(&c->lock);
	if (sm_suspend_needs_stop(c)) {
		sm_clear_full_status_locked(c);
		if (c->controls_charger) ret = sm_stop(&c->io);
	}
	if (ret) {
		c->stop_error = c->last_error = ret;
		c->latched = true;
		c->paused = false;
		mod_delayed_work(system_dfl_wq, &c->work, 0);
	}
	mutex_unlock(&c->lock);
	return ret;
}

static int sm_resume(struct device *dev)
{
	struct sm_charger *c = dev_get_drvdata(dev);

	mutex_lock(&c->lock);
	c->paused = false;
	/* The first fresh post-resume sample is always reported. */
	c->notified_valid = false;
	mod_delayed_work(system_dfl_wq, &c->work, 0);
	mutex_unlock(&c->lock);
	return 0;
}

static void sm_shutdown(struct i2c_client *client) { sm_cleanup(i2c_get_clientdata(client)); }
static DEFINE_SIMPLE_DEV_PM_OPS(sm_pm_ops, sm_suspend, sm_resume);
static const struct of_device_id sm_match[] = {
	{ .compatible = "realme,samurai-mp2650-managed" },
	{ .compatible = "mps,mp2650" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm_match);
/* Explicitly loaded runtime_board can adopt the existing, first unbound
 * observer client. All board/protocol/provider checks still run at probe.
 */
static const struct i2c_device_id sm_ids[] = { { "mp2650" }, { } };
MODULE_DEVICE_TABLE(i2c, sm_ids);
static struct i2c_driver sm_driver = {
	.driver = { .name = "samurai-mp2650", .of_match_table = sm_match,
		.pm = pm_sleep_ptr(&sm_pm_ops) },
	.probe = sm_probe, .shutdown = sm_shutdown, .id_table = sm_ids,
};
module_i2c_driver(sm_driver);
MODULE_DESCRIPTION("RMX1931 OEM-profile managed MP2650 charging candidate");
MODULE_LICENSE("GPL");
