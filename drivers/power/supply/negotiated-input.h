/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SAMURAI_NEGOTIATED_INPUT_H
#define SAMURAI_NEGOTIATED_INPUT_H

/* TCPM source-PSY values are allowances, not ADC measurements. The board's
 * connector PDOs authorize voltage; this consumer never requests a contract.
 * The caller supplies the kernel power_supply USB-type enum and errno names.
 */
struct sm_input_allowance {
	int online, usb_type, voltage_uv, maximum_ua, operating_ua;
};

static inline int sm_input_allowance_equal(const struct sm_input_allowance *a,
					 const struct sm_input_allowance *b)
{
	return a->online == b->online && a->usb_type == b->usb_type &&
		a->voltage_uv == b->voltage_uv && a->maximum_ua == b->maximum_ua &&
		a->operating_ua == b->operating_ua;
}

static inline int sm_typec_allowance_current(const struct sm_input_allowance *a)
{
	if (a->online < 1 || a->online > 3)
		return -ENODATA;
	if (a->voltage_uv <= 0 || a->maximum_ua < 0 || a->operating_ua < 0)
		return -EPROTO;
	switch (a->usb_type) {
	case POWER_SUPPLY_USB_TYPE_C:
		if (a->online != 1 || a->voltage_uv != 5000000)
			return -EPROTO;
		break;
	case POWER_SUPPLY_USB_TYPE_PD:
		if (a->online != 1)
			return -EPROTO;
		break;
	case POWER_SUPPLY_USB_TYPE_PD_PPS:
		if (a->online == 3)
			return -EPROTO;
		break;
	case POWER_SUPPLY_USB_TYPE_PD_SPR_AVS:
		if (a->online == 2)
			return -EPROTO;
		break;
	case POWER_SUPPLY_USB_TYPE_PD_PPS_SPR_AVS:
		break;
	default:
		return -EOPNOTSUPP;
	}
	/* This board has a 5V-only input-policy contract. PD/PPS/AVS source-PSY
	 * online state does not by itself prove that the negotiated voltage is
	 * safe for the inherited MP2650 configuration.
	 */
	if (a->voltage_uv != 5000000)
		return -EPROTO;
	if (a->operating_ua > a->maximum_ua)
		return -EAGAIN;
	return a->operating_ua;
}
#endif
