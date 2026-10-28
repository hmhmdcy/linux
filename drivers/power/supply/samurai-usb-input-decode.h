/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SAMURAI_USB_INPUT_DECODE_H
#define SAMURAI_USB_INPUT_DECODE_H

/* Status fields from the RMX1931 OEM SMB5 header and PM8150B Type-C driver. */
struct samurai_usb_status {
	unsigned char apsd, result, plugin, cc, attached, misc, legacy;
};

enum samurai_usb_kind {
	SAMURAI_USB_UNKNOWN, SAMURAI_USB_SDP, SAMURAI_USB_CDP,
	SAMURAI_USB_DCP, SAMURAI_USB_TYPE_C, SAMURAI_USB_FLOAT,
};

struct samurai_usb_budget {
	int online;
	enum samurai_usb_kind kind;
	int current_ua;
};

/* A detection result is an advertised allowance, never a current measurement.
 * A confirmed SDP has a board-safe 500mA ceiling even when DWC3's configuration
 * callback was missed; a valid callback still narrows the operating allowance
 * to the gadget's unconfigured/configured value.
 * No PD contract, proprietary charging mode or ambiguous result is inferred.
 */
static inline int samurai_usb_decode(const struct samurai_usb_status *s,
				     struct samurai_usb_budget *b)
{
	unsigned int cc = s->cc & 0x7f;
	unsigned int result = s->result & 0x7f;
	int plugin = !!(s->plugin & 0x10);
	int vbus = !!(s->misc & 0x20);
	int attached = !!(s->misc & 1);
	int state_attached = !!(s->attached & 0x20);

	b->online = 0;
	b->kind = SAMURAI_USB_UNKNOWN;
	b->current_ua = -ENODATA;
	if (!plugin && !vbus && !attached && !state_attached) {
		b->current_ua = 0;
		return 0;
	}
	/* Transitional, source-role, water/error and noncompliant states fail. */
	if (!plugin || !vbus || !attached || !state_attached ||
	    (s->misc & 0xd0) || !(s->misc & 8) || (s->legacy & 1) ||
	    (s->plugin & 0x2f))
		return -ENODATA;
	if (cc != 8 && cc != 4 && cc != 2)
		return -ENODATA;
	b->online = 1;
	/* A completed QC detection is outside this observer's contract. */
	if ((s->apsd & 2) || (result & 0x60))
		return 0;
	/* OEM SMB5 accepts USB-stack SDP current after a FLOAT detection.
	 * Keep FLOAT unclassified until a valid native USB2 allowance exists;
	 * never inherit a DCP/Rp high-current budget from this ambiguity.
	 */
	if (result == 0x10) {
		if (!(s->apsd & 1) || s->result != 0x10 || (s->apsd & 0x8c))
			return 0;
		b->kind = SAMURAI_USB_FLOAT;
		return 0;
	}
	if (!(s->legacy & 2) && (cc == 4 || cc == 2)) {
		b->kind = SAMURAI_USB_TYPE_C;
		b->current_ua = cc == 4 ? 1500000 : 3000000;
		return 0;
	}
	if (!(s->apsd & 1))
		return 0;
	switch (result) {
	case 1:
		b->kind = SAMURAI_USB_SDP;
		b->current_ua = 500000;
		break;
	case 4:
		b->kind = SAMURAI_USB_CDP;
		b->current_ua = 1500000;
		break;
	case 8:
		b->kind = SAMURAI_USB_DCP;
		b->current_ua = 1500000;
		break;
	}
	return 0;
}

static inline int samurai_usb_status_equal(const struct samurai_usb_status *a,
					   const struct samurai_usb_status *b)
{
	return a->apsd == b->apsd && a->result == b->result &&
	       a->plugin == b->plugin && a->cc == b->cc &&
	       a->attached == b->attached && a->misc == b->misc &&
	       a->legacy == b->legacy;
}

/* DWC3's accepted SET_CONFIGURATION/reset/suspend callback is an input
 * allowance, not an ADC reading. This USB2 board must never infer 900mA.
 */
static inline int samurai_usb_apply_udc(struct samurai_usb_budget *b,
				       int valid, int current_ua)
{
	if ((b->kind != SAMURAI_USB_SDP && b->kind != SAMURAI_USB_FLOAT) ||
	    !b->online) return 0;
	if (!valid || current_ua < 0 || current_ua > 500000) {
		/* Keep a confirmed SDP at its safe ceiling across the resume gap.
		 * FLOAT remains unavailable until DWC3 authorizes USB2 current.
		 */
		if (b->kind != SAMURAI_USB_SDP)
			b->current_ua = -ENODATA;
		return 0;
	}
	if (b->kind == SAMURAI_USB_FLOAT)
		b->kind = SAMURAI_USB_SDP;
	b->current_ua = current_ua;
	return 0;
}

#endif
