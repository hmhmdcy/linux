/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_A5_IPE_COLOUR_H
#define CAMSS_A5_IPE_COLOUR_H

#include <linux/types.h>

#define CAMSS_A5_IPE_CC_COMMAND_BYTES 44

/* RGB row-major matrix at Q(7 + shift); offsets are RGB signed integers. */
struct camss_a5_ipe_cc {
	s16 matrix[9];
	s16 offset[3];
	u32 shift;
};

int camss_a5_ipe_cc_encode(void *destination, u32 bytes,
			 const struct camss_a5_ipe_cc *cc);

#endif
