/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_A5_IPE_COLOUR_H
#define CAMSS_A5_IPE_COLOUR_H

#include <linux/types.h>

#define CAMSS_A5_IPE_CC_COMMAND_BYTES 44
#define CAMSS_A5_IPE_CC_ARRAY_OFFSET 2592U
#define CAMSS_A5_IPE_CC_POST_OFFSET 32U
#define CAMSS_A5_IPE_CC_PACKET_OFFSET 2656U
#define CAMSS_A5_IPE_CC_IQ_BYTES 2700U

/* RGB row-major matrix at Q(7 + shift); offsets are RGB signed integers. */
struct camss_a5_ipe_cc {
	s16 matrix[9];
	s16 offset[3];
	u32 shift;
};

int camss_a5_ipe_cc_encode(void *destination, u32 bytes,
			 const struct camss_a5_ipe_cc *cc);

/* IQ and both program arrays live in one caller-owned, mapped request page.
 * Every invalid argument leaves the destination unchanged. No arbitrary
 * command, register address or image pointer is accepted.
 */
int camss_a5_ipe_encode_cc_iq(void *destination, u32 bytes, u32 iq_iova,
			    const struct camss_a5_ipe_cc *cc);

#endif
