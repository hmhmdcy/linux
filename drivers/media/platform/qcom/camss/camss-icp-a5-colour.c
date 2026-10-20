// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "camss-icp-a5-colour.h"

int camss_a5_ipe_cc_encode(void *destination, u32 bytes,
			 const struct camss_a5_ipe_cc *cc)
{
	u8 *output = destination;
	u32 words[9], i;

	if (!output || bytes < CAMSS_A5_IPE_CC_COMMAND_BYTES || !cc ||
	    cc->shift > 3)
		return -EINVAL;
	for (i = 0; i < 9; i++)
		if (cc->matrix[i] < -2048 || cc->matrix[i] > 2047)
			return -ERANGE;
	for (i = 0; i < 3; i++)
		if (cc->offset[i] < -4096 || cc->offset[i] > 4095)
			return -ERANGE;

	/* The IPE register order is GBR, including its output rows. */
	words[0] = (cc->matrix[4] & 0xfff) |
		   ((cc->matrix[7] & 0xfff) << 16);
	words[1] = cc->matrix[1] & 0xfff;
	words[2] = (cc->matrix[5] & 0xfff) |
		   ((cc->matrix[8] & 0xfff) << 16);
	words[3] = cc->matrix[2] & 0xfff;
	words[4] = (cc->matrix[3] & 0xfff) |
		   ((cc->matrix[6] & 0xfff) << 16);
	words[5] = cc->matrix[0] & 0xfff;
	words[6] = (cc->offset[1] & 0x1fff) |
		   ((cc->offset[2] & 0x1fff) << 16);
	words[7] = cc->offset[0] & 0x1fff;
	words[8] = cc->shift;

	/* Only this fixed post-LTM range is encoded; no arbitrary CDM input. */
	put_unaligned_le32(0x03000009, output);
	put_unaligned_le32(0x5e68, output + 4);
	for (i = 0; i < 9; i++)
		put_unaligned_le32(words[i], output + 8 + 4 * i);
	return 0;
}
