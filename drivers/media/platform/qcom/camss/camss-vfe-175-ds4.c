// SPDX-License-Identifier: GPL-2.0-only
/* SM8150 first 4:1 YUV downsampler and PD10 packer register composition. */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/string.h>

#include "camss-vfe-175-ds4.h"

#define DS4_FLUSH GENMASK(12, 8)
#define DS4_HEIGHT GENMASK(30, 16)
#define DS4_COEFFICIENT_07 GENMASK(8, 0)
#define DS4_COEFFICIENT_16 GENMASK(18, 10)
#define DS4_COEFFICIENT_25 GENMASK(28, 20)
#define DS4_COEFFICIENTS (DS4_COEFFICIENT_07 | DS4_COEFFICIENT_16 | \
			 DS4_COEFFICIENT_25)
#define R2PD_PACK_MODE BIT(16)

int vfe_175_ds4_encode(const struct vfe_175_ds4_config *config,
		      const u32 previous[4], u32 registers[4])
{
	u32 result[4], sum;

	if (!config || !previous || !registers || config->input_height < 2 ||
	    config->input_height > 8192 || (config->input_height & 1) ||
	    config->coefficient[0] > 256 || config->coefficient[1] > 256 ||
	    config->coefficient[2] > 256)
		return -EINVAL;
	sum = (u32)config->coefficient[0] + config->coefficient[1] +
	      config->coefficient[2];
	if (sum < 256 || sum > 512)
		return -EINVAL;
	/* All checks precede writes; a local result also allows in-place use. */
	result[0] = (previous[0] & ~(DS4_HEIGHT | DS4_FLUSH)) |
		FIELD_PREP(DS4_HEIGHT, config->input_height) |
		FIELD_PREP(DS4_FLUSH, 3);
	result[1] = (previous[1] & ~DS4_COEFFICIENTS) |
		FIELD_PREP(DS4_COEFFICIENT_07, config->coefficient[0]) |
		FIELD_PREP(DS4_COEFFICIENT_16, config->coefficient[1]) |
		FIELD_PREP(DS4_COEFFICIENT_25, config->coefficient[2]);
	result[2] = (previous[2] & ~(DS4_HEIGHT | DS4_FLUSH)) |
		FIELD_PREP(DS4_HEIGHT, config->input_height / 2) |
		FIELD_PREP(DS4_FLUSH, 3);
	/* PD10 is pack mode zero; PD8's mode bit must not survive a change. */
	result[3] = (previous[3] & ~(R2PD_PACK_MODE | DS4_FLUSH)) |
		FIELD_PREP(DS4_FLUSH, 3);
	memcpy(registers, result, sizeof(result));
	return 0;
}
