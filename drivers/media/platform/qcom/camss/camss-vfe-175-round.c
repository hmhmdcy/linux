// SPDX-License-Identifier: GPL-2.0-only
/* VFE175 FULL output depth conversion, after the 10-bit color transform. */

#include <linux/bitfield.h>
#include <linux/io.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-round.h"

#define VFE_175_FRAME_ENABLE		0x04c
#define VFE_175_FRAME_FULL_POST_PROCESS	BIT(4)
#define VFE_175_FULL_Y_ENABLE		0xe0c
#define VFE_175_FULL_C_ENABLE		0xe2c
#define VFE_175_FULL_ROUND_ENABLE		BIT(10)
#define VFE_175_FULL_CLAMP_ENABLE		BIT(11)
#define VFE_175_FULL_DEPTH_ENABLE		(VFE_175_FULL_ROUND_ENABLE | \
					 VFE_175_FULL_CLAMP_ENABLE)
#define VFE_175_FULL_Y_CLAMP		0xe18
#define VFE_175_FULL_Y_ROUND		0xe1c
#define VFE_175_FULL_C_CLAMP		0xe38
#define VFE_175_FULL_C_ROUND		0xe3c
#define VFE_175_ROUND_CLAMP_MIN		GENMASK(9, 0)
#define VFE_175_ROUND_CLAMP_MAX		GENMASK(29, 20)
#define VFE_175_ROUND_INTERLEAVED		BIT(0)
#define VFE_175_ROUND_PATTERN		GENMASK(2, 1)
#define VFE_175_ROUND_BITS		GENMASK(5, 3)
#define VFE_175_ROUND_FIELDS		GENMASK(5, 0)

static bool vfe_175_round_supported(struct vfe_device *vfe)
{
	return !vfe->res->is_lite && vfe->camss->res->version == CAMSS_8150;
}

static void vfe_175_round_write(struct vfe_device *vfe, u32 reg, u32 mask, u32 data)
{
	u32 value = readl_relaxed(vfe->base + reg);

	writel_relaxed((value & ~mask) | data, vfe->base + reg);
}

int vfe_175_round_configure(struct vfe_device *vfe,
			   const struct vfe_175_round_config *config)
{
	u32 round, mask, maximum, enabled;

	if (!vfe_175_round_supported(vfe) || !config ||
	    (config->output_bits != 8 && config->output_bits != 10))
		return -EINVAL;
	maximum = (1U << config->output_bits) - 1;
	if (config->y_min > config->y_max || config->y_max > maximum ||
	    config->c_min > config->c_max || config->c_max > maximum)
		return -EINVAL;

	mask = VFE_175_ROUND_CLAMP_MIN | VFE_175_ROUND_CLAMP_MAX;
	round = FIELD_PREP(VFE_175_ROUND_BITS, 10 - config->output_bits) |
		FIELD_PREP(VFE_175_ROUND_PATTERN, 3);
	vfe_175_round_write(vfe, VFE_175_FULL_Y_CLAMP, mask,
			    FIELD_PREP(VFE_175_ROUND_CLAMP_MIN, config->y_min) |
			    FIELD_PREP(VFE_175_ROUND_CLAMP_MAX, config->y_max));
	vfe_175_round_write(vfe, VFE_175_FULL_Y_ROUND, VFE_175_ROUND_FIELDS, round);
	vfe_175_round_write(vfe, VFE_175_FULL_C_CLAMP, mask,
			    FIELD_PREP(VFE_175_ROUND_CLAMP_MIN, config->c_min) |
			    FIELD_PREP(VFE_175_ROUND_CLAMP_MAX, config->c_max));
	vfe_175_round_write(vfe, VFE_175_FULL_C_ROUND, VFE_175_ROUND_FIELDS,
			    round | VFE_175_ROUND_INTERLEAVED);
	/* Make both output configurations visible before setting their enables. */
	wmb();
	enabled = config->enabled ? VFE_175_FULL_DEPTH_ENABLE : 0;
	/* Y/C have distinct enable registers. Preserve crop/scaler ownership. */
	vfe_175_round_write(vfe, VFE_175_FULL_Y_ENABLE, VFE_175_FULL_DEPTH_ENABLE, enabled);
	vfe_175_round_write(vfe, VFE_175_FULL_C_ENABLE, VFE_175_FULL_DEPTH_ENABLE, enabled);
	/* Route FULL through crop/round/clamp, even for the uncropped image. */
	wmb();
	vfe_175_round_write(vfe, VFE_175_FRAME_ENABLE,
			    VFE_175_FRAME_FULL_POST_PROCESS,
			    config->enabled ? VFE_175_FRAME_FULL_POST_PROCESS : 0);

	return 0;
}

int vfe_175_round_prepare(struct vfe_line *line)
{
	/* CST enforces the chosen quantization; this clamp is at output depth. */
	const struct vfe_175_round_config config = {
		.y_max = 255,
		.c_max = 255,
		.output_bits = 8,
		.enabled = true,
	};

	if (!line->is_pix)
		return 0;
	if (line->output[0].video_out.active_fmt.fmt.pix_mp.pixelformat != V4L2_PIX_FMT_NV12)
		return -EINVAL;

	return vfe_175_round_configure(to_vfe(line), &config);
}

void vfe_175_round_stop(struct vfe_device *vfe)
{
	if (!vfe_175_round_supported(vfe))
		return;
	vfe_175_round_write(vfe, VFE_175_FRAME_ENABLE,
			    VFE_175_FRAME_FULL_POST_PROCESS, 0);
	vfe_175_round_write(vfe, VFE_175_FULL_Y_ENABLE, VFE_175_FULL_DEPTH_ENABLE, 0);
	vfe_175_round_write(vfe, VFE_175_FULL_C_ENABLE, VFE_175_FULL_DEPTH_ENABLE, 0);
}
