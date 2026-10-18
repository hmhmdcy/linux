// SPDX-License-Identifier: GPL-2.0-only
/* Qualcomm SM8150 full VFE175 pixel module programming. */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/qcom-camss-config.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-pixel.h"
#include "camss-vfe-175-demosaic.h"

#define VFE_175_WB_GAIN_0		0x6fc
#define VFE_175_WB_GAIN_1		0x700
#define VFE_175_WB_OFFSET_0	0x704
#define VFE_175_WB_OFFSET_1	0x708
#define VFE_175_WB_GREEN		GENMASK(11, 0)
#define VFE_175_WB_BLUE		GENMASK(27, 16)
#define VFE_175_WB_RED		GENMASK(11, 0)
#define VFE_175_WB_OFFSET_GB	(GENMASK(14, 0) | GENMASK(30, 16))
#define VFE_175_WB_OFFSET_R	GENMASK(14, 0)
#define VFE_175_WB_UNITY		128

static u32 vfe_175_wb_q7(u16 gain)
{
	/* The common block uses U15Q10; this hardware uses U12Q7. */
	return min_t(u32, ((u32)gain + 4) >> 3, 4095);
}

static void vfe_175_wb_write(struct vfe_device *vfe)
{
	const struct vfe_175_pixel_state *state = &vfe->pixel175;
	u32 g = VFE_175_WB_UNITY, b = g, r = g;
	u32 value;

	if (state->wb_enabled) {
		g = vfe_175_wb_q7(state->g_gain);
		b = vfe_175_wb_q7(state->b_gain);
		r = vfe_175_wb_q7(state->r_gain);
	}

	value = readl_relaxed(vfe->base + VFE_175_WB_GAIN_0);
	value &= ~(VFE_175_WB_GREEN | VFE_175_WB_BLUE);
	value |= FIELD_PREP(VFE_175_WB_GREEN, g) |
		 FIELD_PREP(VFE_175_WB_BLUE, b);
	writel_relaxed(value, vfe->base + VFE_175_WB_GAIN_0);
	value = readl_relaxed(vfe->base + VFE_175_WB_GAIN_1);
	value &= ~VFE_175_WB_RED;
	writel_relaxed(value | FIELD_PREP(VFE_175_WB_RED, r),
		       vfe->base + VFE_175_WB_GAIN_1);

	/* No offset conversion is advertised: only zero offsets are accepted. */
	value = readl_relaxed(vfe->base + VFE_175_WB_OFFSET_0);
	writel_relaxed(value & ~VFE_175_WB_OFFSET_GB,
		       vfe->base + VFE_175_WB_OFFSET_0);
	value = readl_relaxed(vfe->base + VFE_175_WB_OFFSET_1);
	writel_relaxed(value & ~VFE_175_WB_OFFSET_R,
		       vfe->base + VFE_175_WB_OFFSET_1);
}

static int vfe_175_wb_validate(struct device *dev,
			       const struct v4l2_isp_block_header *block)
{
	const struct camss_params_wb_gain *wb;

	if (block->flags & ~(V4L2_ISP_PARAMS_FL_BLOCK_ENABLE |
			     V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) ||
	    block->flags == (V4L2_ISP_PARAMS_FL_BLOCK_ENABLE |
			     V4L2_ISP_PARAMS_FL_BLOCK_DISABLE))
		return -EINVAL;
	/* A DISABLE block may contain only the header. Never read its payload. */
	if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE)
		return 0;

	wb = container_of(block, struct camss_params_wb_gain, header);
	if (wb->g_gain > 32767 || wb->b_gain > 32767 || wb->r_gain > 32767 ||
	    wb->g_sub || wb->b_sub || wb->r_sub ||
	    wb->g_add || wb->b_add || wb->r_add ||
	    wb->_pad[0] || wb->_pad[1] || wb->_pad[2])
		return -EINVAL;

	return 0;
}

static int vfe_175_params_unsupported(struct device *dev,
				      const struct v4l2_isp_block_header *block)
{
	return -EINVAL;
}

const struct v4l2_isp_params_block_type_info vfe_175_params_info[2] = {
	[0] = {
		.size = sizeof(struct v4l2_isp_block_header),
		.block_validate = vfe_175_params_unsupported,
	},
	[CAMSS_PARAMS_WB_GAIN] = {
		.size = sizeof(struct camss_params_wb_gain),
		.block_validate = vfe_175_wb_validate,
	},
};

int vfe_175_pixel_prepare(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);

	int ret;

	if (!line->is_pix)
		return 0;
	ret = vfe_175_demosaic_prepare(line);
	if (ret)
		return ret;
	/* Reopening a line starts from neutral gains, then queued params apply. */
	vfe->pixel175 = (struct vfe_175_pixel_state) {
		.g_gain = 1024,
		.b_gain = 1024,
		.r_gain = 1024,
		.wb_enabled = true,
	};
	vfe_175_wb_write(vfe);

	return 0;
}

void vfe_175_pixel_stop(struct vfe_device *vfe)
{
	vfe_175_demosaic_stop(vfe);
}

void vfe_175_params_apply(struct vfe_device *vfe, struct vfe_line *line,
			  const struct v4l2_isp_buffer *params)
{
	size_t offset = 0;

	if (!line->is_pix)
		return;
	/* The common parser checked sizes, types and payloads before queueing. */
	while (offset < params->data_size) {
		const struct v4l2_isp_block_header *block =
			(const void *)&params->data[offset];
		const struct camss_params_wb_gain *wb;

		if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
			vfe->pixel175.wb_enabled = false;
		} else {
			wb = container_of(block, struct camss_params_wb_gain, header);
			vfe->pixel175.g_gain = wb->g_gain;
			vfe->pixel175.b_gain = wb->b_gain;
			vfe->pixel175.r_gain = wb->r_gain;
			if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_ENABLE)
				vfe->pixel175.wb_enabled = true;
		}
		vfe_175_wb_write(vfe);
		offset += block->size;
	}
}
