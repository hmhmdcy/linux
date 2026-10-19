// SPDX-License-Identifier: GPL-2.0-only
/* Qualcomm SM8150 full VFE175 pixel module programming. */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/qcom-camss-config.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-pixel.h"
#include "camss-vfe-175-demosaic.h"
#include "camss-vfe-175-cst.h"
#include "camss-vfe-175-round.h"
#include "camss-vfe-175-gamma.h"
#include "camss-vfe-175-demux.h"
#include "camss-vfe-175-mnds.h"

#define VFE_175_LENS_ENABLE		0x040
#define VFE_175_BLC_ENABLE		BIT(11)
#define VFE_175_BLC_CONFIG		0x6b0
#define VFE_175_BLC_THRESHOLD_0	0x6b4
#define VFE_175_BLC_THRESHOLD_1	0x6b8
#define VFE_175_BLC_OFFSET		GENMASK(13, 0)
#define VFE_175_BLC_SCALE		GENMASK(31, 15)
#define VFE_175_BLC_THRESHOLDS	(GENMASK(13, 0) | GENMASK(29, 16))
#define VFE_175_BLC_UNITY		2048

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

static u32 vfe_175_blc_level(u16 level)
{
	/* Common offsets are full-scale U16; the IFE processes U14 pixels. */
	return ((u32)level * 16383 + 32767) / 65535;
}

static void vfe_175_blc_enable(struct vfe_device *vfe, bool enable)
{
	u32 value = readl_relaxed(vfe->base + VFE_175_LENS_ENABLE);

	value &= ~VFE_175_BLC_ENABLE;
	if (enable)
		value |= VFE_175_BLC_ENABLE;
	writel_relaxed(value, vfe->base + VFE_175_LENS_ENABLE);
}

static void vfe_175_blc_write(struct vfe_device *vfe)
{
	const struct vfe_175_pixel_state *state = &vfe->pixel175;
	u32 level = state->wb_enabled ? vfe_175_blc_level(state->black_level) : 0;
	u32 value;

	value = readl_relaxed(vfe->base + VFE_175_BLC_CONFIG);
	value &= ~(VFE_175_BLC_OFFSET | VFE_175_BLC_SCALE);
	/* Preserve subtract-then-gain semantics; no implicit range stretch. */
	value |= FIELD_PREP(VFE_175_BLC_OFFSET, level) |
		 FIELD_PREP(VFE_175_BLC_SCALE, VFE_175_BLC_UNITY);
	writel_relaxed(value, vfe->base + VFE_175_BLC_CONFIG);
	/* Zero thresholds leave the optional per-channel clipping inactive. */
	value = readl_relaxed(vfe->base + VFE_175_BLC_THRESHOLD_0);
	writel_relaxed(value & ~VFE_175_BLC_THRESHOLDS,
		       vfe->base + VFE_175_BLC_THRESHOLD_0);
	value = readl_relaxed(vfe->base + VFE_175_BLC_THRESHOLD_1);
	writel_relaxed(value & ~VFE_175_BLC_THRESHOLDS,
		       vfe->base + VFE_175_BLC_THRESHOLD_1);
	vfe_175_blc_enable(vfe, level != 0);
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

	vfe_175_blc_write(vfe);

	value = readl_relaxed(vfe->base + VFE_175_WB_GAIN_0);
	value &= ~(VFE_175_WB_GREEN | VFE_175_WB_BLUE);
	value |= FIELD_PREP(VFE_175_WB_GREEN, g) |
		 FIELD_PREP(VFE_175_WB_BLUE, b);
	writel_relaxed(value, vfe->base + VFE_175_WB_GAIN_0);
	value = readl_relaxed(vfe->base + VFE_175_WB_GAIN_1);
	value &= ~VFE_175_WB_RED;
	writel_relaxed(value | FIELD_PREP(VFE_175_WB_RED, r),
		       vfe->base + VFE_175_WB_GAIN_1);

	/* The separate BLS block performs the common pre-gain subtraction. */
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
	    wb->g_sub != wb->b_sub || wb->g_sub != wb->r_sub ||
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

const struct v4l2_isp_params_block_type_info vfe_175_params_info[6] = {
	[0] = {
		.size = sizeof(struct v4l2_isp_block_header),
		.block_validate = vfe_175_params_unsupported,
	},
	[CAMSS_PARAMS_WB_GAIN] = {
		.size = sizeof(struct camss_params_wb_gain),
		.block_validate = vfe_175_wb_validate,
	},
	[CAMSS_PARAMS_CHROMA_ENHAN] = {
		.size = sizeof(struct camss_params_chroma_enhan),
		.block_validate = vfe_175_params_unsupported,
	},
	[CAMSS_PARAMS_COLOR_CORRECT] = {
		.size = sizeof(struct camss_params_color_correct),
		.block_validate = vfe_175_params_unsupported,
	},
	[CAMSS_PARAMS_GAMMA] = {
		.size = sizeof(struct camss_params_gamma),
		.block_validate = vfe_175_params_unsupported,
	},
	[CAMSS_PARAMS_AEC_BHIST] = {
		.size = sizeof(struct camss_params_aec_bhist),
		.block_validate = vfe_175_bhist_validate,
	},
};

int vfe_175_pixel_prepare(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_175_cst_config cst;
	struct vfe_175_gamma_config gamma;
	struct vfe_175_demux_config demux;
	struct vfe_175_mnds_config mnds;
	int ret;

	if (!line->is_pix)
		return 0;
	/* Reject unsupported output colorimetry before any processing MMIO. */
	ret = vfe_175_cst_from_format(&line->output[0].video_out.active_fmt.fmt.pix_mp,
				      &cst);
	if (ret)
		return ret;
	ret = vfe_175_gamma_from_format(&line->output[0].video_out.active_fmt.fmt.pix_mp,
					&gamma);
	if (ret)
		return ret;
	ret = vfe_175_demux_from_code(line->fmt[MSM_VFE_PAD_SINK].code, &demux);
	if (ret)
		return ret;
	ret = vfe_175_mnds_from_line(line, &mnds);
	if (ret)
		return ret;
	ret = vfe_175_demux_configure(vfe, &demux);
	if (ret)
		return ret;
	ret = vfe_175_demosaic_prepare(line);
	if (ret)
		goto error;
	ret = vfe_175_gamma_configure(vfe, &gamma);
	if (ret)
		goto error;
	ret = vfe_175_cst_configure(vfe, &cst);
	if (ret)
		goto error;
	ret = vfe_175_mnds_configure(vfe, &mnds);
	if (ret)
		goto error;
	ret = vfe_175_round_prepare(line);
	if (ret)
		goto error;
	/* Reopening a line starts from neutral gains, then queued params apply. */
	vfe->pixel175 = (struct vfe_175_pixel_state) {
		.g_gain = 1024,
		.b_gain = 1024,
		.r_gain = 1024,
		.wb_enabled = true,
	};
	vfe_175_wb_write(vfe);
	ret = vfe_175_stats_prepare(line);
	if (ret)
		goto error;

	return 0;

error:
	vfe_175_pixel_stop(vfe);
	return ret;
}

void vfe_175_pixel_stop(struct vfe_device *vfe)
{
	vfe_175_blc_enable(vfe, false);
	vfe_175_round_stop(vfe);
	vfe_175_mnds_stop(vfe);
	vfe_175_gamma_stop(vfe);
	vfe_175_cst_stop(vfe);
	vfe_175_demosaic_stop(vfe);
	vfe_175_demux_stop(vfe);
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

		if (block->type == CAMSS_PARAMS_AEC_BHIST) {
			vfe_175_stats_params(vfe, line, block);
			offset += block->size;
			continue;
		}
		if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
			vfe->pixel175.wb_enabled = false;
		} else {
			wb = container_of(block, struct camss_params_wb_gain, header);
			vfe->pixel175.g_gain = wb->g_gain;
			vfe->pixel175.b_gain = wb->b_gain;
			vfe->pixel175.r_gain = wb->r_gain;
			vfe->pixel175.black_level = wb->g_sub;
			if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_ENABLE)
				vfe->pixel175.wb_enabled = true;
		}
		vfe_175_wb_write(vfe);
		offset += block->size;
	}
}
