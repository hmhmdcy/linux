// SPDX-License-Identifier: GPL-2.0-only
/* VFE175 single-IFE MNDS16 luma scaling and NV12 chroma resampling. */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/math64.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-mnds.h"

#define VFE_175_FRAME_ENABLE		0x04c
#define VFE_175_FRAME_FULL_SCALE		BIT(3)
#define VFE_175_SCALE_Y			0xa3c
#define VFE_175_SCALE_C			0xa68
#define VFE_175_SCALE_AXES		GENMASK(1, 0)
#define VFE_175_SCALE_SIZE_IN		GENMASK(13, 0)
#define VFE_175_SCALE_SIZE_OUT		GENMASK(29, 16)
#define VFE_175_SCALE_C_SIZE_IN		GENMASK(15, 0)
#define VFE_175_SCALE_C_SIZE_OUT		GENMASK(31, 16)
#define VFE_175_SCALE_PHASE_MULT		GENMASK(21, 0)
#define VFE_175_SCALE_INTERP		GENMASK(29, 28)
#define VFE_175_SCALE_MN_INIT		GENMASK(13, 0)
#define VFE_175_SCALE_PAD_WIDTH		GENMASK(13, 0)
#define VFE_175_SCALE_PAD_FIELDS		(GENMASK(29, 0) | BIT(31))

struct vfe_175_mnds_axis {
	u32 input;
	u32 output;
	u32 phase;
	u32 resolution;
};

static bool vfe_175_mnds_supported(struct vfe_device *vfe)
{
	return !vfe->res->is_lite && vfe->camss->res->version == CAMSS_8150;
}

static int vfe_175_mnds_axis(u32 input, u32 output, struct vfe_175_mnds_axis *axis)
{
	u32 ratio, resolution;
	u64 phase;

	if (!input || !output || input > 16384 || output > input)
		return -EINVAL;
	/* The chroma path has its own ratio, including the mandatory 2x decimation. */
	if ((u64)input >= (u64)output * 64)
		return -ERANGE;
	ratio = input / output;
	resolution = ratio < 4 ? 3 : ratio < 8 ? 2 : ratio < 16 ? 1 : 0;
	phase = div_u64((u64)input << (14 + resolution), output);
	if (phase > FIELD_MAX(VFE_175_SCALE_PHASE_MULT))
		return -ERANGE;
	*axis = (struct vfe_175_mnds_axis) {
		.input = input,
		.output = output,
		.phase = phase,
		.resolution = resolution,
	};

	return 0;
}

static int vfe_175_mnds_axes(const struct vfe_175_mnds_config *config,
			     struct vfe_175_mnds_axis axes[4])
{
	int ret;

	if (!config || config->input_width < 2 || config->input_height < 2 ||
	    (config->input_width & 1) || (config->input_height & 1) ||
	    !config->output_width || !config->output_height ||
	    (config->output_width & 1) || (config->output_height & 1))
		return -EINVAL;
	ret = vfe_175_mnds_axis(config->input_width, config->output_width, &axes[0]);
	if (ret)
		return ret;
	ret = vfe_175_mnds_axis(config->input_height, config->output_height, &axes[1]);
	if (ret)
		return ret;
	ret = vfe_175_mnds_axis(config->input_width, config->output_width / 2, &axes[2]);
	if (ret)
		return ret;
	return vfe_175_mnds_axis(config->input_height, config->output_height / 2, &axes[3]);
}

int vfe_175_mnds_from_line(const struct vfe_line *line,
			  struct vfe_175_mnds_config *config)
{
	const struct v4l2_pix_format_mplane *pix;
	struct vfe_175_mnds_config result;
	struct vfe_175_mnds_axis axes[4];
	int ret;

	if (!line || !line->is_pix || !config)
		return -EINVAL;
	pix = &line->output[0].video_out.active_fmt.fmt.pix_mp;
	/* Until a crop backend exists, do not silently ignore a negotiated ROI. */
	if (pix->pixelformat != V4L2_PIX_FMT_NV12 || line->compose.left ||
	    line->compose.top || line->crop.left || line->crop.top ||
	    line->compose.width != pix->width || line->compose.height != pix->height ||
	    line->crop.width != pix->width || line->crop.height != pix->height ||
	    line->fmt[MSM_VFE_PAD_SRC].width != pix->width ||
	    line->fmt[MSM_VFE_PAD_SRC].height != pix->height)
		return -EINVAL;
	result = (struct vfe_175_mnds_config) {
		.input_width = line->fmt[MSM_VFE_PAD_SINK].width,
		.input_height = line->fmt[MSM_VFE_PAD_SINK].height,
		.output_width = pix->width,
		.output_height = pix->height,
		.enabled = true,
	};
	ret = vfe_175_mnds_axes(&result, axes);
	if (ret)
		return ret;
	*config = result;

	return 0;
}

static void vfe_175_mnds_write(struct vfe_device *vfe, u32 reg, u32 mask, u32 data)
{
	u32 value = readl_relaxed(vfe->base + reg);

	writel_relaxed((value & ~mask) | data, vfe->base + reg);
}

static void vfe_175_mnds_write_axis(struct vfe_device *vfe, u32 base,
				   const struct vfe_175_mnds_axis *axis, bool chroma)
{
	u32 in_mask = chroma ? VFE_175_SCALE_C_SIZE_IN : VFE_175_SCALE_SIZE_IN;
	u32 out_mask = chroma ? VFE_175_SCALE_C_SIZE_OUT : VFE_175_SCALE_SIZE_OUT;
	u32 size;

	/* C image sizes are 16-bit fields; Y sizes and pad lengths are 14-bit. */
	if (chroma)
		size = FIELD_PREP(VFE_175_SCALE_C_SIZE_IN, axis->input - 1) |
			FIELD_PREP(VFE_175_SCALE_C_SIZE_OUT, axis->output - 1);
	else
		size = FIELD_PREP(VFE_175_SCALE_SIZE_IN, axis->input - 1) |
			FIELD_PREP(VFE_175_SCALE_SIZE_OUT, axis->output - 1);
	vfe_175_mnds_write(vfe, base, in_mask | out_mask, size);
	vfe_175_mnds_write(vfe, base + 4, VFE_175_SCALE_PHASE_MULT | VFE_175_SCALE_INTERP,
			   FIELD_PREP(VFE_175_SCALE_PHASE_MULT, axis->phase) |
			   FIELD_PREP(VFE_175_SCALE_INTERP, axis->resolution));
	vfe_175_mnds_write(vfe, base + 8, VFE_175_SCALE_MN_INIT, 0);
	vfe_175_mnds_write(vfe, base + 12, VFE_175_SCALE_PHASE_MULT, 0);
	/* Single stripe, no skipped samples or padding; retain the reserved bit. */
	vfe_175_mnds_write(vfe, base + 16, VFE_175_SCALE_PAD_FIELDS,
			   FIELD_PREP(VFE_175_SCALE_PAD_WIDTH, axis->input - 1));
}

int vfe_175_mnds_configure(struct vfe_device *vfe,
			  const struct vfe_175_mnds_config *config)
{
	struct vfe_175_mnds_axis axes[4];
	u32 enable;
	int ret;

	if (!vfe_175_mnds_supported(vfe))
		return -EINVAL;
	/* Check all four axes before writing any of them. */
	ret = vfe_175_mnds_axes(config, axes);
	if (ret)
		return ret;
	vfe_175_mnds_write_axis(vfe, VFE_175_SCALE_Y + 4, &axes[0], false);
	vfe_175_mnds_write_axis(vfe, VFE_175_SCALE_Y + 24, &axes[1], false);
	vfe_175_mnds_write_axis(vfe, VFE_175_SCALE_C + 4, &axes[2], true);
	vfe_175_mnds_write_axis(vfe, VFE_175_SCALE_C + 24, &axes[3], true);
	/* Geometry and phases must reach both scalers before enabling the path. */
	wmb();
	enable = config->enabled ? VFE_175_SCALE_AXES : 0;
	vfe_175_mnds_write(vfe, VFE_175_SCALE_Y, VFE_175_SCALE_AXES, enable);
	vfe_175_mnds_write(vfe, VFE_175_SCALE_C, VFE_175_SCALE_AXES, enable);
	vfe_175_mnds_write(vfe, VFE_175_FRAME_ENABLE, VFE_175_FRAME_FULL_SCALE,
			   config->enabled ? VFE_175_FRAME_FULL_SCALE : 0);

	return 0;
}

void vfe_175_mnds_stop(struct vfe_device *vfe)
{
	if (!vfe_175_mnds_supported(vfe))
		return;
	vfe_175_mnds_write(vfe, VFE_175_FRAME_ENABLE, VFE_175_FRAME_FULL_SCALE, 0);
	vfe_175_mnds_write(vfe, VFE_175_SCALE_Y, VFE_175_SCALE_AXES, 0);
	vfe_175_mnds_write(vfe, VFE_175_SCALE_C, VFE_175_SCALE_AXES, 0);
}
