// SPDX-License-Identifier: GPL-2.0-only
/* SM8150 first 4:1 YUV downsampler and PD10 packer register composition. */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/media-bus-format.h>
#include <linux/qcom-camss-config.h>
#include "camss.h"
#include "camss-format.h"
#include "camss-vfe.h"

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

static const struct camss_format_info formats_ds4_175[] = {
	{ MEDIA_BUS_FMT_UYYVYY10_0_5X30, 15, V4L2_PIX_FMT_QCOM_PD10_LOCAL, 1,
	  PER_PLANE_DATA(0, 1, 1, 2, 1, 32) },
};
const struct camss_formats vfe_formats_ds4_175 = {
	.nformats = ARRAY_SIZE(formats_ds4_175), .formats = formats_ds4_175,
};
const struct v4l2_frmsize_stepwise vfe_frame_sizes_ds4_175 = {
	.min_width = 4, .max_width = 1024, .step_width = 2,
	.min_height = 2, .max_height = 1024, .step_height = 2,
};

/* Runtime first-DS4 control and single-plane WM5 transport. */
static bool ds4_supported(struct vfe_device *vfe)
{
	return vfe->camss->res->version == CAMSS_8150 && !vfe->res->is_lite;
}

static void ds4_update_bits(struct vfe_device *vfe, u32 offset, u32 mask, u32 value)
{
	u32 old = readl_relaxed(vfe->base + offset);

	writel_relaxed((old & ~mask) | value, vfe->base + offset);
}

int vfe_175_ds4_validate(struct device *dev,
		       const struct v4l2_isp_block_header *block)
{
	const struct camss_params_ife_ds4 *data =
		container_of(block, struct camss_params_ife_ds4, header);
	struct vfe_175_ds4_config config = { .input_height = 128 };
	u32 previous[4] = {}, registers[4];

	if (block->flags || data->reserved)
		return -EINVAL;
	memcpy(config.coefficient, data->coefficient, sizeof(config.coefficient));
	return vfe_175_ds4_encode(&config, previous, registers);
}

int vfe_175_ds4_validate_line(struct vfe_line *line,
			    const struct v4l2_isp_buffer *params)
{
	size_t offset = 0;

	while (offset < params->data_size) {
		const struct v4l2_isp_block_header *block =
			(const void *)&params->data[offset];

		/* Generic validation may accept header-only disable requests. DS4
		 * coefficients are configuration; stream lifetime owns its enable. */
		if (block->type == CAMSS_PARAMS_IFE_DS4 && block->flags)
			return -EINVAL;
		offset += block->size;
	}
	return 0;
}

static void ds4_write_filter(struct vfe_device *vfe, struct vfe_line *line)
{
	const struct v4l2_pix_format_mplane *full =
		&line->output[0].video_out.active_fmt.fmt.pix_mp;
	struct vfe_175_ds4_config config = { .input_height = full->height };
	u32 previous[4], registers[4];
	unsigned int i;

	memcpy(config.coefficient, vfe->pixel175.ds4_coefficient,
	       sizeof(config.coefficient));
	for (i = 0; i < 4; i++)
		previous[i] = readl_relaxed(vfe->base + 0xd60 + i * 4);
	if (WARN_ON(vfe_175_ds4_encode(&config, previous, registers)))
		return;
	for (i = 0; i < 4; i++)
		writel_relaxed(registers[i], vfe->base + 0xd60 + i * 4);
}

void vfe_175_ds4_params(struct vfe_device *vfe, struct vfe_line *line,
			const struct v4l2_isp_block_header *block)
{
	const struct camss_params_ife_ds4 *data =
		container_of(block, struct camss_params_ife_ds4, header);

	memcpy(vfe->pixel175.ds4_coefficient, data->coefficient,
	       sizeof(vfe->pixel175.ds4_coefficient));
	vfe->pixel175.ds4_configured = true;
	if (vfe->pixel175.ds4_running)
		ds4_write_filter(vfe, line);
}

int vfe_175_ds4_validate_output(const struct vfe_output *output)
{
	const struct v4l2_pix_format_mplane *pix = &output->video_out.active_fmt.fmt.pix_mp;
	const struct v4l2_pix_format_mplane *full =
		&output->line->output[0].video_out.active_fmt.fmt.pix_mp;
	struct camss_pd10_layout layout;
	int ret;

	if (!output->line->is_pix || output->pad != MSM_VFE_PAD_SRC_DS4 ||
	    output->wm_num != 1 || output->wm[0].bus_client != 5 ||
	    output->wm[0].plane || output->comp_group != 1 ||
	    pix->pixelformat != V4L2_PIX_FMT_QCOM_PD10_LOCAL || pix->num_planes != 1 ||
	    pix->width != (ALIGN(full->width, 8) / 4) ||
	    pix->height != (ALIGN(full->height, 8) / 4))
		return -EINVAL;
	ret = camss_format_pd10_layout(pix->width, pix->height, &layout);
	if (ret)
		return ret;
	return pix->plane_fmt[0].bytesperline == layout.stride &&
	       pix->plane_fmt[0].sizeimage >= layout.bytes ? 0 : -EINVAL;
}

int vfe_175_ds4_validate_buffer(const struct vfe_output *output,
			      const struct camss_buffer *buffer)
{
	const struct v4l2_pix_format_mplane *pix = &output->video_out.active_fmt.fmt.pix_mp;
	u64 address = buffer->addr[0], size = pix->plane_fmt[0].sizeimage;
	int ret = vfe_175_ds4_validate_output(output);

	if (ret)
		return ret;
	if (address > U32_MAX || size - 1 > U32_MAX - address)
		return -ERANGE;
	return IS_ALIGNED(address, 16) ? 0 : -EINVAL;
}

void vfe_175_ds4_start(struct vfe_device *vfe, struct vfe_output *output)
{
	const struct v4l2_pix_format_mplane *pix = &output->video_out.active_fmt.fmt.pix_mp;
	const struct v4l2_pix_format_mplane *full =
		&output->line->output[0].video_out.active_fmt.fmt.pix_mp;
	u32 height = pix->height / 2, stride = pix->plane_fmt[0].bytesperline;
	struct camss_pd10_layout layout;
	u32 value;

	/* Buffer ownership is established before arming the address FIFO. */
	if (WARN_ON(camss_format_pd10_layout(pix->width, pix->height, &layout)))
		return;
	ds4_write_filter(vfe, output->line);
	ds4_update_bits(vfe, 0xd84, 0x3fff3fff, full->height - 1);
	ds4_update_bits(vfe, 0xd88, 0x1fff1fff, full->width - 1);
	ds4_update_bits(vfe, 0xda4, 0x3fff3fff, full->height / 2 - 1);
	ds4_update_bits(vfe, 0xda8, 0x1fff1fff, full->width / 2 - 1);
	ds4_update_bits(vfe, 0xd98, 0x3ff003ff, 1023U << 20);
	ds4_update_bits(vfe, 0xd9c, 0x3f, 6);
	ds4_update_bits(vfe, 0xdb8, 0x3ff003ff, 1023U << 20);
	ds4_update_bits(vfe, 0xdbc, 0x3f, 7);
	/* The node aggregator owns local crop, round and clamp enables. */
	ds4_update_bits(vfe, 0xd8c, 0xe00, 0xe00);
	ds4_update_bits(vfe, 0xdac, 0xe00, 0xe00);
	ds4_update_bits(vfe, 0x200c, BIT(5), BIT(5));
	ds4_update_bits(vfe, 0x2084, BIT(5), BIT(5));
	/* Independent composite group: starving DS4 cannot hold up FULL. */
	ds4_update_bits(vfe, 0x2014, BIT(5), BIT(5));
	writel_relaxed(0, vfe->base + 0x2718);
	writel_relaxed(ALIGN(pix->width * 4, 16), vfe->base + 0x271c);
	writel_relaxed(height, vfe->base + 0x2720);
	writel_relaxed(0, vfe->base + 0x2724);
	writel_relaxed(stride, vfe->base + 0x2728);
	writel_relaxed(layout.bytes, vfe->base + 0x2758);
	writel_relaxed(0xf, vfe->base + 0x275c);
	writel_relaxed(0, vfe->base + 0x2748);
	writel_relaxed(1, vfe->base + 0x274c);
	writel_relaxed(0, vfe->base + 0x2750);
	writel_relaxed(1, vfe->base + 0x2754);
	ds4_update_bits(vfe, 0x2744, BIT(0), 0);
	wmb();
	/* Disable unused DS4 postcrop enables, preserve FULL and DS16. */
	value = readl_relaxed(vfe->base + 0x4c);
	writel_relaxed((value & ~0xa0U) | 0x5500, vfe->base + 0x4c);
	writel_relaxed(1, vfe->base + 0x2708);
	vfe->pixel175.ds4_running = true;
}

void vfe_175_ds4_update(struct vfe_device *vfe, struct vfe_output *output,
			const struct camss_buffer *buffer)
{
	writel_relaxed(buffer->addr[0], vfe->base + 0x2714);
}

void vfe_175_ds4_stop(struct vfe_device *vfe, struct vfe_output *output)
{
	if (!ds4_supported(vfe))
		return;
	writel_relaxed(0, vfe->base + 0x2708);
	readl_relaxed(vfe->base + 0x2708);
	ds4_update_bits(vfe, 0x2014, BIT(5), 0);
	ds4_update_bits(vfe, 0x4c, 0x5500, 0);
	ds4_update_bits(vfe, 0xd8c, 0xe00, 0);
	ds4_update_bits(vfe, 0xdac, 0xe00, 0);
	vfe->pixel175.ds4_running = false;
}
