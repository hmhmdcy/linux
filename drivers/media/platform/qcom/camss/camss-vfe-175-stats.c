// SPDX-License-Identifier: GPL-2.0-only
/* Qualcomm SM8150 VFE175 linear luma Bayer histogram. */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/qcom-camss-config.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-stats.h"

#define VFE_175_STATS_ENABLE             0x044
#define VFE_175_BHIST_ENABLE             BIT(5)
#define VFE_175_STATS_CONFIG             0xaac
#define VFE_175_BHIST_UNIFORM            BIT(4)
#define VFE_175_BHIST_CHANNEL            GENMASK(23, 21)
#define VFE_175_BHIST_OFFSET             0xbe4
#define VFE_175_BHIST_REGIONS            0xbe8
#define VFE_175_BHIST_MAX_WIDTH          8192
#define VFE_175_BHIST_MAX_HEIGHT         16384
#define VFE_175_BHIST_CLIENT             16
#define VFE_175_BHIST_GROUP              1
#define VFE_175_BHIST_BYTES              (CAMSS_STATS_BHIST_BINS * sizeof(u32))
#define VFE_175_BUS_CGC                  0x200c
#define VFE_175_BUS_COMP(n)              (0x2010 + (n) * 4)
#define VFE_175_BUS_NO_SYNC              0x2084
#define VFE_175_WM(n, r)                 (0x2200 + (n) * 0x100 + (r))

static struct vfe_output *vfe_175_stats_output(struct vfe_line *line)
{
	unsigned int i;

	for (i = 1; i < line->num_outputs; i++)
		if (line->output[i].pad == MSM_VFE_PAD_SRC_STATS)
			return &line->output[i];

	return NULL;
}

static void vfe_175_bhist_write(struct vfe_device *vfe, struct vfe_line *line)
{
	const struct vfe_175_stats_state *s = &vfe->pixel175.stats;
	struct vfe_output *output = vfe_175_stats_output(line);
	u32 value;

	/* Region offsets are pixels; sizes count 2x2 quads minus one. */
	value = readl_relaxed(vfe->base + VFE_175_BHIST_OFFSET);
	value &= ~(GENMASK(12, 0) | GENMASK(29, 16));
	value |= s->left | ((u32)s->top << 16);
	writel_relaxed(value, vfe->base + VFE_175_BHIST_OFFSET);
	value = readl_relaxed(vfe->base + VFE_175_BHIST_REGIONS);
	value &= ~(GENMASK(11, 0) | GENMASK(28, 16));
	value |= (s->width / 2 - 1) | ((s->height / 2 - 1) << 16);
	writel_relaxed(value, vfe->base + VFE_175_BHIST_REGIONS);
	value = readl_relaxed(vfe->base + VFE_175_STATS_CONFIG);
	/* Channel zero is Y. Uniform bins shift the 14-bit data down to 10. */
	value = (value & ~VFE_175_BHIST_CHANNEL) | VFE_175_BHIST_UNIFORM;
	writel_relaxed(value, vfe->base + VFE_175_STATS_CONFIG);
	value = readl_relaxed(vfe->base + VFE_175_STATS_ENABLE);
	value &= ~VFE_175_BHIST_ENABLE;
	if (s->enabled && output && output->state == VFE_OUTPUT_ON)
		value |= VFE_175_BHIST_ENABLE;
	writel_relaxed(value, vfe->base + VFE_175_STATS_ENABLE);
}

int vfe_175_bhist_validate(struct device *dev,
			   const struct v4l2_isp_block_header *block)
{
	const struct camss_params_aec_bhist *r;

	if (block->flags & ~(V4L2_ISP_PARAMS_FL_BLOCK_ENABLE |
			     V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) ||
	    block->flags == (V4L2_ISP_PARAMS_FL_BLOCK_ENABLE |
			     V4L2_ISP_PARAMS_FL_BLOCK_DISABLE))
		return -EINVAL;
	if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE)
		return 0;

	r = container_of(block, struct camss_params_aec_bhist, header);
	/* Horizontal region count has a minimum of two quads on this IFE. */
	if ((r->left | r->top | r->width | r->height) & 1 ||
	    r->width < 4 || r->height < 2 ||
	    r->left >= VFE_175_BHIST_MAX_WIDTH ||
	    r->top >= VFE_175_BHIST_MAX_HEIGHT ||
	    r->width > VFE_175_BHIST_MAX_WIDTH ||
	    r->height > VFE_175_BHIST_MAX_HEIGHT)
		return -EINVAL;

	return 0;
}

int vfe_175_stats_validate_output(const struct vfe_output *output)
{
	if (output->pad != MSM_VFE_PAD_SRC_STATS || output->wm_num != 1 ||
	    output->wm[0].bus_client != VFE_175_BHIST_CLIENT ||
	    output->wm[0].plane || output->comp_group != VFE_175_BHIST_GROUP ||
	    output->dma_size != VFE_175_BHIST_BYTES || !output->dma_vaddr)
		return -EINVAL;
	if (output->dma_addr > U32_MAX ||
	    VFE_175_BHIST_BYTES - 1 > U32_MAX - output->dma_addr ||
	    !IS_ALIGNED(output->dma_addr, 256))
		return -ERANGE;

	return 0;
}

int vfe_175_stats_prepare(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	struct vfe_output *output = vfe_175_stats_output(line);
	u32 width = line->fmt[MSM_VFE_PAD_SINK].width & ~1U;
	u32 height = line->fmt[MSM_VFE_PAD_SINK].height & ~1U;
	bool valid = width >= 4 && height >= 2 &&
		     width <= VFE_175_BHIST_MAX_WIDTH &&
		     height <= VFE_175_BHIST_MAX_HEIGHT;

	if (output && vb2_is_streaming(&output->video_out.vb2_q) && !valid)
		return -EINVAL;
	vfe->pixel175.stats = (struct vfe_175_stats_state) {
		.width = valid ? width : 0,
		.height = valid ? height : 0,
		.enabled = valid,
	};
	if (valid)
		vfe_175_bhist_write(vfe, line);

	return 0;
}

void vfe_175_stats_params(struct vfe_device *vfe, struct vfe_line *line,
			  const struct v4l2_isp_block_header *block)
{
	struct vfe_175_stats_state *s = &vfe->pixel175.stats;
	u32 w = line->fmt[MSM_VFE_PAD_SINK].width & ~1U;
	u32 h = line->fmt[MSM_VFE_PAD_SINK].height & ~1U;
	const struct camss_params_aec_bhist *r;

	if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
		s->enabled = false;
	} else {
		if (w < 4 || h < 2)
			return;
		r = container_of(block, struct camss_params_aec_bhist, header);
		s->left = min_t(u32, r->left, w - 4);
		s->top = min_t(u32, r->top, h - 2);
		s->width = min_t(u32, r->width, w - s->left);
		s->height = min_t(u32, r->height, h - s->top);
		if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_ENABLE)
			s->enabled = true;
	}
	if (s->width && s->height)
		vfe_175_bhist_write(vfe, line);
}

void vfe_175_stats_start(struct vfe_device *vfe, struct vfe_output *output)
{
	u32 wm = output->wm[0].bus_client;
	u32 value;

	memset(output->dma_vaddr, 0, output->dma_size);
	value = readl_relaxed(vfe->base + VFE_175_BUS_CGC);
	writel_relaxed(value | BIT(wm), vfe->base + VFE_175_BUS_CGC);
	value = readl_relaxed(vfe->base + VFE_175_BUS_NO_SYNC);
	writel_relaxed(value | BIT(wm), vfe->base + VFE_175_BUS_NO_SYNC);
	value = readl_relaxed(vfe->base + VFE_175_BUS_COMP(output->comp_group));
	writel_relaxed(value | BIT(wm),
		       vfe->base + VFE_175_BUS_COMP(output->comp_group));
	/* Statistics are a self-sized PLAIN64 packet, not a strided image. */
	writel_relaxed(0, vfe->base + VFE_175_WM(wm, 0x18));
	writel_relaxed(0, vfe->base + VFE_175_WM(wm, 0x1c));
	writel_relaxed(0, vfe->base + VFE_175_WM(wm, 0x20));
	writel_relaxed(0xa, vfe->base + VFE_175_WM(wm, 0x24));
	writel_relaxed(1, vfe->base + VFE_175_WM(wm, 0x28));
	writel_relaxed(0, vfe->base + VFE_175_WM(wm, 0x48));
	writel_relaxed(1, vfe->base + VFE_175_WM(wm, 0x4c));
	writel_relaxed(0, vfe->base + VFE_175_WM(wm, 0x50));
	writel_relaxed(1, vfe->base + VFE_175_WM(wm, 0x54));
	writel_relaxed(VFE_175_BHIST_BYTES, vfe->base + VFE_175_WM(wm, 0x58));
	writel_relaxed(0xf, vfe->base + VFE_175_WM(wm, 0x5c));
	writel_relaxed(3, vfe->base + VFE_175_WM(wm, 0x08));
	vfe_175_bhist_write(vfe, output->line);
}

void vfe_175_stats_update(struct vfe_device *vfe, struct vfe_output *output)
{
	writel_relaxed(output->dma_addr,
		       vfe->base + VFE_175_WM(output->wm[0].bus_client, 0x14));
}

void vfe_175_stats_stop(struct vfe_device *vfe, struct vfe_output *output)
{
	u32 value = readl_relaxed(vfe->base + VFE_175_STATS_ENABLE);
	u32 wm = output->wm[0].bus_client;

	writel_relaxed(value & ~VFE_175_BHIST_ENABLE,
		       vfe->base + VFE_175_STATS_ENABLE);
	writel_relaxed(0, vfe->base + VFE_175_WM(wm, 0x08));
	/* Complete the stop before metadata buffers can be returned. */
	readl(vfe->base + VFE_175_WM(wm, 0x08));
	value = readl_relaxed(vfe->base + VFE_175_BUS_COMP(output->comp_group));
	writel_relaxed(value & ~BIT(wm),
		       vfe->base + VFE_175_BUS_COMP(output->comp_group));
}
