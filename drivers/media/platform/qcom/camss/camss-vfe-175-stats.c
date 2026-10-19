// SPDX-License-Identifier: GPL-2.0-only
/* Qualcomm SM8150 VFE175 Bayer histogram and RGB grid statistics. */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/qcom-camss-config.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-stats.h"

#define VFE_175_STATS_ENABLE             0x044
#define VFE_175_BHIST_ENABLE             BIT(4)
#define VFE_175_BG_ENABLE                BIT(3)
#define VFE_175_BG_QUAD_SYNC             BIT(6)
#define VFE_175_BG_OFFSET                0xbc8
#define VFE_175_BG_REGIONS               0xbcc
#define VFE_175_BG_SIZE                  0xbd0
#define VFE_175_BG_HIGH_0                0xbd4
#define VFE_175_BG_HIGH_1                0xbd8
#define VFE_175_BG_LOW_0                 0xbdc
#define VFE_175_BG_LOW_1                 0xbe0
#define VFE_175_BG_Y                     0xc14
#define VFE_175_BG_CONFIG                0xcb0
#define VFE_175_BG_CLIENT                15
#define VFE_175_BG_BYTES                 (CAMSS_STATS_BG_MAX_REGIONS * 24)
#define VFE_175_BG_THRESHOLD             16383
#define VFE_175_STATS_MASK               (BIT(15) | BIT(16))
#define VFE_175_STATS_CGC                0x030
#define VFE_175_DMI_CONFIG               0xc24
#define VFE_175_DMI_ADDRESS              0xc28
#define VFE_175_DMI_DATA_LOW             0xc30
#define VFE_175_DMI_RAM_SELECT           GENMASK(6, 0)
#define VFE_175_DMI_AUTO_INCREMENT       BIT(8)
#define VFE_175_DMI_AUTO_LOAD            BIT(12)
#define VFE_175_DMI_BHIST_RAM            0x3a
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

void vfe_175_stats_init_output(struct vfe_output *output)
{
	const u8 clients[] = { VFE_175_BHIST_CLIENT, VFE_175_BG_CLIENT };

	vfe_add_output(output, MSM_VFE_PAD_SRC_STATS, VFE_175_BHIST_GROUP,
		       ARRAY_SIZE(clients), clients);
	/* Each hardware destination remains 256-byte aligned. */
	output->dma_size = VFE_175_BHIST_BYTES + VFE_175_BG_BYTES;
	output->stats_payload_size = CAMSS_STATS_MAX_PAYLOAD;
}

static struct vfe_output *vfe_175_stats_output(struct vfe_line *line)
{
	unsigned int i;

	for (i = 1; i < line->num_outputs; i++)
		if (line->output[i].pad == MSM_VFE_PAD_SRC_STATS)
			return &line->output[i];

	return NULL;
}

static void vfe_175_stats_write_mask(struct vfe_device *vfe, u32 reg,
				      u32 mask, u32 value)
{
	u32 old = readl_relaxed(vfe->base + reg);

	writel_relaxed((old & ~mask) | value, vfe->base + reg);
}

static void vfe_175_bg_write(struct vfe_device *vfe)
{
	const struct vfe_175_bg_state *bg = &vfe->pixel175.stats.bg;
	u32 thresholds = VFE_175_BG_THRESHOLD | (VFE_175_BG_THRESHOLD << 16);
	u32 mask = GENMASK(13, 0) | GENMASK(29, 16);

	vfe_175_stats_write_mask(vfe, VFE_175_BG_OFFSET,
		GENMASK(12, 0) | GENMASK(29, 16), bg->left | (bg->top << 16));
	vfe_175_stats_write_mask(vfe, VFE_175_BG_REGIONS,
		GENMASK(7, 0) | GENMASK(22, 16),
		(bg->columns - 1) | ((bg->rows - 1) << 16));
	vfe_175_stats_write_mask(vfe, VFE_175_BG_SIZE,
		GENMASK(8, 0) | GENMASK(25, 16),
		(bg->width - 1) | ((bg->height - 1) << 16));
	vfe_175_stats_write_mask(vfe, VFE_175_BG_HIGH_0, mask, thresholds);
	vfe_175_stats_write_mask(vfe, VFE_175_BG_HIGH_1, mask, thresholds);
	vfe_175_stats_write_mask(vfe, VFE_175_BG_LOW_0, mask, 0);
	vfe_175_stats_write_mask(vfe, VFE_175_BG_LOW_1, mask, 0);
	/* Regular RGB records: disable Y and saturation extension, shift zero. */
	vfe_175_stats_write_mask(vfe, VFE_175_BG_Y, BIT(0), 0);
	vfe_175_stats_write_mask(vfe, VFE_175_BG_CONFIG,
		GENMASK(3, 0) | GENMASK(23, 8), GENMASK(23, 8));
	vfe_175_stats_write_mask(vfe, VFE_175_STATS_CONFIG,
		VFE_175_BG_QUAD_SYNC, VFE_175_BG_QUAD_SYNC);
}

void vfe_175_stats_fill(struct vfe_device *vfe, struct vfe_output *output,
			struct v4l2_isp_buffer *stats)
{
	const struct vfe_175_bg_state *bg = &vfe->pixel175.stats.bg;
	u32 count = bg->columns * bg->rows;
	struct v4l2_isp_stats_block_type_info info[] = {
		[CAMSS_STATS_AEC_BHIST] = { .size = sizeof(struct camss_stats_bhist) },
		[CAMSS_STATS_AWB_BG] = {
			.size = sizeof(struct camss_stats_awb_bg) +
				count * sizeof(struct camss_stats_bg_region),
		},
	};
	const __le32 *raw = output->dma_vaddr + VFE_175_BHIST_BYTES;
	struct v4l2_isp_block_header *header;
	struct camss_stats_bhist *bhist;
	struct camss_stats_awb_bg *grid;
	unsigned int i;

	header = v4l2_isp_stats_init_block(vfe->camss->dev, stats, info,
		ARRAY_SIZE(info), CAMSS_STATS_AEC_BHIST, output->stats_payload_size);
	if (IS_ERR(header))
		return;
	bhist = container_of(header, struct camss_stats_bhist, header);
	memcpy(bhist->bins, output->dma_vaddr, sizeof(bhist->bins));

	header = v4l2_isp_stats_init_block(vfe->camss->dev, stats, info,
		ARRAY_SIZE(info), CAMSS_STATS_AWB_BG, output->stats_payload_size);
	if (IS_ERR(header))
		return;
	grid = container_of(header, struct camss_stats_awb_bg, header);
	grid->left = bg->left;
	grid->top = bg->top;
	grid->region_width = bg->width;
	grid->region_height = bg->height;
	grid->columns = bg->columns;
	grid->rows = bg->rows;
	grid->high_threshold = VFE_175_BG_THRESHOLD;
	grid->input_bits = 14;
	/* Six little-endian words per regular hardware region; omit reserved bits. */
	for (i = 0; i < count; i++, raw += 6) {
		struct camss_stats_bg_region *r = &grid->regions[i];

		r->r_sum = le32_to_cpu(raw[0]) & GENMASK(29, 0);
		r->b_sum = le32_to_cpu(raw[1]) & GENMASK(29, 0);
		r->gr_sum = le32_to_cpu(raw[2]) & GENMASK(29, 0);
		r->gb_sum = le32_to_cpu(raw[3]) & GENMASK(29, 0);
		r->r_count = le32_to_cpu(raw[4]);
		r->b_count = le32_to_cpu(raw[4]) >> 16;
		r->gr_count = le32_to_cpu(raw[5]);
		r->gb_count = le32_to_cpu(raw[5]) >> 16;
	}
}

/* The register reset does not initialize the histogram counter SRAM. */
static void vfe_175_bhist_clear(struct vfe_device *vfe)
{
	u32 cgc = readl_relaxed(vfe->base + VFE_175_STATS_CGC);
	u32 config = readl_relaxed(vfe->base + VFE_175_DMI_CONFIG);
	u32 address = readl_relaxed(vfe->base + VFE_175_DMI_ADDRESS);
	u32 select;
	unsigned int i;

	/* Pre-stream setup owns DMI; keep the SRAM clock on during CPU access. */
	writel(cgc | VFE_175_BHIST_ENABLE, vfe->base + VFE_175_STATS_CGC);
	select = config & ~(VFE_175_DMI_RAM_SELECT |
			    VFE_175_DMI_AUTO_INCREMENT | VFE_175_DMI_AUTO_LOAD);
	writel(select | VFE_175_DMI_BHIST_RAM, vfe->base + VFE_175_DMI_CONFIG);
	for (i = 0; i < CAMSS_STATS_BHIST_BINS; i++) {
		/* Explicit indices avoid inheriting another DMI user's mode. */
		writel(i, vfe->base + VFE_175_DMI_ADDRESS);
		writel(0, vfe->base + VFE_175_DMI_DATA_LOW);
	}
	/* Complete SRAM writes before restoring shared state and clock policy. */
	readl(vfe->base + VFE_175_DMI_CONFIG);
	writel(config, vfe->base + VFE_175_DMI_CONFIG);
	writel(address, vfe->base + VFE_175_DMI_ADDRESS);
	writel(cgc, vfe->base + VFE_175_STATS_CGC);
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
	if (output->pad != MSM_VFE_PAD_SRC_STATS || output->wm_num != 2 ||
	    output->wm[0].bus_client != VFE_175_BHIST_CLIENT ||
	    output->wm[0].plane ||
	    output->wm[1].bus_client != VFE_175_BG_CLIENT ||
	    output->wm[1].plane != 1 || output->comp_group != VFE_175_BHIST_GROUP ||
	    output->dma_size != VFE_175_BHIST_BYTES + VFE_175_BG_BYTES ||
	    !output->dma_vaddr)
		return -EINVAL;
	if (output->dma_addr > U32_MAX ||
	    output->dma_size - 1 > U32_MAX - output->dma_addr ||
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
	struct vfe_175_bg_state *bg;
	bool valid = width >= 6 && height >= 2 &&
		     width <= VFE_175_BHIST_MAX_WIDTH &&
		     height <= VFE_175_BHIST_MAX_HEIGHT;

	if (output && vb2_is_streaming(&output->video_out.vb2_q) && !valid)
		return -EINVAL;
	vfe->pixel175.stats = (struct vfe_175_stats_state) {
		.width = valid ? width : 0,
		.height = valid ? height : 0,
		.enabled = valid,
	};
	if (valid) {
		bg = &vfe->pixel175.stats.bg;
		bg->columns = min_t(u32, CAMSS_STATS_BG_COLUMNS, width / 6);
		bg->rows = min_t(u32, CAMSS_STATS_BG_ROWS, height / 2);
		bg->width = min_t(u32, (width / bg->columns) & ~1U, 390);
		bg->height = min_t(u32, (height / bg->rows) & ~1U, 512);
		bg->left = ((width - bg->columns * bg->width) / 2) & ~1U;
		bg->top = ((height - bg->rows * bg->height) / 2) & ~1U;
		vfe_175_bg_write(vfe);
		vfe_175_bhist_clear(vfe);
		vfe_175_bhist_write(vfe, line);
	}

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
	u32 value;
	unsigned int i;

	memset(output->dma_vaddr, 0, output->dma_size);
	value = readl_relaxed(vfe->base + VFE_175_BUS_CGC);
	writel_relaxed(value | VFE_175_STATS_MASK, vfe->base + VFE_175_BUS_CGC);
	value = readl_relaxed(vfe->base + VFE_175_BUS_NO_SYNC);
	writel_relaxed(value | VFE_175_STATS_MASK, vfe->base + VFE_175_BUS_NO_SYNC);
	value = readl_relaxed(vfe->base + VFE_175_BUS_COMP(output->comp_group));
	writel_relaxed(value | VFE_175_STATS_MASK,
		       vfe->base + VFE_175_BUS_COMP(output->comp_group));
	for (i = 0; i < output->wm_num; i++) {
		u32 wm = output->wm[i].bus_client;
		u32 bytes = i ? VFE_175_BG_BYTES : VFE_175_BHIST_BYTES;

		/* Self-sized PLAIN64 statistics packets, not strided images. */
		writel_relaxed(0, vfe->base + VFE_175_WM(wm, 0x18));
		writel_relaxed(0, vfe->base + VFE_175_WM(wm, 0x1c));
		writel_relaxed(0, vfe->base + VFE_175_WM(wm, 0x20));
		writel_relaxed(0xa, vfe->base + VFE_175_WM(wm, 0x24));
		writel_relaxed(1, vfe->base + VFE_175_WM(wm, 0x28));
		writel_relaxed(0, vfe->base + VFE_175_WM(wm, 0x48));
		writel_relaxed(1, vfe->base + VFE_175_WM(wm, 0x4c));
		writel_relaxed(0, vfe->base + VFE_175_WM(wm, 0x50));
		writel_relaxed(1, vfe->base + VFE_175_WM(wm, 0x54));
		writel_relaxed(bytes, vfe->base + VFE_175_WM(wm, 0x58));
		writel_relaxed(0xf, vfe->base + VFE_175_WM(wm, 0x5c));
		writel_relaxed(3, vfe->base + VFE_175_WM(wm, 0x08));
	}
	vfe_175_bhist_write(vfe, output->line);
	vfe_175_stats_write_mask(vfe, VFE_175_STATS_ENABLE,
		VFE_175_BG_ENABLE, VFE_175_BG_ENABLE);
}

void vfe_175_stats_update(struct vfe_device *vfe, struct vfe_output *output)
{
	unsigned int i;

	for (i = 0; i < output->wm_num; i++)
		writel_relaxed(output->dma_addr + (i ? VFE_175_BHIST_BYTES : 0),
			       vfe->base + VFE_175_WM(output->wm[i].bus_client, 0x14));
}

void vfe_175_stats_stop(struct vfe_device *vfe, struct vfe_output *output)
{
	u32 value = readl_relaxed(vfe->base + VFE_175_STATS_ENABLE);
	unsigned int i;

	writel_relaxed(value & ~(VFE_175_BHIST_ENABLE | VFE_175_BG_ENABLE),
		       vfe->base + VFE_175_STATS_ENABLE);
	for (i = 0; i < output->wm_num; i++) {
		u32 wm = output->wm[i].bus_client;

		writel_relaxed(0, vfe->base + VFE_175_WM(wm, 0x08));
		/* Complete both stops before metadata buffers can be returned. */
		readl(vfe->base + VFE_175_WM(wm, 0x08));
	}
	value = readl_relaxed(vfe->base + VFE_175_BUS_COMP(output->comp_group));
	writel_relaxed(value & ~VFE_175_STATS_MASK,
		       vfe->base + VFE_175_BUS_COMP(output->comp_group));
}
