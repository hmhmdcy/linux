// SPDX-License-Identifier: GPL-2.0-only
/* VFE175 regular mesh lens shading: no module tuning or calibration policy. */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/qcom-camss-config.h>
#include <linux/string.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-lsc.h"

#define LENS_CGC_OVERRIDE	0x02c
#define LENS_ENABLE		0x040
#define LSC_ENABLE		BIT(8)
#define LSC_CFG			0x6bc
#define LSC_BANK			BIT(16)
#define LSC_COLUMNS		GENMASK(23, 18)
#define LSC_ROWS			GENMASK(29, 24)
#define LSC_GRID0		0x6c0
#define LSC_GRID1		0x6c4
#define LSC_GRID2		0x6c8
#define LSC_RIGHT_GRID0		0x6cc
#define LSC_RIGHT_GRID1		0x6d0
#define LSC_RIGHT_GRID2		0x6d4
#define LSC_STRIPE0		0x6d8
#define LSC_STRIPE1		0x6dc
#define LSC_RIGHT_STRIPE0		0x6e0
#define LSC_RIGHT_STRIPE1		0x6e4
#define DMI_CFG			0xc24
#define DMI_ADDR			0xc28
#define DMI_DATA_LO		0xc30
#define DMI_SELECT		GENMASK(7, 0)
#define LSC_GAIN_MASK		GENMASK(12, 0)

static void lsc_enable(struct vfe_device *vfe, bool enable)
{
	u32 value = readl_relaxed(vfe->base + LENS_ENABLE);

	value &= ~LSC_ENABLE;
	if (enable)
		value |= LSC_ENABLE;
	writel_relaxed(value, vfe->base + LENS_ENABLE);
}

int vfe_175_lsc_validate(struct device *dev,
			 const struct v4l2_isp_block_header *block)
{
	const struct camss_params_lsc *lsc;
	unsigned int scale, channel, point;

	if (block->flags & ~(V4L2_ISP_PARAMS_FL_BLOCK_ENABLE |
			     V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) ||
	    block->flags == (V4L2_ISP_PARAMS_FL_BLOCK_ENABLE |
			     V4L2_ISP_PARAMS_FL_BLOCK_DISABLE))
		return -EINVAL;
	if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE)
		return 0;
	lsc = container_of(block, struct camss_params_lsc, header);
	if (lsc->subdivision > 3 || memchr_inv(lsc->_pad, 0, sizeof(lsc->_pad)))
		return -EINVAL;
	scale = 1U << lsc->subdivision;
	if (lsc->cell_width < 18 || lsc->cell_width > 2048 ||
	    lsc->cell_height < 9 || lsc->cell_height > 1024 ||
	    lsc->cell_width % scale || lsc->cell_height % scale ||
	    lsc->cell_width / scale < 9 || lsc->cell_height / scale < 9 ||
	    lsc->x_start >= (CAMSS_LSC_MESH_WIDTH - 1) * lsc->cell_width ||
	    lsc->y_start >= (CAMSS_LSC_MESH_HEIGHT - 1) * lsc->cell_height)
		return -EINVAL;
	for (channel = 0; channel < CAMSS_LSC_CHANNELS; channel++)
		for (point = 0; point < CAMSS_LSC_MESH_POINTS; point++)
			if (lsc->gains[channel][point] < 1024 ||
			    lsc->gains[channel][point] > LSC_GAIN_MASK)
				return -EINVAL;
	return 0;
}

int vfe_175_params_validate_line(struct vfe_line *line,
				 const struct v4l2_isp_buffer *params)
{
	size_t offset = 0;

	/* Common validation has already checked every block's bounds and type. */
	while (offset < params->data_size) {
		const struct v4l2_isp_block_header *block =
			(const void *)&params->data[offset];

		if (block->type == CAMSS_PARAMS_LSC &&
		    !(block->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE)) {
			const struct camss_params_lsc *lsc =
				container_of(block, struct camss_params_lsc, header);
			const struct v4l2_mbus_framefmt *format =
				&line->fmt[MSM_VFE_PAD_SINK];

			if (!format->width || !format->height ||
			    lsc->x_start + DIV_ROUND_UP(format->width, 2) >
				(CAMSS_LSC_MESH_WIDTH - 1) * lsc->cell_width ||
			    lsc->y_start + DIV_ROUND_UP(format->height, 2) >
				(CAMSS_LSC_MESH_HEIGHT - 1) * lsc->cell_height)
				return -EINVAL;
		}
		offset += block->size;
	}
	return 0;
}

void vfe_175_lsc_params(struct vfe_device *vfe,
			const struct v4l2_isp_block_header *block)
{
	const struct camss_params_lsc *lsc;
	u32 dmi_cfg, dmi_addr, cgc, config, grid0, grid1, grid2, stripe0, stripe1;
	u32 sw, sh, dx, dy, x, y, bx, by, sx, sy, px, py;
	unsigned int table, i;
	u8 bank;

	if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
		vfe->pixel175.lsc_enabled = false;
		lsc_enable(vfe, false);
		return;
	}
	lsc = container_of(block, struct camss_params_lsc, header);
	sw = lsc->cell_width >> lsc->subdivision;
	sh = lsc->cell_height >> lsc->subdivision;
	dx = BIT(20) / sw;
	dy = BIT(20) / sh;
	bx = lsc->x_start / lsc->cell_width;
	by = lsc->y_start / lsc->cell_height;
	x = lsc->x_start % lsc->cell_width;
	y = lsc->y_start % lsc->cell_height;
	sx = x / sw;
	sy = y / sh;
	px = x % sw;
	py = y % sh;
	grid0 = (lsc->cell_width - 1) | (lsc->cell_height - 1) << 16;
	grid1 = (sh - 1) | dy << 12 | (u32)lsc->subdivision << 30;
	grid2 = (sw - 1) | dx << 12;
	stripe0 = bx | by << 6 | (dy * py) << 12;
	stripe1 = px | py << 12 | sx << 24 | sy << 28;
	bank = !vfe->pixel175.lsc_bank;
	dmi_cfg = readl_relaxed(vfe->base + DMI_CFG);
	dmi_addr = readl_relaxed(vfe->base + DMI_ADDR);
	cgc = readl_relaxed(vfe->base + LENS_CGC_OVERRIDE);
	writel(cgc | LSC_ENABLE, vfe->base + LENS_CGC_OVERRIDE);
	/* output_lock serializes parameters; Gamma only uses DMI before PIX starts. */
	for (table = 0; table < 2; table++) {
		u32 selector = (bank ? 18 : 14) + table;

		writel((dmi_cfg & ~DMI_SELECT) | selector, vfe->base + DMI_CFG);
		for (i = 0; i < CAMSS_LSC_MESH_POINTS; i++) {
			u32 value;

			if (!table)
				value = lsc->gains[0][i] | (u32)lsc->gains[1][i] << 13;
			else
				value = lsc->gains[3][i] | (u32)lsc->gains[2][i] << 13;
			writel(i, vfe->base + DMI_ADDR);
			writel(value, vfe->base + DMI_DATA_LO);
		}
	}
	writel(dmi_cfg, vfe->base + DMI_CFG);
	writel(dmi_addr, vfe->base + DMI_ADDR);
	wmb();
	config = FIELD_PREP(LSC_COLUMNS, CAMSS_LSC_MESH_WIDTH - 2) |
		 FIELD_PREP(LSC_ROWS, CAMSS_LSC_MESH_HEIGHT - 2) |
		 (bank ? LSC_BANK : 0);
	writel_relaxed(config, vfe->base + LSC_CFG);
	writel_relaxed(grid0, vfe->base + LSC_GRID0);
	writel_relaxed(grid1, vfe->base + LSC_GRID1);
	writel_relaxed(grid2, vfe->base + LSC_GRID2);
	writel_relaxed(grid0, vfe->base + LSC_RIGHT_GRID0);
	writel_relaxed(grid1, vfe->base + LSC_RIGHT_GRID1);
	writel_relaxed(grid2, vfe->base + LSC_RIGHT_GRID2);
	writel_relaxed(stripe0, vfe->base + LSC_STRIPE0);
	writel_relaxed(stripe1, vfe->base + LSC_STRIPE1);
	writel_relaxed(stripe0, vfe->base + LSC_RIGHT_STRIPE0);
	writel_relaxed(stripe1, vfe->base + LSC_RIGHT_STRIPE1);
	wmb();
	writel(cgc, vfe->base + LENS_CGC_OVERRIDE);
	vfe->pixel175.lsc_bank = bank;
	if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_ENABLE)
		vfe->pixel175.lsc_enabled = true;
	lsc_enable(vfe, vfe->pixel175.lsc_enabled);
}

void vfe_175_lsc_init(struct vfe_device *vfe)
{
	vfe->pixel175.lsc_bank = !!(readl_relaxed(vfe->base + LSC_CFG) & LSC_BANK);
	vfe_175_lsc_stop(vfe);
}

void vfe_175_lsc_stop(struct vfe_device *vfe)
{
	vfe->pixel175.lsc_enabled = false;
	lsc_enable(vfe, false);
}
