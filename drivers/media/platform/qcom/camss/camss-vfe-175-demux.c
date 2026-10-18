// SPDX-License-Identifier: GPL-2.0-only
/* VFE175 decoded Bayer routing and gain/black-level configuration. */

#include <linux/bitfield.h>
#include <linux/io.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-camif.h"
#include "camss-vfe-175-demux.h"

#define VFE_175_LENS_ENABLE		0x040
#define VFE_175_LENS_DEMUX		BIT(2)
#define VFE_175_DEMUX_CFG			0x560
#define VFE_175_DEMUX_PERIOD		GENMASK(3, 0)
#define VFE_175_DEMUX_BLACK_IN		GENMASK(15, 4)
#define VFE_175_DEMUX_BLACK_OUT		GENMASK(31, 20)
#define VFE_175_DEMUX_GAIN0		0x564
#define VFE_175_DEMUX_GAIN1		0x568
#define VFE_175_DEMUX_RIGHT_GAIN0		0x56c
#define VFE_175_DEMUX_RIGHT_GAIN1		0x570
#define VFE_175_DEMUX_EVEN		0x574
#define VFE_175_DEMUX_ODD			0x578
#define VFE_175_DEMUX_GAIN_LOW		GENMASK(14, 0)
#define VFE_175_DEMUX_GAIN_HIGH		GENMASK(30, 16)

static bool vfe_175_demux_supported(struct vfe_device *vfe)
{
	return !vfe->res->is_lite && vfe->camss->res->version == CAMSS_8150;
}

int vfe_175_demux_from_code(u32 code, struct vfe_175_demux_config *config)
{
	int pattern;

	if (!config)
		return -EINVAL;
	pattern = vfe_175_bayer_pattern(code);
	if (pattern < 0)
		return pattern;
	/* CSID delivers decoded pixels; do not infer an extra gain from RAW bpp. */
	*config = (struct vfe_175_demux_config) {
		.gr_gain = 1024,
		.gb_gain = 1024,
		.r_gain = 1024,
		.b_gain = 1024,
		.pattern = pattern,
		.enabled = true,
	};

	return 0;
}

static void vfe_175_demux_write(struct vfe_device *vfe, u32 reg, u32 mask, u32 data)
{
	u32 value = readl_relaxed(vfe->base + reg);

	writel_relaxed((value & ~mask) | data, vfe->base + reg);
}

int vfe_175_demux_configure(struct vfe_device *vfe,
			   const struct vfe_175_demux_config *config)
{
	static const u8 even[4] = { 0xc9, 0x9c, 0xca, 0xac };
	static const u8 odd[4] = { 0xac, 0xca, 0x9c, 0xc9 };
	u32 green_even, green_odd, gain0, gain1, mask, value;

	if (!vfe_175_demux_supported(vfe) || !config || config->pattern > 3 ||
	    config->gr_gain > 32767 || config->gb_gain > 32767 ||
	    config->r_gain > 32767 || config->b_gain > 32767 ||
	    config->black_in > 4095 || config->black_out > 4095)
		return -EINVAL;

	green_even = config->pattern < 2 ? config->gr_gain : config->gb_gain;
	green_odd = config->pattern < 2 ? config->gb_gain : config->gr_gain;
	gain0 = FIELD_PREP(VFE_175_DEMUX_GAIN_LOW, green_even) |
		FIELD_PREP(VFE_175_DEMUX_GAIN_HIGH, green_odd);
	gain1 = FIELD_PREP(VFE_175_DEMUX_GAIN_LOW, config->b_gain) |
		FIELD_PREP(VFE_175_DEMUX_GAIN_HIGH, config->r_gain);
	mask = VFE_175_DEMUX_PERIOD | VFE_175_DEMUX_BLACK_IN | VFE_175_DEMUX_BLACK_OUT;
	value = FIELD_PREP(VFE_175_DEMUX_PERIOD, 1) |
		FIELD_PREP(VFE_175_DEMUX_BLACK_IN, config->black_in) |
		FIELD_PREP(VFE_175_DEMUX_BLACK_OUT, config->black_out);
	vfe_175_demux_write(vfe, VFE_175_DEMUX_CFG, mask, value);
	mask = VFE_175_DEMUX_GAIN_LOW | VFE_175_DEMUX_GAIN_HIGH;
	/* Single IFE: both stripe halves start with the same channel calibration. */
	vfe_175_demux_write(vfe, VFE_175_DEMUX_GAIN0, mask, gain0);
	vfe_175_demux_write(vfe, VFE_175_DEMUX_GAIN1, mask, gain1);
	vfe_175_demux_write(vfe, VFE_175_DEMUX_RIGHT_GAIN0, mask, gain0);
	vfe_175_demux_write(vfe, VFE_175_DEMUX_RIGHT_GAIN1, mask, gain1);
	writel_relaxed(even[config->pattern], vfe->base + VFE_175_DEMUX_EVEN);
	writel_relaxed(odd[config->pattern], vfe->base + VFE_175_DEMUX_ODD);
	/* Publish routing and gains before enabling Demux; output owns PIX RUP. */
	wmb();
	vfe_175_demux_write(vfe, VFE_175_LENS_ENABLE, VFE_175_LENS_DEMUX,
			    config->enabled ? VFE_175_LENS_DEMUX : 0);

	return 0;
}

void vfe_175_demux_stop(struct vfe_device *vfe)
{
	if (!vfe_175_demux_supported(vfe))
		return;
	vfe_175_demux_write(vfe, VFE_175_LENS_ENABLE, VFE_175_LENS_DEMUX, 0);
}
