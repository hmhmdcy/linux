// SPDX-License-Identifier: GPL-2.0-only
/* VFE175 64-entry Gamma16 tables, independent of OEM tuning or OPE LUTs. */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/videodev2.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-gamma.h"

#define VFE_175_COLOR_CGC_OVERRIDE	0x034
#define VFE_175_COLOR_ENABLE		0x048
#define VFE_175_COLOR_GAMMA		BIT(3)
#define VFE_175_RGB_LUT_CFG		0x798
#define VFE_175_RGB_LUT_BANKS		GENMASK(2, 0)
#define VFE_175_DMI_CFG			0xc24
#define VFE_175_DMI_ADDR			0xc28
#define VFE_175_DMI_DATA_LO		0xc30
#define VFE_175_DMI_SELECT		GENMASK(7, 0)
#define VFE_175_GAMMA_BASE		GENMASK(9, 0)
#define VFE_175_GAMMA_DELTA		GENMASK(19, 10)

/* Standard transfer functions sampled at x=i/64, rounded to 10-bit knots. */
static const u16 vfe_175_gamma_srgb[VFE_175_GAMMA_POINTS] = {
	0, 135, 199, 246, 284, 317, 347, 373,
	398, 421, 442, 462, 481, 500, 517, 534,
	550, 565, 580, 595, 609, 623, 636, 649,
	662, 674, 686, 698, 709, 720, 732, 742,
	753, 763, 774, 784, 794, 803, 813, 823,
	832, 841, 850, 859, 868, 877, 885, 894,
	902, 910, 918, 926, 934, 942, 950, 958,
	966, 973, 981, 988, 995, 1003, 1010, 1017,
	1024,
};

static const u16 vfe_175_gamma_709[VFE_175_GAMMA_POINTS] = {
	0, 72, 135, 183, 222, 256, 286, 314,
	340, 364, 387, 408, 428, 448, 467, 484,
	502, 518, 535, 550, 565, 580, 595, 609,
	622, 636, 649, 662, 674, 687, 699, 711,
	722, 734, 745, 756, 767, 778, 789, 799,
	809, 820, 830, 840, 849, 859, 869, 878,
	887, 897, 906, 915, 924, 932, 941, 950,
	958, 967, 975, 984, 992, 1000, 1008, 1016,
	1024,
};

static bool vfe_175_gamma_supported(struct vfe_device *vfe)
{
	return !vfe->res->is_lite && vfe->camss->res->version == CAMSS_8150;
}

int vfe_175_gamma_from_format(const struct v4l2_pix_format_mplane *pix,
			      struct vfe_175_gamma_config *config)
{
	const u16 *curve = NULL;
	u32 transfer;
	unsigned int ch, i;

	if (!pix || !config || pix->pixelformat != V4L2_PIX_FMT_NV12)
		return -EINVAL;
	transfer = pix->xfer_func;
	if (transfer == V4L2_XFER_FUNC_DEFAULT)
		transfer = V4L2_MAP_XFER_FUNC_DEFAULT(pix->colorspace);
	switch (transfer) {
	case V4L2_XFER_FUNC_SRGB:
		curve = vfe_175_gamma_srgb;
		break;
	case V4L2_XFER_FUNC_709:
		curve = vfe_175_gamma_709;
		break;
	case V4L2_XFER_FUNC_NONE:
		break;
	default:
		return -EINVAL;
	}
	for (ch = 0; ch < 3; ch++)
		for (i = 0; i < VFE_175_GAMMA_POINTS; i++)
			config->points[ch][i] = curve ? curve[i] : i * 16;
	config->enabled = true;

	return 0;
}

static int vfe_175_gamma_pack(const struct vfe_175_gamma_config *config,
			    u32 packed[3][VFE_175_GAMMA_ENTRIES])
{
	unsigned int ch, i;

	if (!config)
		return -EINVAL;
	for (ch = 0; ch < 3; ch++) {
		if (config->points[ch][VFE_175_GAMMA_ENTRIES] > 1024)
			return -EINVAL;
		for (i = 0; i < VFE_175_GAMMA_ENTRIES; i++) {
			u32 base = config->points[ch][i];
			int delta = (int)config->points[ch][i + 1] - (int)base;

			/* Only the terminal knot may equal 1024; slopes are signed10. */
			if (base > 1023 || delta < -512 || delta > 511)
				return -EINVAL;
			packed[ch][i] = FIELD_PREP(VFE_175_GAMMA_BASE, base) |
				FIELD_PREP(VFE_175_GAMMA_DELTA, (u32)delta & 0x3ff);
		}
	}

	return 0;
}

int vfe_175_gamma_configure(struct vfe_device *vfe,
			   const struct vfe_175_gamma_config *config)
{
	/* LUT_CFG banks are R/G/B; DMI memory selectors are R30/G26/B28. */
	static const u8 selectors[3] = { 30, 26, 28 };
	u32 packed[3][VFE_175_GAMMA_ENTRIES];
	u32 lut_cfg, dmi_cfg, dmi_addr, cgc_override, value;
	unsigned int ch, i;
	int ret;

	if (!vfe_175_gamma_supported(vfe))
		return -EINVAL;
	ret = vfe_175_gamma_pack(config, packed);
	if (ret)
		return ret;
	if (config->enabled) {
		lut_cfg = readl_relaxed(vfe->base + VFE_175_RGB_LUT_CFG);
		dmi_cfg = readl_relaxed(vfe->base + VFE_175_DMI_CFG);
		dmi_addr = readl_relaxed(vfe->base + VFE_175_DMI_ADDR);
		/* SRAM access also needs the clock when no pixels are arriving yet. */
		cgc_override = readl_relaxed(vfe->base + VFE_175_COLOR_CGC_OVERRIDE);
		writel(cgc_override | VFE_175_COLOR_GAMMA,
		       vfe->base + VFE_175_COLOR_CGC_OVERRIDE);
		for (ch = 0; ch < 3; ch++) {
			u32 bank = !(lut_cfg & BIT(ch));
			u32 select = selectors[ch] + bank;

			/* Caller owns pre-stream setup; no concurrent DMI consumer yet. */
			writel((dmi_cfg & ~VFE_175_DMI_SELECT) | select,
			       vfe->base + VFE_175_DMI_CFG);
			for (i = 0; i < VFE_175_GAMMA_ENTRIES; i++) {
				/* Explicit indices avoid depending on the auto-increment mode. */
				writel(i, vfe->base + VFE_175_DMI_ADDR);
				writel(packed[ch][i], vfe->base + VFE_175_DMI_DATA_LO);
			}
		}
		/* Restore shared DMI state before publishing the newly filled banks. */
		writel(dmi_cfg, vfe->base + VFE_175_DMI_CFG);
		writel(dmi_addr, vfe->base + VFE_175_DMI_ADDR);
		/* Complete all LUT writes before changing the bank selection. */
		wmb();
		writel_relaxed(lut_cfg ^ VFE_175_RGB_LUT_BANKS,
			       vfe->base + VFE_175_RGB_LUT_CFG);
		/* Complete the bank switch before returning to the saved clock policy. */
		writel(cgc_override, vfe->base + VFE_175_COLOR_CGC_OVERRIDE);
	}
	/* Publish the bank/configuration before enabling Gamma; no PIX RUP. */
	wmb();
	value = readl_relaxed(vfe->base + VFE_175_COLOR_ENABLE);
	value &= ~VFE_175_COLOR_GAMMA;
	if (config->enabled)
		value |= VFE_175_COLOR_GAMMA;
	writel_relaxed(value, vfe->base + VFE_175_COLOR_ENABLE);

	return 0;
}

void vfe_175_gamma_stop(struct vfe_device *vfe)
{
	u32 value;

	if (!vfe_175_gamma_supported(vfe))
		return;
	value = readl_relaxed(vfe->base + VFE_175_COLOR_ENABLE);
	writel_relaxed(value & ~VFE_175_COLOR_GAMMA,
		       vfe->base + VFE_175_COLOR_ENABLE);
}
