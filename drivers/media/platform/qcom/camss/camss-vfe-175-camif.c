// SPDX-License-Identifier: GPL-2.0-only
/* Qualcomm SM8150 full VFE175 CAMIF pixel input configuration. */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/media-bus-format.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-camif.h"

#define VFE_175_CORE_CFG                     0x050
#define VFE_175_CORE_PIXEL_PATTERN           GENMASK(2, 0)
#define VFE_175_CORE_DSP_ENABLE              BIT(3)
#define VFE_175_CORE_EXTERNAL_UPDATE         BIT(4)
#define VFE_175_CORE_INPUT_MUX               GENMASK(6, 5)
#define VFE_175_CORE_DSP_MODE                BIT(23)

#define VFE_175_CAMIF_CMD                    0x478
#define VFE_175_CAMIF_CLEAR_STATUS           BIT(2)
#define VFE_175_CAMIF_CONFIG                 0x47c
#define VFE_175_CAMIF_VFE_OUTPUT             BIT(6)
#define VFE_175_CAMIF_BUS_OUTPUT             BIT(7)
#define VFE_175_CAMIF_RAW_CROP               BIT(22)
#define VFE_175_CAMIF_LINE_SKIP              0x488
#define VFE_175_CAMIF_PIXEL_SKIP             0x48c
#define VFE_175_CAMIF_SKIP_PERIOD            0x490
#define VFE_175_CAMIF_IRQ_SUBSAMPLE           0x49c
#define VFE_175_CAMIF_EPOCH                  0x4a0
#define VFE_175_RAW_CROP_WIDTH               0xce4
#define VFE_175_RAW_CROP_HEIGHT              0xce8
#define VFE_175_RAW_CROP_LAST                GENMASK(13, 0)

/* The full IFE accepts decoded Bayer samples, not the packed RDI payload. */
static int vfe_175_bayer_pattern(u32 code)
{
	switch (code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_SRGGB14_1X14:
		return 0;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGRBG14_1X14:
		return 1;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SBGGR14_1X14:
		return 2;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGBRG14_1X14:
		return 3;
	default:
		return -EINVAL;
	}
}

int vfe_175_camif_prepare(struct vfe_line *line)
{
	struct vfe_device *vfe = to_vfe(line);
	const struct v4l2_mbus_framefmt *input = &line->fmt[MSM_VFE_PAD_SINK];
	u32 core, last_pixel, last_line;
	int pattern;

	/* RDI register programming remains entirely in the shared 17x backend. */
	if (!line->is_pix)
		return 0;
	if (vfe->res->is_lite || vfe->camss->res->version != CAMSS_8150)
		return -EINVAL;

	pattern = vfe_175_bayer_pattern(input->code);
	if (pattern < 0)
		return pattern;
	if (!input->width || !input->height ||
	    input->width - 1 > FIELD_MAX(VFE_175_RAW_CROP_LAST) ||
	    input->height - 1 > FIELD_MAX(VFE_175_RAW_CROP_LAST))
		return -EINVAL;

	last_pixel = input->width - 1;
	last_line = input->height - 1;

	/* Single-IFE CSID input; preserve core fields owned by other paths. */
	core = readl_relaxed(vfe->base + VFE_175_CORE_CFG);
	core &= ~(VFE_175_CORE_PIXEL_PATTERN | VFE_175_CORE_DSP_ENABLE |
		  VFE_175_CORE_EXTERNAL_UPDATE | VFE_175_CORE_INPUT_MUX |
		  VFE_175_CORE_DSP_MODE);
	core |= FIELD_PREP(VFE_175_CORE_PIXEL_PATTERN, pattern);
	writel_relaxed(core, vfe->base + VFE_175_CORE_CFG);

	writel_relaxed(VFE_175_CAMIF_CLEAR_STATUS,
		       vfe->base + VFE_175_CAMIF_CMD);
	writel_relaxed(VFE_175_CAMIF_VFE_OUTPUT | VFE_175_CAMIF_BUS_OUTPUT |
		       VFE_175_CAMIF_RAW_CROP, vfe->base + VFE_175_CAMIF_CONFIG);

	/* Sixteen accepted lines/pixels, no dropped frames or IRQ subsampling. */
	writel_relaxed(0xffff, vfe->base + VFE_175_CAMIF_LINE_SKIP);
	writel_relaxed(0xffff, vfe->base + VFE_175_CAMIF_PIXEL_SKIP);
	writel_relaxed(0x0f0f, vfe->base + VFE_175_CAMIF_SKIP_PERIOD);
	writel_relaxed(1, vfe->base + VFE_175_CAMIF_IRQ_SUBSAMPLE);

	/* Full decoded frame; output crop/scaling belongs to the pixel modules. */
	writel_relaxed(last_pixel, vfe->base + VFE_175_RAW_CROP_WIDTH);
	writel_relaxed(last_line, vfe->base + VFE_175_RAW_CROP_HEIGHT);
	writel_relaxed((last_line / 2) << 16 | min_t(u32, last_line, 20),
		       vfe->base + VFE_175_CAMIF_EPOCH);

	/* The caller must arm processing/outputs before issuing PIX reg update. */
	return 0;
}
