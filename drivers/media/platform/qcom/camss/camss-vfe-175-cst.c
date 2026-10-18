// SPDX-License-Identifier: GPL-2.0-only
/* SM8150 full VFE175 CST12 color encoding, separate from sensor color tuning. */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/videodev2.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-cst.h"

#define VFE_175_FRAME_ENABLE		0x04c
#define VFE_175_FRAME_CST		BIT(0)
#define VFE_175_CST_COEFF0(n)		(0xf30 + (n) * 0x10)
#define VFE_175_CST_COEFF1(n)		(0xf34 + (n) * 0x10)
#define VFE_175_CST_OFFSET(n)		(0xf38 + (n) * 0x10)
#define VFE_175_CST_CLAMP(n)		(0xf3c + (n) * 0x10)
#define VFE_175_CST_C0			GENMASK(12, 0)
#define VFE_175_CST_C1			GENMASK(28, 16)
#define VFE_175_CST_INPUT_OFFSET		GENMASK(10, 0)
#define VFE_175_CST_OUTPUT_OFFSET		GENMASK(26, 16)
#define VFE_175_CST_MIN			GENMASK(9, 0)
#define VFE_175_CST_MAX			GENMASK(25, 16)

static bool vfe_175_cst_supported(struct vfe_device *vfe)
{
	return !vfe->res->is_lite && vfe->camss->res->version == CAMSS_8150;
}

int vfe_175_cst_from_format(const struct v4l2_pix_format_mplane *pix,
			    struct vfe_175_cst_config *config)
{
	/* Rounding preserves zero chroma on the neutral axis in all profiles. */
	static const s16 matrices[2][2][3][3] = {
		{ /* BT.601: full, limited. */
			{ { 306, 601, 117 }, { -173, -339, 512 }, { 512, -429, -83 } },
			{ { 262, 515, 100 }, { -151, -297, 448 }, { 448, -375, -73 } },
		},
		{ /* BT.709: full, limited. */
			{ { 218, 732, 74 }, { -117, -395, 512 }, { 512, -465, -47 } },
			{ { 186, 628, 63 }, { -103, -345, 448 }, { 448, -407, -41 } },
		},
	};
	struct vfe_175_cst_config result = { .enabled = true };
	u32 encoding, quantization;
	unsigned int profile, limited, i;

	if (!pix || !config || pix->pixelformat != V4L2_PIX_FMT_NV12)
		return -EINVAL;
	encoding = pix->ycbcr_enc;
	if (encoding == V4L2_YCBCR_ENC_DEFAULT)
		encoding = V4L2_MAP_YCBCR_ENC_DEFAULT(pix->colorspace);
	if (encoding != V4L2_YCBCR_ENC_601 && encoding != V4L2_YCBCR_ENC_709)
		return -EINVAL;
	quantization = pix->quantization;
	if (quantization == V4L2_QUANTIZATION_DEFAULT)
		quantization = V4L2_MAP_QUANTIZATION_DEFAULT(false, pix->colorspace,
							    encoding);
	if (quantization != V4L2_QUANTIZATION_FULL_RANGE &&
	    quantization != V4L2_QUANTIZATION_LIM_RANGE)
		return -EINVAL;

	profile = encoding == V4L2_YCBCR_ENC_709;
	limited = quantization == V4L2_QUANTIZATION_LIM_RANGE;
	memcpy(result.matrix, matrices[profile][limited], sizeof(result.matrix));
	/* Limited 10-bit Y': 64..940; Cb/Cr: 64..960, centered at 512. */
	result.offset[0] = limited ? 64 : 0;
	result.offset[1] = 512;
	result.offset[2] = 512;
	for (i = 0; i < 3; i++) {
		result.clamp_min[i] = limited ? 64 : 0;
		result.clamp_max[i] = limited ? (i ? 960 : 940) : 1023;
	}
	/* Transfer/gamut conversion belongs to the preceding Gamma/CC stages. */
	*config = result;

	return 0;
}

static void vfe_175_cst_write(struct vfe_device *vfe, u32 reg, u32 mask, u32 data)
{
	u32 value = readl_relaxed(vfe->base + reg);

	writel_relaxed((value & ~mask) | data, vfe->base + reg);
}

int vfe_175_cst_configure(struct vfe_device *vfe,
			 const struct vfe_175_cst_config *config)
{
	unsigned int i, j;

	if (!vfe_175_cst_supported(vfe) || !config)
		return -EINVAL;
	/* Validate every row before touching even the first register. */
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++)
			if (config->matrix[i][j] < -4096 || config->matrix[i][j] > 4095)
				return -EINVAL;
		if (config->offset[i] > 1023 || config->clamp_max[i] > 1023 ||
		    config->clamp_min[i] > config->clamp_max[i])
			return -EINVAL;
	}

	for (i = 0; i < 3; i++) {
		/* CST12 slots are G', B', R'; keep the kernel config in RGB order. */
		vfe_175_cst_write(vfe, VFE_175_CST_COEFF0(i),
				  VFE_175_CST_C0 | VFE_175_CST_C1,
				  FIELD_PREP(VFE_175_CST_C0, (u32)config->matrix[i][1] & 0x1fff) |
				  FIELD_PREP(VFE_175_CST_C1, (u32)config->matrix[i][2] & 0x1fff));
		vfe_175_cst_write(vfe, VFE_175_CST_COEFF1(i), VFE_175_CST_C0,
				  FIELD_PREP(VFE_175_CST_C0, (u32)config->matrix[i][0] & 0x1fff));
		/* Input subtraction remains zero until its transform is validated. */
		vfe_175_cst_write(vfe, VFE_175_CST_OFFSET(i),
				  VFE_175_CST_INPUT_OFFSET | VFE_175_CST_OUTPUT_OFFSET,
				  FIELD_PREP(VFE_175_CST_OUTPUT_OFFSET, config->offset[i]));
		vfe_175_cst_write(vfe, VFE_175_CST_CLAMP(i),
				  VFE_175_CST_MIN | VFE_175_CST_MAX,
				  FIELD_PREP(VFE_175_CST_MIN, config->clamp_min[i]) |
				  FIELD_PREP(VFE_175_CST_MAX, config->clamp_max[i]));
	}
	/* Latch only when the output issues PIX RUP, after all modules are ready. */
	wmb();
	vfe_175_cst_write(vfe, VFE_175_FRAME_ENABLE, VFE_175_FRAME_CST,
			  config->enabled ? VFE_175_FRAME_CST : 0);

	return 0;
}

void vfe_175_cst_stop(struct vfe_device *vfe)
{
	if (!vfe_175_cst_supported(vfe))
		return;
	vfe_175_cst_write(vfe, VFE_175_FRAME_ENABLE, VFE_175_FRAME_CST, 0);
}
