// SPDX-License-Identifier: GPL-2.0-only
/* VFE175 colour correction. Matrices and tuning policy stay in userspace. */
#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/qcom-camss-config.h>
#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-ccm.h"

#define COLOR_ENABLE	0x048
#define CCM_ENABLE	BIT(1)
#define CCM_COEFF0	0x760
#define CCM_OFFSET0	0x784
#define CCM_QFACTOR	0x790
#define CCM_COEFF_MASK	(GENMASK(11, 0) | GENMASK(27, 16))
#define CCM_OFFSET_MASK	(GENMASK(10, 0) | GENMASK(26, 16))
#define CCM_Q_MASK	(GENMASK(1, 0) | GENMASK(17, 16))

static void ccm_enable(struct vfe_device *vfe, bool enable)
{
	u32 value = readl_relaxed(vfe->base + COLOR_ENABLE);

	value &= ~CCM_ENABLE;
	if (enable)
		value |= CCM_ENABLE;
	writel_relaxed(value, vfe->base + COLOR_ENABLE);
}

static void ccm_write(struct vfe_device *vfe, unsigned int offset,
		      u32 mask, u16 field)
{
	u32 value = readl_relaxed(vfe->base + offset);

	/* Apply the same matrix to both hardware stripe fields. */
	value = (value & ~mask) | ((u32)field | ((u32)field << 16));
	writel_relaxed(value, vfe->base + offset);
}

int vfe_175_ccm_validate(struct device *dev,
			 const struct v4l2_isp_block_header *block)
{
	const struct camss_params_color_correct *ccm;
	unsigned int i;

	(void)dev;
	if (block->flags & ~(V4L2_ISP_PARAMS_FL_BLOCK_ENABLE |
			     V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) ||
	    block->flags == (V4L2_ISP_PARAMS_FL_BLOCK_ENABLE |
			     V4L2_ISP_PARAMS_FL_BLOCK_DISABLE))
		return -EINVAL;
	/* Header-only disable is valid; its payload must never be read. */
	if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE)
		return 0;
	ccm = container_of(block, struct camss_params_color_correct, header);
	if (ccm->qfactor > 3 || ccm->_pad[0] || ccm->_pad[1] || ccm->_pad[2])
		return -EINVAL;
	for (i = 0; i < 3; i++)
		if (ccm->a[i] > 0xfff || ccm->b[i] > 0xfff ||
		    ccm->c[i] > 0xfff || ccm->k[i] > 0x7ff)
			return -EINVAL;
	return 0;
}

void vfe_175_ccm_params(struct vfe_device *vfe,
			const struct v4l2_isp_block_header *block)
{
	const struct camss_params_color_correct *ccm;
	unsigned int i;

	if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE) {
		ccm_enable(vfe, false);
		return;
	}
	ccm = container_of(block, struct camss_params_color_correct, header);
	for (i = 0; i < 3; i++) {
		ccm_write(vfe, CCM_COEFF0 + i * 12, CCM_COEFF_MASK, ccm->a[i]);
		ccm_write(vfe, CCM_COEFF0 + i * 12 + 4, CCM_COEFF_MASK, ccm->b[i]);
		ccm_write(vfe, CCM_COEFF0 + i * 12 + 8, CCM_COEFF_MASK, ccm->c[i]);
		ccm_write(vfe, CCM_OFFSET0 + i * 4, CCM_OFFSET_MASK, ccm->k[i]);
	}
	ccm_write(vfe, CCM_QFACTOR, CCM_Q_MASK, ccm->qfactor);
	if (block->flags & V4L2_ISP_PARAMS_FL_BLOCK_ENABLE)
		ccm_enable(vfe, true);
}

void vfe_175_ccm_init(struct vfe_device *vfe)
{
	const struct camss_params_color_correct unity = {
		.header = { .type = CAMSS_PARAMS_COLOR_CORRECT },
		.a = { 128, 0, 0 },
		.b = { 0, 128, 0 },
		.c = { 0, 0, 128 },
	};

	ccm_enable(vfe, false);
	vfe_175_ccm_params(vfe, &unity.header);
}

void vfe_175_ccm_stop(struct vfe_device *vfe)
{
	ccm_enable(vfe, false);
}
