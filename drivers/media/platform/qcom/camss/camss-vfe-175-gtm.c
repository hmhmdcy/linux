// SPDX-License-Identifier: GPL-2.0-only
/* SM8150 IFE GTM; canonical CPU DMI words, independent of OEM calibration. */
#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/qcom-camss-config.h>
#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-gtm.h"

#define GTM_CGC 0x034
#define GTM_ENABLE 0x048
#define GTM_BIT BIT(2)
#define GTM_CONFIG 0x794
#define GTM_BANK BIT(0)
#define DMI_CONFIG 0xc24
#define DMI_ADDRESS 0xc28
#define DMI_HIGH 0xc2c
#define DMI_LOW 0xc30
#define DMI_SELECT GENMASK(6, 0)
#define DMI_AUTO_INCREMENT BIT(8)
#define DMI_AUTO_LOAD BIT(12)
#define GTM_DMI_BANK0 24

int vfe_175_gtm_encode(const struct camss_params_ife_gtm *gtm, u32 words[128])
{
	unsigned int i;

	if (!gtm || !words || !gtm->cookie || gtm->enabled > 1 || gtm->reserved)
		return -EINVAL;
	for (i = 0; i < CAMSS_IFE_GTM_ENTRIES; i++)
		if (gtm->segments[i].base > 0x3ffff ||
		    gtm->segments[i].slope < -(1 << 25) ||
		    gtm->segments[i].slope > (1 << 25) - 1)
			return -EINVAL;
	for (i = 0; i < CAMSS_IFE_GTM_ENTRIES; i++) {
		words[i * 2] = gtm->segments[i].base;
		words[i * 2 + 1] = (u32)gtm->segments[i].slope & 0x3ffffff;
	}
	return 0;
}

int vfe_175_gtm_validate(struct device *dev, const struct v4l2_isp_block_header *block)
{
	const struct camss_params_ife_gtm *gtm;
	u32 words[128];

	/* Full payload, including an opaque cookie, is required for disable too. */
	if (block->flags || block->size != sizeof(*gtm))
		return -EINVAL;
	gtm = container_of(block, struct camss_params_ife_gtm, header);
	return vfe_175_gtm_encode(gtm, words);
}

static void gtm_enable(struct vfe_device *vfe, bool enable)
{
	u32 value = readl_relaxed(vfe->base + GTM_ENABLE);

	writel_relaxed((value & ~GTM_BIT) | (enable ? GTM_BIT : 0),
		       vfe->base + GTM_ENABLE);
}

void vfe_175_gtm_params(struct vfe_device *vfe, const struct v4l2_isp_block_header *block)
{
	const struct camss_params_ife_gtm *gtm =
		container_of(block, struct camss_params_ife_gtm, header);
	struct vfe_175_gtm_state *s = &vfe->pixel175.gtm;
	u32 config, dmi, address, cgc, bank, words[128];
	unsigned int i;

	/* Queue validation and defer() precede all processing MMIO. */
	if (WARN_ON_ONCE(s->pending) || vfe_175_gtm_encode(gtm, words))
		return;
	config = readl_relaxed(vfe->base + GTM_CONFIG);
	bank = config & GTM_BANK;
	if (gtm->enabled) {
		bank ^= 1;
		dmi = readl_relaxed(vfe->base + DMI_CONFIG);
		address = readl_relaxed(vfe->base + DMI_ADDRESS);
		cgc = readl_relaxed(vfe->base + GTM_CGC);
		writel(cgc | GTM_BIT, vfe->base + GTM_CGC);
		writel((dmi & ~(DMI_SELECT | DMI_AUTO_INCREMENT | DMI_AUTO_LOAD)) |
		       (GTM_DMI_BANK0 + bank), vfe->base + DMI_CONFIG);
		for (i = 0; i < CAMSS_IFE_GTM_ENTRIES; i++) {
			writel(i, vfe->base + DMI_ADDRESS);
			/* Pinned official SW CDM64 writes low then high; no HW-CDM swap. */
			writel(words[i * 2], vfe->base + DMI_LOW);
			writel(words[i * 2 + 1], vfe->base + DMI_HIGH);
		}
		readl(vfe->base + DMI_CONFIG);
		writel(dmi, vfe->base + DMI_CONFIG);
		writel(address, vfe->base + DMI_ADDRESS);
		wmb();
		writel_relaxed((config & ~GTM_BANK) | bank, vfe->base + GTM_CONFIG);
		writel(cgc, vfe->base + GTM_CGC);
	}
	gtm_enable(vfe, gtm->enabled);
	s->pending_cookie = gtm->cookie;
	s->pending_bank = bank;
	s->pending_enabled = gtm->enabled;
	s->pending = true;
	s->armed = false;
	s->tracking = true;
}

bool vfe_175_gtm_defer(struct vfe_device *vfe, const struct v4l2_isp_buffer *params)
{
	struct vfe_175_gtm_state *s = &vfe->pixel175.gtm;
	bool tracked = s->tracking;
	size_t offset;

	for (offset = 0; !tracked && offset < params->data_size;) {
		const struct v4l2_isp_block_header *block = (const void *)&params->data[offset];

		tracked = block->type == CAMSS_PARAMS_IFE_GTM;
		offset += block->size;
	}
	/* Never attach a new cookie to an older outstanding PIX RUP ACK. */
	s->deferred = tracked && (s->pending || (vfe->reg_update & BIT(0)));
	return s->deferred;
}

void vfe_175_gtm_arm(struct vfe_device *vfe)
{
	if (vfe->pixel175.gtm.pending)
		vfe->pixel175.gtm.armed = true;
}

void vfe_175_gtm_ack(struct vfe_device *vfe)
{
	struct vfe_175_gtm_state *s = &vfe->pixel175.gtm;

	if (!s->pending || !s->armed)
		return;
	s->cookie = s->pending_cookie;
	s->bank = s->pending_bank;
	s->generation++;
	s->status = CAMSS_IFE_GTM_STATUS_ACKED |
		(s->pending_enabled ? CAMSS_IFE_GTM_STATUS_ENABLED : 0);
	s->pending = false;
	s->armed = false;
}

void vfe_175_gtm_tag(struct vfe_device *vfe, struct camss_buffer *buf)
{
	const struct vfe_175_gtm_state *s = &vfe->pixel175.gtm;

	buf->gtm_cookie = s->cookie;
	buf->gtm_generation = s->generation;
	buf->gtm_status = s->status;
	buf->gtm_bank = s->bank;
}

void vfe_175_gtm_fill(struct vfe_device *vfe, struct vfe_output *output,
		    const struct camss_buffer *buf, struct v4l2_isp_buffer *stats)
{
	struct v4l2_isp_stats_block_type_info info[] = {
		[CAMSS_STATS_IFE_GTM] = { .size = sizeof(struct camss_stats_ife_gtm) },
	};
	struct v4l2_isp_block_header *block;
	struct camss_stats_ife_gtm *gtm;

	if (!buf->sof_valid || !buf->gtm_cookie)
		return;
	block = v4l2_isp_stats_init_block(vfe->camss->dev, stats, info,
		ARRAY_SIZE(info), CAMSS_STATS_IFE_GTM, output->stats_payload_size);
	if (IS_ERR(block))
		return;
	gtm = container_of(block, struct camss_stats_ife_gtm, header);
	gtm->cookie = buf->gtm_cookie;
	gtm->generation = buf->gtm_generation;
	gtm->status = buf->gtm_status;
	gtm->bank = buf->gtm_bank;
}

void vfe_175_gtm_stop(struct vfe_device *vfe)
{
	gtm_enable(vfe, false);
	vfe->pixel175.gtm = (struct vfe_175_gtm_state) { };
}
