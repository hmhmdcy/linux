// SPDX-License-Identifier: GPL-2.0-only
/* Qualcomm VFE175 single-IFE FULL NV12 write-master transport. */

#include <linux/io.h>
#include <linux/limits.h>

#include "camss.h"
#include "camss-vfe.h"
#include "camss-vfe-175-output.h"

#define VFE_175_FULL_MAX_WIDTH               4096
#define VFE_175_FULL_MAX_HEIGHT              4096
#define VFE_175_FULL_Y                       3
#define VFE_175_FULL_C                       4
#define VFE_175_FULL_MASK                    (BIT(3) | BIT(4))
#define VFE_175_BUS_CGC_OVERRIDE             0x200c
#define VFE_175_BUS_COMP_MASK                0x2010
#define VFE_175_BUS_ADDR_NO_SYNC             0x2084
#define VFE_175_WM_CFG(n)                    (0x2208 + (n) * 0x100)
#define VFE_175_WM_IMAGE_ADDR(n)             (0x2214 + (n) * 0x100)
#define VFE_175_WM_IMAGE_OFFSET(n)           (0x2218 + (n) * 0x100)
#define VFE_175_WM_WIDTH(n)                  (0x221c + (n) * 0x100)
#define VFE_175_WM_HEIGHT(n)                 (0x2220 + (n) * 0x100)
#define VFE_175_WM_PACKER(n)                 (0x2224 + (n) * 0x100)
#define VFE_175_WM_STRIDE(n)                 (0x2228 + (n) * 0x100)
#define VFE_175_WM_UBWC_MODE(n)              (0x2244 + (n) * 0x100)
#define VFE_175_WM_IRQ_PERIOD(n)             (0x2248 + (n) * 0x100)
#define VFE_175_WM_IRQ_PATTERN(n)            (0x224c + (n) * 0x100)
#define VFE_175_WM_DROP_PERIOD(n)            (0x2250 + (n) * 0x100)
#define VFE_175_WM_DROP_PATTERN(n)           (0x2254 + (n) * 0x100)
#define VFE_175_WM_FRAME_INC(n)              (0x2258 + (n) * 0x100)
#define VFE_175_WM_BURST_LIMIT(n)            (0x225c + (n) * 0x100)
#define VFE_175_PACKER_NV12                  0xe

int vfe_175_validate_output(const struct vfe_output *output)
{
	const struct v4l2_pix_format_mplane *pix = &output->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[0].bytesperline;
	u64 required;

	/* One single-IFE FULL image, fixed Y/C clients and composite group zero. */
	if (!output->line->is_pix || output->pad != MSM_VFE_PAD_SRC ||
	    output != &output->line->output[0] || output->wm_num != 2 ||
	    output->comp_group != 0 || output->line->num_outputs != 1 ||
	    output->wm[0].bus_client != VFE_175_FULL_Y || output->wm[0].plane != 0 ||
	    output->wm[1].bus_client != VFE_175_FULL_C || output->wm[1].plane != 1)
		return -EINVAL;
	if (pix->pixelformat != V4L2_PIX_FMT_NV12 || pix->num_planes != 1 ||
	    !pix->width || !pix->height || (pix->width & 1) || (pix->height & 1) ||
	    pix->width > VFE_175_FULL_MAX_WIDTH ||
	    pix->height > VFE_175_FULL_MAX_HEIGHT ||
	    stride < pix->width || !IS_ALIGNED(stride, 16) || stride > U16_MAX)
		return -EINVAL;

	required = (u64)stride * pix->height * 3 / 2;
	if (required > pix->plane_fmt[0].sizeimage)
		return -EINVAL;

	return 0;
}

int vfe_175_validate_buffer(const struct vfe_output *output,
			    const struct camss_buffer *buf)
{
	const struct v4l2_pix_format_mplane *pix = &output->video_out.active_fmt.fmt.pix_mp;
	u64 luma = (u64)pix->plane_fmt[0].bytesperline * pix->height;
	u64 size = luma * 3 / 2;
	u64 address = buf->addr[0];
	int ret;

	ret = vfe_175_validate_output(output);
	if (ret)
		return ret;
	if (address > U32_MAX || size - 1 > U32_MAX - address)
		return -ERANGE;
	if (!IS_ALIGNED(address, 16) || buf->addr[1] != address + luma)
		return -EINVAL;

	return 0;
}

/* Called only after line format and capture DMA preparation succeeded. */
void vfe_175_full_start(struct vfe_device *vfe, struct vfe_output *output)
{
	const struct v4l2_pix_format_mplane *pix = &output->video_out.active_fmt.fmt.pix_mp;
	u32 stride = pix->plane_fmt[0].bytesperline;
	u32 value;
	unsigned int i;

	value = readl_relaxed(vfe->base + VFE_175_BUS_CGC_OVERRIDE);
	writel_relaxed(value | VFE_175_FULL_MASK, vfe->base + VFE_175_BUS_CGC_OVERRIDE);
	/* Single IFE: no cross-core address synchronization. */
	value = readl_relaxed(vfe->base + VFE_175_BUS_ADDR_NO_SYNC);
	writel_relaxed(value | VFE_175_FULL_MASK, vfe->base + VFE_175_BUS_ADDR_NO_SYNC);
	value = readl_relaxed(vfe->base + VFE_175_BUS_COMP_MASK);
	writel_relaxed(value | VFE_175_FULL_MASK, vfe->base + VFE_175_BUS_COMP_MASK);

	for (i = 0; i < output->wm_num; i++) {
		u8 wm = output->wm[i].bus_client;
		u32 height = pix->height >> output->wm[i].plane;

		writel_relaxed(0, vfe->base + VFE_175_WM_IMAGE_OFFSET(wm));
		writel_relaxed(pix->width, vfe->base + VFE_175_WM_WIDTH(wm));
		writel_relaxed(height, vfe->base + VFE_175_WM_HEIGHT(wm));
		writel_relaxed(VFE_175_PACKER_NV12, vfe->base + VFE_175_WM_PACKER(wm));
		writel_relaxed(stride, vfe->base + VFE_175_WM_STRIDE(wm));
		writel_relaxed(stride * height, vfe->base + VFE_175_WM_FRAME_INC(wm));
		writel_relaxed(0xf, vfe->base + VFE_175_WM_BURST_LIMIT(wm));
		writel_relaxed(0, vfe->base + VFE_175_WM_IRQ_PERIOD(wm));
		writel_relaxed(1, vfe->base + VFE_175_WM_IRQ_PATTERN(wm));
		writel_relaxed(0, vfe->base + VFE_175_WM_DROP_PERIOD(wm));
		writel_relaxed(1, vfe->base + VFE_175_WM_DROP_PATTERN(wm));
		value = readl_relaxed(vfe->base + VFE_175_WM_UBWC_MODE(wm));
		writel_relaxed(value & ~BIT(0), vfe->base + VFE_175_WM_UBWC_MODE(wm));
		writel_relaxed(1, vfe->base + VFE_175_WM_CFG(wm));
	}
}

void vfe_175_full_update(struct vfe_device *vfe, struct vfe_output *output,
			 const struct camss_buffer *buf)
{
	unsigned int i;

	/* 175 uses byte addresses; newer IFE address>>8 programming is different. */
	for (i = 0; i < output->wm_num; i++)
		writel_relaxed(buf->addr[output->wm[i].plane],
			       vfe->base + VFE_175_WM_IMAGE_ADDR(output->wm[i].bus_client));
}

void vfe_175_full_stop(struct vfe_device *vfe, struct vfe_output *output)
{
	u32 value;
	unsigned int i;

	for (i = 0; i < output->wm_num; i++)
		writel_relaxed(0, vfe->base + VFE_175_WM_CFG(output->wm[i].bus_client));
	value = readl_relaxed(vfe->base + VFE_175_BUS_COMP_MASK);
	writel_relaxed(value & ~VFE_175_FULL_MASK, vfe->base + VFE_175_BUS_COMP_MASK);
}

void vfe_175_comp_done(struct vfe_device *vfe, u8 comp)
{
	struct camss_buf_done_info info = { };

	if (comp >= 6 || vfe->res->line_num <= VFE_LINE_PIX ||
	    !vfe->line[VFE_LINE_PIX].is_pix)
		return;

	info.status = BIT(comp);
	/* One image completion for the group, never one per Y/C client. */
	vfe_buf_done(vfe, VFE_LINE_PIX, &info);
}
