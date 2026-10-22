/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_GTM_H
#define CAMSS_VFE_175_GTM_H
#include <linux/types.h>
struct device;
struct vfe_device;
struct vfe_output;
struct camss_buffer;
struct v4l2_isp_buffer;
struct v4l2_isp_block_header;
struct camss_params_ife_gtm;

struct vfe_175_gtm_state {
	u64 cookie, generation, pending_cookie;
	u32 status, bank, pending_bank;
	bool pending, armed, pending_enabled, tracking, deferred;
};
int vfe_175_gtm_encode(const struct camss_params_ife_gtm *gtm, u32 words[128]);
int vfe_175_gtm_validate(struct device *dev, const struct v4l2_isp_block_header *block);
int vfe_175_gtm_validate_buffer(const struct v4l2_isp_buffer *params);
void vfe_175_gtm_params(struct vfe_device *vfe, const struct v4l2_isp_block_header *block);
bool vfe_175_gtm_defer(struct vfe_device *vfe, const struct v4l2_isp_buffer *params);
void vfe_175_gtm_arm(struct vfe_device *vfe);
void vfe_175_gtm_ack(struct vfe_device *vfe);
void vfe_175_gtm_tag(struct vfe_device *vfe, struct camss_buffer *buf);
void vfe_175_gtm_fill(struct vfe_device *vfe, struct vfe_output *output,
		    const struct camss_buffer *buf, struct v4l2_isp_buffer *stats);
void vfe_175_gtm_stop(struct vfe_device *vfe);
#endif
