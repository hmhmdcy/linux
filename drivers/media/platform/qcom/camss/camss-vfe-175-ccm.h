/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_CCM_H
#define CAMSS_VFE_175_CCM_H

struct device;
struct vfe_device;
struct v4l2_isp_block_header;

int vfe_175_ccm_validate(struct device *dev,
			 const struct v4l2_isp_block_header *block);
void vfe_175_ccm_params(struct vfe_device *vfe,
			const struct v4l2_isp_block_header *block);
void vfe_175_ccm_init(struct vfe_device *vfe);
void vfe_175_ccm_stop(struct vfe_device *vfe);

#endif
