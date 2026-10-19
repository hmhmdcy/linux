/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_LSC_H
#define CAMSS_VFE_175_LSC_H

struct device;
struct vfe_device;
struct vfe_line;
struct v4l2_isp_block_header;
struct v4l2_isp_buffer;

int vfe_175_lsc_validate(struct device *dev,
			 const struct v4l2_isp_block_header *block);
int vfe_175_params_validate_line(struct vfe_line *line,
				 const struct v4l2_isp_buffer *params);
void vfe_175_lsc_params(struct vfe_device *vfe,
			const struct v4l2_isp_block_header *block);
void vfe_175_lsc_init(struct vfe_device *vfe);
void vfe_175_lsc_stop(struct vfe_device *vfe);

#endif
