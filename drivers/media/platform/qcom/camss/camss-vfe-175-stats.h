/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_STATS_H
#define CAMSS_VFE_175_STATS_H

#include <linux/types.h>

struct device;
struct camss_video;
struct camss_buffer;
struct vfe_device;
struct vfe_line;
struct vfe_output;
struct v4l2_isp_block_header;

struct vfe_175_stats_state {
	u16 left, top, width, height;
	bool enabled;
};

int vfe_175_bhist_validate(struct device *dev,
			   const struct v4l2_isp_block_header *block);
int vfe_175_stats_validate_output(const struct vfe_output *output);
int vfe_175_stats_prepare(struct vfe_line *line);
void vfe_175_stats_params(struct vfe_device *vfe, struct vfe_line *line,
			  const struct v4l2_isp_block_header *block);
void vfe_175_stats_start(struct vfe_device *vfe, struct vfe_output *output);
void vfe_175_stats_stop(struct vfe_device *vfe, struct vfe_output *output);
void vfe_175_stats_update(struct vfe_device *vfe, struct vfe_output *output);

#endif /* CAMSS_VFE_175_STATS_H */
