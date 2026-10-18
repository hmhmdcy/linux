/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_PIXEL_H
#define CAMSS_VFE_175_PIXEL_H

#include <linux/types.h>
#include <media/v4l2-isp.h>
#include "camss-vfe-175-stats.h"

struct vfe_device;
struct vfe_line;
struct v4l2_isp_buffer;
struct v4l2_isp_params_block_type_info;

/* Cached configuration survives parameter updates while WB is bypassed. */
struct vfe_175_pixel_state {
	u16 g_gain;
	u16 b_gain;
	u16 r_gain;
	bool wb_enabled;
	struct vfe_175_stats_state stats;
};

extern const struct v4l2_isp_params_block_type_info vfe_175_params_info[6];
int vfe_175_pixel_prepare(struct vfe_line *line);
void vfe_175_pixel_stop(struct vfe_device *vfe);
void vfe_175_params_apply(struct vfe_device *vfe, struct vfe_line *line,
			  const struct v4l2_isp_buffer *params);

#endif /* CAMSS_VFE_175_PIXEL_H */
