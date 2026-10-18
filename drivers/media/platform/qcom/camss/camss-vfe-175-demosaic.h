/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_DEMOSAIC_H
#define CAMSS_VFE_175_DEMOSAIC_H

#include <linux/types.h>

struct vfe_device;
struct vfe_line;

struct vfe_175_demosaic_config {
	u16 lambda_g;
	u16 lambda_rb;
	u16 classifier_a;
	s16 classifier_w;
	bool directional_g_disable;
	bool directional_rb_disable;
	bool cosited_rgb;
	bool dynamic_g_clamp;
	bool dynamic_rb_clamp;
	bool enabled;
};

int vfe_175_demosaic_configure(struct vfe_device *vfe,
			       const struct vfe_175_demosaic_config *config);
int vfe_175_demosaic_prepare(struct vfe_line *line);
void vfe_175_demosaic_stop(struct vfe_device *vfe);

#endif /* CAMSS_VFE_175_DEMOSAIC_H */
