/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_MNDS_H
#define CAMSS_VFE_175_MNDS_H

#include <linux/types.h>

struct vfe_device;
struct vfe_line;

struct vfe_175_mnds_config {
	u32 input_width;
	u32 input_height;
	u32 output_width;
	u32 output_height;
	bool enabled;
};

int vfe_175_mnds_from_line(const struct vfe_line *line,
			  struct vfe_175_mnds_config *config);
int vfe_175_mnds_configure(struct vfe_device *vfe,
			  const struct vfe_175_mnds_config *config);
void vfe_175_mnds_stop(struct vfe_device *vfe);

#endif /* CAMSS_VFE_175_MNDS_H */
