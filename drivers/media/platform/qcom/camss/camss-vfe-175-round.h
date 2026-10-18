/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_ROUND_H
#define CAMSS_VFE_175_ROUND_H

#include <linux/types.h>

struct vfe_device;
struct vfe_line;

struct vfe_175_round_config {
	u16 y_min;
	u16 y_max;
	u16 c_min;
	u16 c_max;
	u8 output_bits;
	bool enabled;
};

int vfe_175_round_configure(struct vfe_device *vfe,
			   const struct vfe_175_round_config *config);
int vfe_175_round_prepare(struct vfe_line *line);
void vfe_175_round_stop(struct vfe_device *vfe);

#endif /* CAMSS_VFE_175_ROUND_H */
