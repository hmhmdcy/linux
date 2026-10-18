/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_CST_H
#define CAMSS_VFE_175_CST_H

#include <linux/types.h>

struct vfe_device;
struct v4l2_pix_format_mplane;

/* Kernel-internal CST12 configuration; input is nonlinear 10-bit R', G', B'. */
struct vfe_175_cst_config {
	s16 matrix[3][3]; /* Y', Cb, Cr rows; R', G', B' columns; signed Q10. */
	u16 offset[3];
	u16 clamp_min[3];
	u16 clamp_max[3];
	bool enabled;
};

int vfe_175_cst_from_format(const struct v4l2_pix_format_mplane *pix,
			    struct vfe_175_cst_config *config);
int vfe_175_cst_configure(struct vfe_device *vfe,
			 const struct vfe_175_cst_config *config);
void vfe_175_cst_stop(struct vfe_device *vfe);

#endif /* CAMSS_VFE_175_CST_H */
