/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_GAMMA_H
#define CAMSS_VFE_175_GAMMA_H

#include <linux/types.h>

#define VFE_175_GAMMA_ENTRIES 64
#define VFE_175_GAMMA_POINTS (VFE_175_GAMMA_ENTRIES + 1)

struct vfe_device;
struct v4l2_pix_format_mplane;

/* RGB channel order, 65 uniformly spaced input knots, 10-bit output domain. */
struct vfe_175_gamma_config {
	u16 points[3][VFE_175_GAMMA_POINTS];
	bool enabled;
};

int vfe_175_gamma_from_format(const struct v4l2_pix_format_mplane *pix,
			      struct vfe_175_gamma_config *config);
int vfe_175_gamma_configure(struct vfe_device *vfe,
			   const struct vfe_175_gamma_config *config);
void vfe_175_gamma_stop(struct vfe_device *vfe);

#endif /* CAMSS_VFE_175_GAMMA_H */
