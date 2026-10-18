/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_DEMUX_H
#define CAMSS_VFE_175_DEMUX_H

#include <linux/types.h>

struct vfe_device;

/* Gains are unsigned15 Q10; Gr/Gb name the green on the red/blue row. */
struct vfe_175_demux_config {
	u16 gr_gain;
	u16 gb_gain;
	u16 r_gain;
	u16 b_gain;
	u16 black_in;
	u16 black_out;
	u8 pattern;
	bool enabled;
};

int vfe_175_demux_from_code(u32 code, struct vfe_175_demux_config *config);
int vfe_175_demux_configure(struct vfe_device *vfe,
			   const struct vfe_175_demux_config *config);
void vfe_175_demux_stop(struct vfe_device *vfe);

#endif /* CAMSS_VFE_175_DEMUX_H */
