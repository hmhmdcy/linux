/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_DS4_H
#define CAMSS_VFE_175_DS4_H

#include <linux/types.h>

/* Coefficients are selected calibration input, never sensor-specific defaults. */
struct vfe_175_ds4_config {
	u32 input_height;
	u16 coefficient[3];
};

/* Ordered Y configuration, filter, C configuration, and R2PD configuration.
 * Compose only: no MMIO, enable bits, DMA or fabricated output pixels. */
int vfe_175_ds4_encode(const struct vfe_175_ds4_config *config,
		      const u32 previous[4], u32 registers[4]);

#endif /* CAMSS_VFE_175_DS4_H */
