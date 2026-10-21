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

struct device;
struct vfe_device;
struct vfe_line;
struct vfe_output;
struct camss_buffer;
struct v4l2_isp_block_header;
struct v4l2_isp_buffer;
struct camss_formats;
struct v4l2_frmsize_stepwise;
extern const struct camss_formats vfe_formats_ds4_175;
extern const struct v4l2_frmsize_stepwise vfe_frame_sizes_ds4_175;
int vfe_175_ds4_validate(struct device *dev, const struct v4l2_isp_block_header *block);
int vfe_175_ds4_validate_line(struct vfe_line *line, const struct v4l2_isp_buffer *params);
void vfe_175_ds4_params(struct vfe_device *vfe, struct vfe_line *line,
			const struct v4l2_isp_block_header *block);
int vfe_175_ds4_validate_output(const struct vfe_output *output);
int vfe_175_ds4_validate_buffer(const struct vfe_output *output, const struct camss_buffer *buffer);
void vfe_175_ds4_start(struct vfe_device *vfe, struct vfe_output *output);
void vfe_175_ds4_update(struct vfe_device *vfe, struct vfe_output *output, const struct camss_buffer *buffer);
void vfe_175_ds4_stop(struct vfe_device *vfe, struct vfe_output *output);

#endif /* CAMSS_VFE_175_DS4_H */
