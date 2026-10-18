/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_VFE_175_OUTPUT_H
#define CAMSS_VFE_175_OUTPUT_H

#include <linux/types.h>

struct vfe_device;
struct vfe_output;
struct camss_buffer;

int vfe_175_validate_output(const struct vfe_output *output);
int vfe_175_validate_buffer(const struct vfe_output *output,
			    const struct camss_buffer *buf);
void vfe_175_full_start(struct vfe_device *vfe, struct vfe_output *output);
void vfe_175_full_stop(struct vfe_device *vfe, struct vfe_output *output);
void vfe_175_full_update(struct vfe_device *vfe, struct vfe_output *output,
			 const struct camss_buffer *buf);
void vfe_175_comp_done(struct vfe_device *vfe, u8 comp);

#endif /* CAMSS_VFE_175_OUTPUT_H */
