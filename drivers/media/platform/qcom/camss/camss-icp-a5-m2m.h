/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_ICP_A5_M2M_H
#define CAMSS_ICP_A5_M2M_H

#include <linux/types.h>
struct module;
struct camss_a5;
struct camss_a5_m2m;

/* The platform owner supplies initialized resources/runtime PM and exclusive
 * A5 ownership. Open files and retained hardware backing pin that owner.
 */
struct camss_a5_m2m *camss_a5_m2m_register(struct camss_a5 *a5,
					struct module *owner, const char *firmware,
					bool (*bound)(struct device *dev));
int camss_a5_m2m_cleanup(struct camss_a5_m2m *dev);
int camss_a5_m2m_unregister(struct camss_a5_m2m *dev);
ssize_t camss_a5_m2m_status(struct camss_a5_m2m *dev, char *buffer, size_t size);
ssize_t camss_a5_m2m_dc_snapshot(struct camss_a5_m2m *dev, char *buffer,
				loff_t offset, size_t count);
#endif
