/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_ICP_A5_RESOURCES_H
#define CAMSS_ICP_A5_RESOURCES_H

#include <linux/clk.h>
#include <linux/pm_domain.h>

struct platform_device;

/* The owner serializes these calls and stops A5 before suspending resources. */
struct camss_a5_resources {
	struct device *dev;
	struct device_link *camss_link;
	struct dev_pm_domain_list *domains;
	struct clk_bulk_data clocks[19];
	u8 enabled_groups;
	u8 held_domains;
	u8 ipe_reset_acks[2];
	void __iomem *sierra;
	void __iomem *bps;
	void __iomem *ipe[2];
};

int camss_a5_resources_init(struct platform_device *pdev,
			    struct camss_a5_resources *res);
int camss_a5_resources_resume(struct camss_a5_resources *res);
int camss_a5_resources_suspend(struct camss_a5_resources *res);
int camss_a5_resources_release(struct camss_a5_resources *res);

int camss_a5_ipe_quiesce(struct camss_a5_resources *res);

#endif
