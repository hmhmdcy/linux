// SPDX-License-Identifier: GPL-2.0-only
/* SM8150 A5/BPS/IPE resources; CPAS remains owned by the CAMSS supplier. */
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "camss-icp-a5-resources.h"

static const char * const domain_names[] = { "bps", "ipe0", "ipe1" };
static const char * const clock_names[] = {
	"fast_ahb_src", "icp_ahb", "icp_src", "icp",
	"bps_ahb", "bps_areg", "bps_axi", "bps_src", "bps",
	"ipe0_ahb", "ipe0_areg", "ipe0_axi", "ipe_src", "ipe0",
	"ipe1_ahb", "ipe1_areg", "ipe1_axi", "ipe1_src", "ipe1",
};
static const unsigned int group_offset[] = { 0, 4, 9, 14 };
static const unsigned int group_count[] = { 4, 5, 5, 5 };

static void camss_a5_delete_link(void *data)
{
	device_link_del(data);
}

int camss_a5_resources_init(struct platform_device *pdev,
			    struct camss_a5_resources *res)
{
	const struct dev_pm_domain_attach_data attach = {
		.pd_names = domain_names,
		.num_pd_names = ARRAY_SIZE(domain_names),
		.pd_flags = PD_FLAG_NO_DEV_LINK | PD_FLAG_DETACH_POWER_OFF,
	};
	struct device *dev = &pdev->dev;
	struct platform_device *supplier;
	struct device_node *node;
	unsigned int i;
	int ret;

	if (res->dev)
		return -EBUSY;
	node = of_parse_phandle(dev->of_node, "qcom,camss", 0);
	if (!node)
		return -EINVAL;
	supplier = of_find_device_by_node(node);
	of_node_put(node);
	if (!supplier)
		return -EPROBE_DEFER;
	if (!device_is_bound(&supplier->dev)) {
		put_device(&supplier->dev);
		return -EPROBE_DEFER;
	}
	res->camss_link = device_link_add(dev, &supplier->dev,
					DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
	put_device(&supplier->dev);
	if (!res->camss_link)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, camss_a5_delete_link, res->camss_link);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(clock_names); i++)
		res->clocks[i].id = clock_names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(res->clocks), res->clocks);
	if (ret)
		return ret;
	res->sierra = devm_platform_ioremap_resource_byname(pdev, "sierra");
	if (IS_ERR(res->sierra))
		return PTR_ERR(res->sierra);
	res->bps = devm_platform_ioremap_resource_byname(pdev, "bps");
	if (IS_ERR(res->bps))
		return PTR_ERR(res->bps);
	res->ipe[0] = devm_platform_ioremap_resource_byname(pdev, "ipe0");
	if (IS_ERR(res->ipe[0]))
		return PTR_ERR(res->ipe[0]);
	res->ipe[1] = devm_platform_ioremap_resource_byname(pdev, "ipe1");
	if (IS_ERR(res->ipe[1]))
		return PTR_ERR(res->ipe[1]);
	ret = devm_pm_domain_attach_list(dev, &attach, &res->domains);
	if (ret < 0)
		return ret;
	if (!res->domains || res->domains->num_pds != ARRAY_SIZE(domain_names))
		return -ENODEV;
	res->dev = dev;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_resources_init);

int camss_a5_resources_suspend(struct camss_a5_resources *res)
{
	int group, ret, first_error = 0;

	if (!res->dev)
		return -EINVAL;
	for (group = 3; group >= 0; group--) {
		if (res->enabled_groups & BIT(group)) {
			clk_bulk_disable_unprepare(group_count[group],
						   res->clocks + group_offset[group]);
			res->enabled_groups &= ~BIT(group);
		}
		if (group && (res->held_domains & BIT(group - 1))) {
			ret = pm_runtime_put_sync(res->domains->pd_devs[group - 1]);
			res->held_domains &= ~BIT(group - 1);
			if (ret < 0 && !first_error)
				first_error = ret;
		}
	}
	return first_error;
}
EXPORT_SYMBOL_GPL(camss_a5_resources_suspend);

int camss_a5_resources_resume(struct camss_a5_resources *res)
{
	/* Original SM8150 SVS rates; IPE1 shares IPE0's source. */
	static const unsigned int rate_index[] = { 0, 2, 7, 12 };
	static const unsigned long rates[] = {
		200000000, 400000000, 400000000, 475000000,
	};
	unsigned int group, i;
	int ret, unwind;

	if (!res->dev)
		return -EINVAL;
	if (res->enabled_groups || res->held_domains)
		return -EBUSY;
	for (i = 0; i < ARRAY_SIZE(rates); i++) {
		ret = clk_set_rate(res->clocks[rate_index[i]].clk, rates[i]);
		if (ret)
			return ret;
	}
	for (group = 0; group < ARRAY_SIZE(group_offset); group++) {
		if (group) {
			ret = pm_runtime_resume_and_get(res->domains->pd_devs[group - 1]);
			if (ret < 0)
				goto unwind;
			res->held_domains |= BIT(group - 1);
		}
		ret = clk_bulk_prepare_enable(group_count[group],
					    res->clocks + group_offset[group]);
		if (ret)
			goto unwind;
		res->enabled_groups |= BIT(group);
	}
	return 0;
unwind:
	unwind = camss_a5_resources_suspend(res);
	if (unwind)
		dev_err(res->dev, "A5 resource unwind failed: %d\n", unwind);
	return ret;
}
EXPORT_SYMBOL_GPL(camss_a5_resources_resume);

int camss_a5_resources_release(struct camss_a5_resources *res)
{
	if (res->enabled_groups || res->held_domains)
		return -EBUSY;
	/* devres releases the link, clocks, mappings and attached domains. */
	res->dev = NULL;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_resources_release);
