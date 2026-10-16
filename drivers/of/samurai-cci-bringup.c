// SPDX-License-Identifier: GPL-2.0-only
/* Temporary live CCI1 activation for the realme X2 Pro camera port. */
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

static struct of_changeset cci_changes;
static bool applied;

static int __init samurai_cci_bringup_init(void)
{
	struct platform_device *pdev;
	struct device_node *node;
	const char *status;
	int ret;

	if (!of_machine_is_compatible("realme,samurai"))
		return -ENODEV;

	node = of_find_node_by_path("/soc@0/cci@ac4b000");
	if (!node)
		return -ENODEV;

	if (!of_device_is_compatible(node, "qcom,sm8150-cci") ||
	    of_property_read_string(node, "status", &status) ||
	    strcmp(status, "disabled") ||
	    of_node_check_flag(node, OF_POPULATED)) {
		ret = -EBUSY;
		goto put_node;
	}

	pdev = of_find_device_by_node(node);
	if (pdev) {
		platform_device_put(pdev);
		ret = -EBUSY;
		goto put_node;
	}

	of_changeset_init(&cci_changes);
	ret = of_changeset_update_prop_string(&cci_changes, node, "status", "okay");
	if (ret)
		goto destroy;
	ret = of_changeset_apply(&cci_changes);
	/* A notifier can fail after the single property update took effect. */
	applied = of_device_is_available(node);
	if (ret)
		goto revert;
	if (!applied) {
		ret = -EIO;
		goto destroy;
	}

	pdev = of_find_device_by_node(node);
	if (!pdev) {
		ret = -ENODEV;
		goto revert;
	}
	if (!device_is_bound(&pdev->dev)) {
		platform_device_put(pdev);
		ret = -EPROBE_DEFER;
		goto revert;
	}
	dev_info(&pdev->dev, "samurai camera bringup: CCI1 live, no sensor power changes\n");
	platform_device_put(pdev);
	of_node_put(node);
	return 0;

revert:
	if (applied && of_changeset_revert(&cci_changes))
		pr_err("samurai camera bringup: failed to restore disabled CCI1\n");
	applied = false;
destroy:
	of_changeset_destroy(&cci_changes);
put_node:
	of_node_put(node);
	return ret;
}

static void __exit samurai_cci_bringup_exit(void)
{
	if (applied && of_changeset_revert(&cci_changes))
		pr_err("samurai camera bringup: failed to restore disabled CCI1\n");
	of_changeset_destroy(&cci_changes);
}

module_init(samurai_cci_bringup_init);
module_exit(samurai_cci_bringup_exit);

MODULE_DESCRIPTION("Temporary reversible realme X2 Pro CCI1 activation");
MODULE_AUTHOR("Samurai Linux port contributors");
MODULE_LICENSE("GPL");
