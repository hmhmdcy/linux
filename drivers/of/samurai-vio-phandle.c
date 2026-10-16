// SPDX-License-Identifier: GPL-2.0-only
/* Temporary live phandle for the existing camera VIO regulator, including RPMH initial state. */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include "of_private.h"

static struct of_changeset vio_changes;
static struct device_node *vio_node, *probe_node, *root_node;
static bool applied;
static unsigned int assigned_handle;
static int supply_microvolts;
static int supply_enable_state;
module_param(assigned_handle, uint, 0444);
module_param(supply_microvolts, int, 0444);
module_param(supply_enable_state, int, 0444);

static void destroy_changes(void)
{
	of_changeset_destroy(&vio_changes);
	of_node_put(probe_node);
	of_node_put(vio_node);
	of_node_put(root_node);
}

static int __init samurai_vio_phandle_init(void)
{
	struct device_node *node, *found;
	struct regulator *supply;
	struct device *dev;
	unsigned int maximum = 0;
	int ret;

	if (!of_machine_is_compatible("realme,samurai"))
		return -ENODEV;
	vio_node = of_find_node_by_path("/soc@0/rsc@18200000/regulators-1/ldo1");
	root_node = of_find_node_by_path("/");
	if (!vio_node || !root_node) {
		ret = -ENODEV;
		goto put_nodes;
	}
	if (vio_node->phandle || of_find_property(vio_node, "phandle", NULL) ||
	    of_find_property(vio_node, "compatible", NULL) ||
	    of_get_child_count(vio_node) ||
	    !of_node_name_eq(vio_node, "ldo1") ||
	    !of_device_is_compatible(vio_node->parent, "qcom,pm8150l-rpmh-regulators")) {
		ret = -EBUSY;
		goto put_nodes;
	}
	for_each_node_with_property(node, "phandle") {
		if (node->phandle != OF_PHANDLE_ILLEGAL)
			maximum = max(maximum, node->phandle);
	}
	for_each_node_with_property(node, "linux,phandle") {
		if (node->phandle != OF_PHANDLE_ILLEGAL)
			maximum = max(maximum, node->phandle);
	}
	if (maximum >= OF_PHANDLE_ILLEGAL - 1) {
		ret = -EOVERFLOW;
		goto put_nodes;
	}
	assigned_handle = maximum + 1;
	of_changeset_init(&vio_changes);
	/* Reattachment updates the OF phandle cache through normal OF APIs. */
	ret = of_changeset_detach_node(&vio_changes, vio_node);
	if (ret)
		goto destroy;
	ret = of_changeset_add_prop_u32(&vio_changes, vio_node, "phandle", assigned_handle);
	if (ret)
		goto destroy;
	ret = of_changeset_attach_node(&vio_changes, vio_node);
	if (ret)
		goto destroy;
	probe_node = of_changeset_create_node(&vio_changes, root_node, "samurai-vio-lookup");
	if (!probe_node) {
		ret = -ENOMEM;
		goto destroy;
	}
	ret = of_changeset_add_prop_u32(&vio_changes, probe_node, "vif-supply", assigned_handle);
	if (ret)
		goto destroy;
	ret = of_changeset_apply(&vio_changes);
	applied = vio_node->phandle == assigned_handle &&
		  !of_node_check_flag(vio_node, OF_DETACHED);
	if (ret)
		goto revert;
	found = of_find_node_by_phandle(assigned_handle);
	if (found != vio_node) {
		of_node_put(found);
		ret = -ENODEV;
		goto revert;
	}
	of_node_put(found);
	dev = root_device_register("samurai-vio-lookup");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		goto revert;
	}
	supply = of_regulator_get(dev, probe_node, "vif");
	if (IS_ERR(supply)) {
		ret = PTR_ERR(supply);
	} else {
		supply_microvolts = regulator_get_voltage(supply);
		supply_enable_state = regulator_is_enabled(supply);
		/* RPMH starts with -EINVAL until the first real enable request. */
		if (supply_microvolts != 1800000)
			ret = -EINVAL;
		else if (supply_enable_state == 0 || supply_enable_state == -EINVAL)
			ret = 0;
		else
			ret = supply_enable_state < 0 ? supply_enable_state : -EBUSY;
		regulator_put(supply);
	}
	root_device_unregister(dev);
	if (ret)
		goto revert;
	pr_info("samurai VIO: exact 1.8V OF supply, state %d, phandle %u\n",
		supply_enable_state, assigned_handle);
	return 0;
revert:
	if (applied && of_changeset_revert(&vio_changes))
		pr_err("samurai VIO: failed to restore original device tree\n");
	applied = false;
destroy:
	destroy_changes();
	return ret;
put_nodes:
	of_node_put(vio_node);
	of_node_put(root_node);
	return ret;
}

static void __exit samurai_vio_phandle_exit(void)
{
	if (applied && of_changeset_revert(&vio_changes))
		pr_err("samurai VIO: failed to restore original device tree\n");
	destroy_changes();
}

module_init(samurai_vio_phandle_init);
module_exit(samurai_vio_phandle_exit);
MODULE_DESCRIPTION("Temporary realme X2 Pro camera VIO OF lookup fixture");
MODULE_LICENSE("GPL");
