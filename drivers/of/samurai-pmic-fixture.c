// SPDX-License-Identifier: GPL-2.0-only
/* Temporary front-PMIC node for testing the normal PM8008 probe path. */
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>

static struct of_changeset fixture;
static struct device_node *pmic_node;
static struct device_node *bus_node;
static bool applied;

static int __init samurai_pmic_fixture_init(void)
{
	struct device_node *gpio, *s8, *existing;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	u32 reset[3];
	int ret;

	if (!of_machine_is_compatible("realme,samurai"))
		return -ENODEV;
	bus_node = of_find_node_by_path("/soc@0/geniqup@8c0000/i2c@890000");
	gpio = of_find_node_by_path("/soc@0/spmi@c440000/pmic@4/gpio@c000");
	s8 = of_find_node_by_path("/soc@0/rsc@18200000/regulators-1/smps8");
	if (!bus_node || !gpio || !s8 || !gpio->phandle || !s8->phandle) {
		ret = -ENODEV;
		goto put_nodes;
	}
	existing = of_get_child_by_name(bus_node, "pmic");
	if (existing) {
		of_node_put(existing);
		ret = -EBUSY;
		goto put_nodes;
	}
	adapter = of_find_i2c_adapter_by_node(bus_node);
	if (!adapter) {
		ret = -ENODEV;
		goto put_nodes;
	}
	i2c_put_adapter(adapter);
	reset[0] = gpio->phandle;
	reset[1] = 4; /* PM8150L GPIO4, cdev offset 3. */
	reset[2] = 1; /* GPIO_ACTIVE_LOW */
	of_changeset_init(&fixture);
	pmic_node = of_changeset_create_node(&fixture, bus_node, "pmic@4");
	if (!pmic_node) {
		ret = -ENOMEM;
		goto destroy;
	}
	ret = of_changeset_add_prop_string(&fixture, pmic_node, "compatible", "qcom,pm8008");
	if (ret)
		goto destroy;
	ret = of_changeset_add_prop_u32(&fixture, pmic_node, "reg", 4);
	if (ret)
		goto destroy;
	ret = of_changeset_add_prop_u32(&fixture, pmic_node, "qcom,initial-address", 8);
	if (ret)
		goto destroy;
	ret = of_changeset_add_prop_u32_array(&fixture, pmic_node, "reset-gpios", reset,
					      ARRAY_SIZE(reset));
	if (ret)
		goto destroy;
	ret = of_changeset_add_prop_u32(&fixture, pmic_node, "vdd-l1-l2-supply", s8->phandle);
	if (ret)
		goto destroy;
	ret = of_changeset_apply(&fixture);
	applied = !of_node_check_flag(pmic_node, OF_DETACHED);
	if (ret)
		goto revert;
	client = of_find_i2c_device_by_node(pmic_node);
	if (!client) {
		ret = -ENODEV;
		goto revert;
	}
	ret = device_is_bound(&client->dev) ? 0 : -EPROBE_DEFER;
	put_device(&client->dev);
	if (ret)
		goto revert;
	pr_info("samurai PMIC fixture: normal PM8008 driver bound at 0x04\n");
	of_node_put(gpio);
	of_node_put(s8);
	return 0;
revert:
	if (applied && of_changeset_revert(&fixture))
		pr_err("samurai PMIC fixture: failed to remove live node\n");
	applied = false;
destroy:
	of_changeset_destroy(&fixture);
	of_node_put(pmic_node);
	pmic_node = NULL;
put_nodes:
	of_node_put(gpio);
	of_node_put(s8);
	of_node_put(bus_node);
	bus_node = NULL;
	return ret;
}

static void __exit samurai_pmic_fixture_exit(void)
{
	if (applied && of_changeset_revert(&fixture))
		pr_err("samurai PMIC fixture: failed to remove live node\n");
	of_changeset_destroy(&fixture);
	of_node_put(pmic_node);
	of_node_put(bus_node);
}

module_init(samurai_pmic_fixture_init);
module_exit(samurai_pmic_fixture_exit);
MODULE_DESCRIPTION("Temporary reversible realme X2 Pro PM8008 fixture");
MODULE_LICENSE("GPL");
