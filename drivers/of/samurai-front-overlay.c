// SPDX-License-Identifier: GPL-2.0-only
/* Development overlay only: the normal IMX471 driver performs the probe. */
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_clk.h>
#include <linux/pinctrl/consumer.h>

static int overlay_id;
module_param(overlay_id, int, 0444);
static int device_clock_error, node_clock_error, provider_clock_error;
static unsigned int clock_index;
module_param(device_clock_error, int, 0444);
module_param(node_clock_error, int, 0444);
module_param(provider_clock_error, int, 0444);
module_param(clock_index, uint, 0444);

static void inspect_clock_lookup(void)
{
	struct device_node *node;
	struct of_phandle_args spec;
	struct i2c_client *client;
	struct clk *clock;

	node = of_find_node_by_path("/soc@0/cci@ac4b000/i2c-bus@1/camera@10");
	if (!node)
		return;
	client = of_find_i2c_device_by_node(node);
	if (client) {
		clock = clk_get(&client->dev, NULL);
		device_clock_error = PTR_ERR_OR_ZERO(clock);
		if (!IS_ERR(clock))
			clk_put(clock);
		put_device(&client->dev);
	}
	clock = of_clk_get(node, 0);
	node_clock_error = PTR_ERR_OR_ZERO(clock);
	if (!IS_ERR(clock))
		clk_put(clock);
	if (!of_parse_phandle_with_args(node, "clocks", "#clock-cells", 0, &spec)) {
		clock_index = spec.args[0];
		clock = of_clk_get_from_provider(&spec);
		provider_clock_error = PTR_ERR_OR_ZERO(clock);
		if (!IS_ERR(clock))
			clk_put(clock);
		of_node_put(spec.np);
	}
	of_node_put(node);
}

static int restore_sleep_pins(void)
{
	struct device_node *node;
	struct pinctrl *pins;
	struct device *dev;
	int ret;

	node = of_find_node_by_path("/samurai-front-restore");
	if (!node)
		return -ENODEV;
	dev = root_device_register("samurai-front-restore");
	if (IS_ERR(dev)) {
		ret = PTR_ERR(dev);
		goto put_node;
	}
	device_set_node(dev, of_fwnode_handle(node));
	pins = pinctrl_get_select_default(dev);
	ret = PTR_ERR_OR_ZERO(pins);
	if (!ret)
		pinctrl_put(pins);
	device_set_node(dev, NULL);
	root_device_unregister(dev);
put_node:
	of_node_put(node);
	return ret;
}

static int __init samurai_front_overlay_init(void)
{
	const struct firmware *firmware;
	struct device_node *node;
	struct device *dev;
	int ret;

	if (!of_machine_is_compatible("realme,samurai"))
		return -ENODEV;
	node = of_find_node_by_path("/soc@0/cci@ac4b000/i2c-bus@1/camera@10");
	if (node) {
		of_node_put(node);
		return -EBUSY;
	}
	node = of_find_node_by_path("/soc@0/rsc@18200000/regulators-1/ldo1");
	if (!node)
		return -ENODEV;
	ret = node->phandle ? 0 : -EINVAL;
	of_node_put(node);
	if (ret)
		return ret;
	dev = root_device_register("samurai-front-overlay");
	if (IS_ERR(dev))
		return PTR_ERR(dev);
	ret = request_firmware_direct(&firmware, "samurai/k196-front.dtbo", dev);
	if (ret)
		goto unregister;
	ret = of_overlay_fdt_apply(firmware->data, firmware->size, &overlay_id, NULL);
	release_firmware(firmware);
	if (!ret)
		inspect_clock_lookup();
	if (ret && overlay_id)
		of_overlay_remove(&overlay_id);
unregister:
	root_device_unregister(dev);
	return ret;
}

static void __exit samurai_front_overlay_exit(void)
{
	int ret;

	ret = restore_sleep_pins();
	pr_info("samurai front overlay: sleep-pin restore=%d\n", ret);
	if (overlay_id && of_overlay_remove(&overlay_id))
		pr_err("samurai front overlay: failed to remove live overlay\n");
}

module_init(samurai_front_overlay_init);
module_exit(samurai_front_overlay_exit);
MODULE_DESCRIPTION("Temporary realme X2 Pro front-camera OF resource overlay");
MODULE_LICENSE("GPL");
