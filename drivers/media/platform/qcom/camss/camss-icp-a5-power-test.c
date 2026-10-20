// SPDX-License-Identifier: GPL-2.0-only
/* Temporary resource lifecycle test. Never writes A5 control/reset registers. */
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "camss-icp-a5-resources.h"
#include "camss-icp-a5-power-test-blob.h"

static char result[2048] = "{\"terminal\":false}";
module_param_string(result, result, sizeof(result), 0444);
static int overlay_id;

static int power_resume(struct device *dev)
{
	return camss_a5_resources_resume(dev_get_drvdata(dev));
}

static int power_suspend(struct device *dev)
{
	return camss_a5_resources_suspend(dev_get_drvdata(dev));
}

static const struct dev_pm_ops power_ops = {
	RUNTIME_PM_OPS(power_suspend, power_resume, NULL)
};

static int power_test_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct camss_a5_resources *res;
	u32 version = 0, control = 0, status = 0;
	unsigned int powered = 0, domains = 0, i;
	unsigned long rates[19] = {};
	int ret, suspend_error = 0;

	res = devm_kzalloc(dev, sizeof(*res), GFP_KERNEL);
	if (!res)
		return -ENOMEM;
	ret = camss_a5_resources_init(pdev, res);
	if (ret) {
		scnprintf(result, sizeof(result),
			  "{\"terminal\":true,\"phase\":\"resources-init\",\"error\":%d}", ret);
		return ret;
	}
	platform_set_drvdata(pdev, res);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	ret = pm_runtime_resume_and_get(dev);
	if (ret >= 0) {
		powered = res->enabled_groups;
		domains = res->held_domains;
		version = readl(res->sierra);
		control = readl(res->sierra + 8);
		status = readl(res->sierra + 0x200);
		for (i = 0; i < ARRAY_SIZE(rates); i++)
			rates[i] = clk_get_rate(res->clocks[i].clk);
		ret = version && version != U32_MAX && !(control & BIT(9)) &&
			powered == 15 && domains == 7 ? 0 : -EIO;
		suspend_error = pm_runtime_put_sync_suspend(dev);
		if (suspend_error > 0)
			suspend_error = 0;
	}
	scnprintf(result, sizeof(result),
		  "{\"terminal\":true,\"phase\":\"done\",\"error\":%d,\"suspend_error\":%d,"
		  "\"powered_groups\":%u,\"powered_domains\":%u,\"remaining_groups\":%u,"
		  "\"remaining_domains\":%u,\"version\":%u,\"control\":%u,\"status\":%u,"
		  "\"rates\":[%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu],"
		  "\"runtime_suspended\":%s,\"firmware_started\":false}",
		  ret, suspend_error, powered, domains, res->enabled_groups,
		  res->held_domains, version, control, status,
		  rates[0], rates[1], rates[2], rates[3], rates[4], rates[5], rates[6],
		  rates[7], rates[8], rates[9], rates[10], rates[11], rates[12],
		  rates[13], rates[14], rates[15], rates[16], rates[17], rates[18],
		  pm_runtime_status_suspended(dev) ? "true" : "false");
	return 0;
}

static void power_test_remove(struct platform_device *pdev)
{
	struct camss_a5_resources *res = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	if (res->enabled_groups || res->held_domains)
		camss_a5_resources_suspend(res);
	camss_a5_resources_release(res);
}

static const struct of_device_id power_test_match[] = {
	{ .compatible = "realme,samurai-a5-icp-power-test" },
	{}
};
MODULE_DEVICE_TABLE(of, power_test_match);

static struct platform_driver power_test_driver = {
	.probe = power_test_probe,
	.remove = power_test_remove,
	.driver = {
		.name = "samurai-a5-icp-power-test",
		.of_match_table = power_test_match,
		.pm = pm_ptr(&power_ops),
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
	},
};

static int __init power_test_init(void)
{
	int ret;

	ret = platform_driver_register(&power_test_driver);
	if (ret)
		return ret;
	ret = of_overlay_fdt_apply(a5_power_test_overlay,
				  sizeof(a5_power_test_overlay), &overlay_id, NULL);
	if (ret) {
		if (overlay_id > 0)
			of_overlay_remove(&overlay_id);
		platform_driver_unregister(&power_test_driver);
	}
	return ret;
}

static void __exit power_test_exit(void)
{
	of_overlay_remove(&overlay_id);
	platform_driver_unregister(&power_test_driver);
}

module_init(power_test_init);
module_exit(power_test_exit);
MODULE_DESCRIPTION("Temporary A5/BPS/IPE resource lifecycle diagnostic");
MODULE_LICENSE("GPL");
