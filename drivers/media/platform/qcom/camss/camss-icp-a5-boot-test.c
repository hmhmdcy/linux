// SPDX-License-Identifier: GPL-2.0-only
/* Temporary bootstrap diagnostic. No image sessions or sensor changes. */
#include <linux/firmware.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "camss-icp-a5-boot.h"
#include "camss-icp-a5-boot-test-blob.h"

static DEFINE_MUTEX(test_lock);
static struct camss_a5 *device_a5;
static char result[2048] = "{\"terminal\":false}";
static char sfr[512];
module_param_string(result, result, sizeof(result), 0444);
module_param_string(sfr, sfr, sizeof(sfr), 0444);
static int overlay_id, test_error;
static bool finished, pinned;
static const char *phase = "waiting";
static unsigned int group_devices;

static int group_device(struct device *dev, void *target)
{
	group_devices++;
	return dev == target ? 0 : -EBUSY;
}

static void update_result(void)
{
	struct camss_a5 *a5 = device_a5;

	if (!a5) {
		scnprintf(result, sizeof(result),
			  "{\"terminal\":%s,\"phase\":\"%s\",\"error\":%d,\"backing_owned\":false,\"cpu_enabled\":false}",
			  finished ? "true" : "false", phase, test_error);
		return;
	}
	strscpy(sfr, a5->sfr, sizeof(sfr));
	scnprintf(result, sizeof(result),
		  "{\"terminal\":%s,\"phase\":\"%s\",\"error\":%d,\"cleanup_error\":%d,\"pc_error\":%d,"
		  "\"cpu_release_attempted\":%s,\"init_response\":%u,\"fw_version\":%u,\"response_before\":%u,\"init_bytes\":%u,"
		  "\"init_done\":%s,\"ping_done\":%s,\"pc_done\":%s,\"reset_asserted\":%s,"
		  "\"cpu_enabled\":%s,\"irq_enabled\":%s,\"power_held\":%s,\"backing_owned\":%s,"
		  "\"mapped_bytes\":%u,\"remaining_groups\":%u,\"remaining_domains\":%u,"
		  "\"interrupts\":%u,\"messages\":%u,\"debug_packets\":%u,\"watchdogs\":%u,\"protocol_errors\":%u,"
		  "\"last_type\":%u,\"queue_state\":[%u,%u,%u,%u,%u,%u],\"image_sessions\":0}",
		  finished ? "true" : "false", phase, test_error, a5->cleanup_error, a5->pc_error,
		  a5->cpu_release_attempted ? "true" : "false", a5->init_response, a5->fw_version,
		  a5->response_before, a5->init_bytes,
		  a5->init_done ? "true" : "false", a5->ping_done ? "true" : "false",
		  a5->pc_done ? "true" : "false", a5->reset_asserted ? "true" : "false",
		  a5->cpu_enabled ? "true" : "false", a5->irq_enabled ? "true" : "false",
		  a5->power_held ? "true" : "false", camss_a5_backing_owned(a5) ? "true" : "false",
		  a5->firmware.mapped + a5->shared.mapped + a5->heap.mapped,
		  a5->resources.enabled_groups, a5->resources.held_domains,
		  a5->interrupts, a5->messages, a5->debug_packets, a5->watchdogs, a5->protocol_errors,
		  a5->last_type, a5->queue_state[0], a5->queue_state[1], a5->queue_state[2],
		  a5->queue_state[3], a5->queue_state[4], a5->queue_state[5]);
}

static int cleanup(void)
{
	int ret = device_a5 ? camss_a5_stop(device_a5) : 0;

	if (!ret && pinned && !camss_a5_backing_owned(device_a5) &&
	    !device_a5->cpu_enabled && !device_a5->power_held &&
	    !device_a5->resources.enabled_groups && !device_a5->resources.held_domains) {
		pinned = false;
		module_put(THIS_MODULE);
	}
	return ret;
}

static ssize_t cleanup_store(struct device *dev, struct device_attribute *attr,
			     const char *buffer, size_t count)
{
	int ret;

	if (!sysfs_streq(buffer, "1"))
		return -EINVAL;
	mutex_lock(&test_lock);
	ret = cleanup();
	if (ret)
		test_error = ret;
	update_result();
	mutex_unlock(&test_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(cleanup);
static struct attribute *test_attributes[] = { &dev_attr_cleanup.attr, NULL };
static const struct attribute_group test_group = { .attrs = test_attributes };
static const struct attribute_group *test_groups[] = { &test_group, NULL };

static int test_resume(struct device *dev)
{
	struct camss_a5 *a5 = dev_get_drvdata(dev);

	return camss_a5_runtime_resume(a5);
}

static int test_suspend(struct device *dev)
{
	struct camss_a5 *a5 = dev_get_drvdata(dev);

	return camss_a5_runtime_suspend(a5);
}

static const struct dev_pm_ops test_ops = {
	RUNTIME_PM_OPS(test_suspend, test_resume, NULL)
};

static int boot_test_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct firmware *firmware;
	struct camss_a5 *a5;
	struct iommu_group *group;
	struct device_node *memory;
	int ret, stop;

	a5 = devm_kzalloc(dev, sizeof(*a5), GFP_KERNEL);
	if (!a5)
		return -ENOMEM;
	mutex_lock(&test_lock);
	phase = "iommu-group";
	group_devices = 0;
	group = iommu_group_get(dev);
	ret = group ? iommu_group_for_each_dev(group, dev, group_device) : -ENODEV;
	if (group)
		iommu_group_put(group);
	if (ret || group_devices != 1) {
		test_error = ret ? ret : -EBUSY;
		finished = true;
		update_result();
		mutex_unlock(&test_lock);
		return test_error;
	}
	/* An offline fdtoverlay merge does not prove live phandle resolution. */
	phase = "firmware-reference";
	memory = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!memory) {
		test_error = -ENXIO;
		finished = true;
		update_result();
		mutex_unlock(&test_lock);
		return dev_err_probe(dev, test_error, "Firmware memory phandle is unresolved\n");
	}
	dev_info(dev, "Firmware memory resolves to %pOF, no-map=%d\n",
		 memory, of_property_present(memory, "no-map"));
	of_node_put(memory);
	phase = "resources";
	ret = camss_a5_init(pdev, a5);
	if (ret) {
		test_error = ret;
		finished = true;
		update_result();
		mutex_unlock(&test_lock);
		return ret;
	}
	device_a5 = a5;
	platform_set_drvdata(pdev, a5);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	phase = "request-firmware";
	ret = request_firmware_direct(&firmware, "qcom/sm8150/samurai-icp356.elf", dev);
	if (ret)
		goto done;
	phase = "start";
	ret = camss_a5_start(a5, firmware);
	release_firmware(firmware);
	if (ret)
		goto done;
	phase = "ping";
	ret = camss_a5_ping(a5, 0x353536706d636173ULL);
done:
	stop = cleanup();
	if (stop)
		ret = stop;
	if ((camss_a5_backing_owned(a5) || a5->cpu_enabled || a5->power_held ||
	     a5->resources.enabled_groups || a5->resources.held_domains) && !pinned) {
		__module_get(THIS_MODULE);
		pinned = true;
	}
	if (!ret && a5->pc_error)
		ret = a5->pc_error;
	test_error = ret;
	finished = true;
	if (!ret)
		phase = "done";
	update_result();
	mutex_unlock(&test_lock);
	return 0;
}

static void boot_test_remove(struct platform_device *pdev)
{
	mutex_lock(&test_lock);
	cleanup();
	pm_runtime_disable(&pdev->dev);
	/* The driver core may runtime-resume the device before ->remove. */
	if (device_a5->resources.enabled_groups || device_a5->resources.held_domains)
		camss_a5_runtime_suspend(device_a5);
	camss_a5_resources_release(&device_a5->resources);
	device_a5 = NULL;
	mutex_unlock(&test_lock);
}

static const struct of_device_id boot_test_match[] = {
	{ .compatible = "realme,samurai-a5-icp-boot-test" }, {}
};
MODULE_DEVICE_TABLE(of, boot_test_match);

static struct platform_driver boot_test_driver = {
	.probe = boot_test_probe,
	.remove = boot_test_remove,
	.driver = {
		.name = "samurai-a5-icp-boot-test",
		.of_match_table = boot_test_match,
		.pm = pm_ptr(&test_ops),
		.dev_groups = test_groups,
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
	},
};

static int __init boot_test_init(void)
{
	int ret;

	ret = platform_driver_register(&boot_test_driver);
	if (ret)
		return ret;
	ret = of_overlay_fdt_apply(a5_boot_test_overlay,
				  sizeof(a5_boot_test_overlay), &overlay_id, NULL);
	if (ret) {
		if (pinned)
			return 0;
		if (overlay_id > 0)
			of_overlay_remove(&overlay_id);
		platform_driver_unregister(&boot_test_driver);
	}
	return ret;
}

static void __exit boot_test_exit(void)
{
	of_overlay_remove(&overlay_id);
	platform_driver_unregister(&boot_test_driver);
}

module_init(boot_test_init);
module_exit(boot_test_exit);
MODULE_DESCRIPTION("Temporary SM8150 A5 firmware bootstrap diagnostic");
MODULE_LICENSE("GPL");
