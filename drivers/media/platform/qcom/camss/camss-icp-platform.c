// SPDX-License-Identifier: GPL-2.0-only
/* SM8150 ICP/IPE platform owner. Shared CAMSS resources have one module owner. */
#include <linux/device/bus.h>
#include <linux/firmware.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>

#include "camss-icp-a5-boot.h"
#include "camss-icp-a5-m2m.h"

#define ICP_FIRMWARE "qcom/sm8150/camera-icp.elf"

struct camss_icp {
	struct camss_a5 a5;
	struct camss_a5_m2m *m2m;
	struct mutex lock;
};

struct icp_group_check {
	struct device *owner;
	unsigned int count;
};

static int check_group_device(struct device *dev, void *data)
{
	struct icp_group_check *check = data;

	check->count++;
	return dev == check->owner ? 0 : -EBUSY;
}

static int check_dma_group(struct device *dev)
{
	struct icp_group_check check = { .owner = dev };
	struct iommu_group *group;
	int ret;

	group = iommu_group_get(dev);
	if (!group)
		return -ENODEV;
	ret = iommu_group_for_each_dev(group, &check, check_group_device);
	iommu_group_put(group);
	if (ret)
		return ret;
	return check.count == 1 ? 0 : -EBUSY;
}

static ssize_t m2m_status_show(struct device *dev, struct device_attribute *attr,
			       char *buffer)
{
	struct camss_icp *icp = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&icp->lock);
	ret = camss_a5_m2m_status(icp->m2m, buffer, PAGE_SIZE);
	mutex_unlock(&icp->lock);
	return ret;
}
static DEVICE_ATTR_RO(m2m_status);

/* Diagnostic counters contain no firmware RAM, images or tuning data. */
static ssize_t state_show(struct device *dev, struct device_attribute *attr,
			  char *buffer)
{
	struct camss_icp *icp = dev_get_drvdata(dev);
	struct camss_a5 *a5 = &icp->a5;
	ssize_t ret;

	mutex_lock(&a5->lock);
	ret = sysfs_emit(buffer,
		"{\"engine_dma_capable\":%s,\"cpu_release_attempted\":%s,"
		"\"backing_owned\":%s,\"cpu_enabled\":%s,\"irq_enabled\":%s,"
		"\"power_held\":%s,\"cleanup_error\":%d,\"pc_error\":%d,"
		"\"remaining_groups\":%u,\"remaining_domains\":%u,"
		"\"mapped_bytes\":%u,\"ipe_dma_pending\":%s,\"request_fw_mapped\":%s,"
		"\"imported_buffers\":%u,\"imported_bytes\":%llu,"
		"\"init_response\":%u,\"init_done\":%s,\"pc_done\":%s,"
		"\"session_creates\":%u,\"session_destroys\":%u,\"session_peak\":%u,"
		"\"frames_completed\":%u,\"direct_frames\":%u,\"copy_frames\":%u,"
		"\"interrupts\":%u,\"messages\":%u,\"watchdogs\":%u,\"protocol_errors\":%u}\n",
		a5->engine_dma_capable ? "true" : "false",
		a5->cpu_release_attempted ? "true" : "false",
		camss_a5_backing_owned(a5) ? "true" : "false",
		a5->cpu_enabled ? "true" : "false", a5->irq_enabled ? "true" : "false",
		a5->power_held ? "true" : "false", a5->cleanup_error, a5->pc_error,
		a5->resources.enabled_groups, a5->resources.held_domains,
		a5->firmware.mapped + a5->shared.mapped + a5->heap.mapped +
		 a5->config.mapped + a5->request.mapped + a5->image_input.mapped +
		 a5->image_output.mapped,
		a5->ipe_dma_pending ? "true" : "false",
		a5->request_fw_mapped ? "true" : "false",
		!!a5->dma_input.buffer + !!a5->dma_output.buffer,
		(unsigned long long)a5->dma_input.bytes + a5->dma_output.bytes,
		a5->init_response, a5->init_done ? "true" : "false",
		a5->pc_done ? "true" : "false", a5->session_creates,
		a5->session_destroys, a5->session_peak, a5->frames_completed,
		a5->direct_frames, a5->copy_frames, a5->interrupts, a5->messages,
		a5->watchdogs, a5->protocol_errors);
	mutex_unlock(&a5->lock);
	return ret;
}
static DEVICE_ATTR_RO(state);

/* Retry teardown retained after a failed firmware command/reset. Open
 * contexts are rejected by the adapter; this cannot free active DMA.
 */
static ssize_t cleanup_store(struct device *dev, struct device_attribute *attr,
			     const char *buffer, size_t count)
{
	struct camss_icp *icp = dev_get_drvdata(dev);
	int ret;

	if (!sysfs_streq(buffer, "1"))
		return -EINVAL;
	mutex_lock(&icp->lock);
	ret = camss_a5_m2m_cleanup(icp->m2m);
	mutex_unlock(&icp->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(cleanup);

static struct attribute *icp_attrs[] = {
	&dev_attr_m2m_status.attr, &dev_attr_state.attr, &dev_attr_cleanup.attr, NULL
};
ATTRIBUTE_GROUPS(icp);

static int icp_runtime_resume(struct device *dev)
{
	struct camss_icp *icp = dev_get_drvdata(dev);

	return camss_a5_runtime_resume(&icp->a5);
}

static int icp_runtime_suspend(struct device *dev)
{
	struct camss_icp *icp = dev_get_drvdata(dev);

	return camss_a5_runtime_suspend(&icp->a5);
}

static const struct dev_pm_ops icp_pm_ops = {
	RUNTIME_PM_OPS(icp_runtime_suspend, icp_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
};

static int camss_icp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const char *firmware = ICP_FIRMWARE;
	struct camss_icp *icp;
	int ret;

	ret = check_dma_group(dev);
	if (ret)
		return dev_err_probe(dev, ret, "ICP needs its own DMA group\n");
	icp = devm_kzalloc(dev, sizeof(*icp), GFP_KERNEL);
	if (!icp)
		return -ENOMEM;
	mutex_init(&icp->lock);
	ret = camss_a5_init(pdev, &icp->a5);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to acquire ICP resources\n");
	if (!icp->a5.engine_dma_capable)
		return dev_err_probe(dev, -EINVAL, "Incomplete ICP/BPS/IPE stream ownership\n");
	ret = of_property_read_string(dev->of_node, "firmware-name", &firmware);
	if (ret && ret != -EINVAL)
		return dev_err_probe(dev, ret, "Invalid firmware-name\n");
	if (!firmware[0])
		return dev_err_probe(dev, -EINVAL, "Empty firmware-name\n");
	platform_set_drvdata(pdev, icp);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	icp->m2m = camss_a5_m2m_register(&icp->a5, THIS_MODULE, firmware, device_is_bound);
	if (IS_ERR(icp->m2m)) {
		ret = PTR_ERR(icp->m2m);
		pm_runtime_disable(dev);
		camss_a5_resources_release(&icp->a5.resources);
		return dev_err_probe(dev, ret, "Failed to register IPE queues\n");
	}
	dev_info(dev, "SM8150 IPE ready; firmware starts on a queued job\n");
	return 0;
}

static void camss_icp_remove(struct platform_device *pdev)
{
	struct camss_icp *icp = platform_get_drvdata(pdev);
	int ret;

	/* Open contexts, exports and retained DMA pin this same qcom-camss
 * module. User unbind is suppressed for both supplier and consumer;
 * module exit unregisters ICP before the CAMSS supplier.
 */
	mutex_lock(&icp->lock);
	ret = camss_a5_m2m_unregister(icp->m2m);
	if (WARN_ON(ret))
		goto unlock;
	icp->m2m = NULL;
	pm_runtime_disable(&pdev->dev);
	if (icp->a5.resources.enabled_groups || icp->a5.resources.held_domains) {
		ret = camss_a5_runtime_suspend(&icp->a5);
		if (WARN_ON(ret))
			goto unlock;
	}
	WARN_ON(camss_a5_resources_release(&icp->a5.resources));
unlock:
	mutex_unlock(&icp->lock);
}

static const struct of_device_id camss_icp_match[] = {
	{ .compatible = "qcom,sm8150-camss-icp" }, {}
};
MODULE_DEVICE_TABLE(of, camss_icp_match);
MODULE_FIRMWARE(ICP_FIRMWARE);

/* Registered together with camss_driver by the qcom-camss module. */
struct platform_driver camss_icp_driver = {
	.probe = camss_icp_probe,
	.remove = camss_icp_remove,
	.driver = {
		.name = "qcom-sm8150-icp",
		.of_match_table = camss_icp_match,
		.pm = pm_ptr(&icp_pm_ops),
		.dev_groups = icp_groups,
		.suppress_bind_attrs = true,
	},
};
