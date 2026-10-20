// SPDX-License-Identifier: GPL-2.0-only
/* Temporary memory-only diagnostic: no A5 registers or firmware execution. */
#include <linux/device.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/iommu.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/string.h>

#include "camss-icp-a5.h"
#include "camss-icp-a5-memory-test-blob.h"

static DEFINE_MUTEX(test_lock);
static struct camss_a5_fw_memory memory;
static char result[1024] = "{\"terminal\":false}";
module_param_string(result, result, sizeof(result), 0444);
static int overlay_id;
static unsigned int group_devices, verified, removed;
static int test_error;
static const char *phase = "waiting";
static bool pinned, finished;
static dma_addr_t backing_dma;

static void update_result(void)
{
	scnprintf(result, sizeof(result),
		  "{\"terminal\":%s,\"error\":%d,\"phase\":\"%s\","
		  "\"group_devices\":%u,\"verified_pages\":%u,\"removed_pages\":%u,"
		  "\"mapped_bytes\":%u,\"backing_owned\":%s,\"backing_dma\":%llu,"
		  "\"firmware_started\":false}",
		  finished ? "true" : "false", test_error, phase, group_devices,
		  verified, removed, memory.mapped, memory.cpu ? "true" : "false",
		  (unsigned long long)backing_dma);
}

static int cleanup_memory(void)
{
	int ret;

	ret = camss_a5_fw_memory_unmap(&memory);
	if (ret)
		return ret;
	ret = camss_a5_fw_memory_free(&memory);
	if (!ret && pinned) {
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
	ret = cleanup_memory();
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

static int check_group_device(struct device *dev, void *target)
{
	group_devices++;
	return dev == target ? 0 : -EBUSY;
}

static int memory_test_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iommu_group *group;
	void *scratch = NULL;
	dma_addr_t scratch_dma = 0;
	u32 offset;
	int ret, cleanup;

	mutex_lock(&test_lock);
	phase = "group";
	group = iommu_group_get(dev);
	if (!group) {
		ret = -ENODEV;
		goto done;
	}
	ret = iommu_group_for_each_dev(group, dev, check_group_device);
	iommu_group_put(group);
	if (ret || group_devices != 1) {
		ret = -EBUSY;
		goto done;
	}
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		goto done;
	phase = "allocate";
	ret = camss_a5_fw_memory_alloc(dev, &memory);
	if (ret)
		goto done;
	backing_dma = memory.dma;
	for (offset = 0; offset < memory.bytes; offset += PAGE_SIZE) {
		*(u32 *)((u8 *)memory.cpu + offset) = offset ^ 0x35400000;
		*(u32 *)((u8 *)memory.cpu + offset + PAGE_SIZE - 4) = ~offset;
	}
	phase = "map";
	ret = camss_a5_fw_memory_map(&memory);
	if (ret)
		goto done;
	phase = "verify";
	for (offset = 0; offset < memory.bytes; offset += PAGE_SIZE) {
		phys_addr_t original = iommu_iova_to_phys(memory.domain, memory.dma + offset);
		phys_addr_t alias = iommu_iova_to_phys(memory.domain, offset);

		if (!original || original != alias ||
		    *(u32 *)((u8 *)memory.cpu + offset) != (offset ^ 0x35400000) ||
		    *(u32 *)((u8 *)memory.cpu + offset + PAGE_SIZE - 4) != ~offset) {
			ret = -EIO;
			goto done;
		}
		verified++;
	}
	phase = "dma-allocation-with-alias";
	scratch = dma_alloc_coherent(dev, PAGE_SIZE, &scratch_dma, GFP_KERNEL);
	if (!scratch) {
		ret = -ENOMEM;
		goto done;
	}
	if (scratch_dma < CAMSS_A5_FW_WINDOW || scratch_dma > U32_MAX - PAGE_SIZE + 1) {
		ret = -ERANGE;
		goto done;
	}
	memset(scratch, 0x54, PAGE_SIZE);
	phase = "unmap";
	ret = camss_a5_fw_memory_unmap(&memory);
	if (ret)
		goto done;
	for (offset = 0; offset < memory.bytes; offset += PAGE_SIZE) {
		if (iommu_iova_to_phys(memory.domain, offset)) {
			ret = -EIO;
			goto done;
		}
		removed++;
	}
	phase = "done";
done:
	if (scratch)
		dma_free_coherent(dev, PAGE_SIZE, scratch, scratch_dma);
	cleanup = cleanup_memory();
	if (cleanup)
		ret = cleanup;
	if (memory.cpu && !pinned) {
		/* A failed unmap must keep both the backing and this cleanup code. */
		__module_get(THIS_MODULE);
		pinned = true;
	}
	test_error = ret;
	finished = true;
	update_result();
	mutex_unlock(&test_lock);
	/* Keep the diagnostic device bound so cleanup can be retried if needed. */
	return 0;
}

static void memory_test_remove(struct platform_device *pdev)
{
	mutex_lock(&test_lock);
	cleanup_memory();
	mutex_unlock(&test_lock);
}

static const struct of_device_id memory_test_match[] = {
	{ .compatible = "realme,samurai-a5-icp-memory-test" },
	{}
};
MODULE_DEVICE_TABLE(of, memory_test_match);

static struct platform_driver memory_test_driver = {
	.probe = memory_test_probe,
	.remove = memory_test_remove,
	.driver = {
		.name = "samurai-a5-icp-memory-test",
		.of_match_table = memory_test_match,
		.dev_groups = test_groups,
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
	},
};

static int __init memory_test_init(void)
{
	int ret;

	ret = platform_driver_register(&memory_test_driver);
	if (ret)
		return ret;
	ret = of_overlay_fdt_apply(a5_memory_test_overlay,
				  sizeof(a5_memory_test_overlay), &overlay_id, NULL);
	if (ret) {
		if (memory.cpu) {
			test_error = ret;
			finished = true;
			update_result();
			return 0;
		}
		if (overlay_id > 0)
			of_overlay_remove(&overlay_id);
		platform_driver_unregister(&memory_test_driver);
	}
	return ret;
}

static void __exit memory_test_exit(void)
{
	of_overlay_remove(&overlay_id);
	platform_driver_unregister(&memory_test_driver);
}

module_init(memory_test_init);
module_exit(memory_test_exit);
MODULE_DESCRIPTION("Temporary A5 ICP memory/IOMMU diagnostic; no firmware boot");
MODULE_LICENSE("GPL");
