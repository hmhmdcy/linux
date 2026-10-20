// SPDX-License-Identifier: GPL-2.0-only
#include <linux/device.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/iommu.h>
#include <linux/limits.h>
#include <linux/mm.h>
#include <linux/string.h>

#include "camss-icp-a5.h"

static bool firmware_iova_reserved(struct device *device)
{
	struct iommu_resv_region *region;
	LIST_HEAD(regions);
	bool found = false;

	iommu_get_resv_regions(device, &regions);
	list_for_each_entry(region, &regions, list) {
		if (region->type == IOMMU_RESV_RESERVED && !region->start &&
		    region->length >= CAMSS_A5_FW_WINDOW) {
			found = true;
			break;
		}
	}
	iommu_put_resv_regions(device, &regions);
	return found;
}

int camss_a5_fw_memory_alloc(struct device *device,
			     struct camss_a5_fw_memory *memory)
{
	struct camss_a5_fw_memory found = {};

	if (!device || !memory)
		return -EINVAL;
	if (memory->cpu || memory->mapped || memory->bytes)
		return -EBUSY;
	found.domain = iommu_get_domain_for_dev(device);
	if (!found.domain || !iommu_is_dma_domain(found.domain))
		return -ENODEV;
	/* The DMA allocator must reserve the alias before allocating any RAM;
	 * checking only that the IOVA currently looks empty is insufficient.
	 */
	if (!firmware_iova_reserved(device))
		return -EINVAL;
	found.device = device;
	found.bytes = CAMSS_A5_FW_WINDOW;
	found.cpu = dma_alloc_coherent(device, found.bytes, &found.dma, GFP_KERNEL);
	if (!found.cpu)
		return -ENOMEM;
	if (found.dma < CAMSS_A5_FW_WINDOW || found.dma > U32_MAX ||
	    found.bytes - 1 > U32_MAX - found.dma) {
		dma_free_coherent(device, found.bytes, found.cpu, found.dma);
		return -ERANGE;
	}
	memset(found.cpu, 0, found.bytes);
	*memory = found;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_fw_memory_alloc);

int camss_a5_fw_memory_unmap(struct camss_a5_fw_memory *memory)
{
	size_t unmapped;

	if (!memory)
		return -EINVAL;
	if (!memory->mapped)
		return 0;
	if (!memory->cpu ||
	    iommu_get_domain_for_dev(memory->device) != memory->domain)
		return -EXDEV;
	/* Remove from the end so mapped always describes an owned prefix,
	 * including after an error. Never free RAM still reachable by A5.
	 */
	while (memory->mapped) {
		unmapped = iommu_unmap(memory->domain,
				       memory->mapped - PAGE_SIZE, PAGE_SIZE);
		if (unmapped != PAGE_SIZE)
			return -EIO;
		memory->mapped -= PAGE_SIZE;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_fw_memory_unmap);

int camss_a5_fw_memory_map(struct camss_a5_fw_memory *memory)
{
	u32 offset;
	int ret, cleanup, prot = IOMMU_READ | IOMMU_WRITE;
	phys_addr_t physical;

	if (!memory || !memory->cpu || memory->bytes != CAMSS_A5_FW_WINDOW)
		return -EINVAL;
	if (memory->mapped)
		return -EBUSY;
	if (iommu_get_domain_for_dev(memory->device) != memory->domain)
		return -EXDEV;
	for (offset = 0; offset < memory->bytes; offset += PAGE_SIZE) {
		if (iommu_iova_to_phys(memory->domain, offset))
			return -EBUSY;
	}
	if (dev_is_dma_coherent(memory->device))
		prot |= IOMMU_CACHE;

	for (offset = 0; offset < memory->bytes; offset += PAGE_SIZE) {
		/* DMA allocations may use scattered physical pages. Translate each
		 * original DMA page instead of assuming one physical extent.
		 */
		physical = iommu_iova_to_phys(memory->domain, memory->dma + offset);
		if (!physical || !IS_ALIGNED(physical, PAGE_SIZE)) {
			ret = -EFAULT;
			goto rollback;
		}
		ret = iommu_map(memory->domain, offset, physical, PAGE_SIZE, prot, GFP_KERNEL);
		if (ret)
			goto rollback;
		memory->mapped += PAGE_SIZE;
	}
	return 0;

rollback:
	cleanup = camss_a5_fw_memory_unmap(memory);
	return cleanup ? cleanup : ret;
}
EXPORT_SYMBOL_GPL(camss_a5_fw_memory_map);

int camss_a5_fw_memory_free(struct camss_a5_fw_memory *memory)
{
	if (!memory)
		return -EINVAL;
	if (memory->mapped)
		return -EBUSY;
	if (!memory->cpu)
		return 0;
	if (iommu_get_domain_for_dev(memory->device) != memory->domain)
		return -EXDEV;
	dma_free_coherent(memory->device, memory->bytes, memory->cpu, memory->dma);
	memset(memory, 0, sizeof(*memory));
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_fw_memory_free);
