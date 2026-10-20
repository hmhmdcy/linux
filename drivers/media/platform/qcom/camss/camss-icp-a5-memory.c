// SPDX-License-Identifier: GPL-2.0-only
#include <linux/device.h>
#include <linux/dma-map-ops.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/iommu.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/mm.h>
#include <linux/string.h>

#include "camss-icp-a5.h"

static bool iova_reserved(struct device *device, u32 iova, u32 bytes)
{
	struct iommu_resv_region *region;
	LIST_HEAD(regions);
	bool found = false;

	iommu_get_resv_regions(device, &regions);
	list_for_each_entry(region, &regions, list) {
		if (region->type == IOMMU_RESV_RESERVED && region->start <= iova &&
		    (u64)region->start + region->length >= (u64)iova + bytes) {
			found = true;
			break;
		}
	}
	iommu_put_resv_regions(device, &regions);
	return found;
}

int camss_a5_iova_memory_alloc(struct device *device,
			       struct camss_a5_fw_memory *memory,
			       u32 iova, u32 bytes)
{
	struct camss_a5_fw_memory found = {};

	if (!device || !memory || !bytes || !IS_ALIGNED(iova, PAGE_SIZE) ||
	    !IS_ALIGNED(bytes, PAGE_SIZE) || (u64)iova + bytes > (1ULL << 32))
		return -EINVAL;
	if (memory->cpu || memory->mapped || memory->bytes)
		return -EBUSY;
	found.domain = iommu_get_domain_for_dev(device);
	if (!found.domain || !iommu_is_dma_domain(found.domain))
		return -ENODEV;
	/* The DMA allocator must reserve the alias before allocating any RAM;
	 * checking only that the IOVA currently looks empty is insufficient.
	 */
	if (!iova_reserved(device, iova, bytes))
		return -EINVAL;
	found.device = device;
	found.iova = iova;
	found.bytes = bytes;
	found.cpu = dma_alloc_coherent(device, found.bytes, &found.dma, GFP_KERNEL);
	if (!found.cpu)
		return -ENOMEM;
	if (found.dma > U32_MAX ||
	    (found.dma < (u64)iova + bytes && found.dma + bytes > iova) ||
	    found.bytes - 1 > U32_MAX - found.dma) {
		dma_free_coherent(device, found.bytes, found.cpu, found.dma);
		return -ERANGE;
	}
	memset(found.cpu, 0, found.bytes);
	*memory = found;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_iova_memory_alloc);

int camss_a5_fw_memory_alloc(struct device *device,
			     struct camss_a5_fw_memory *memory)
{
	int ret = camss_a5_iova_memory_alloc(device, memory, 0, CAMSS_A5_FW_WINDOW);

	/* The OEM firmware mapping is privileged. Writable unprivileged pages
	 * fault on A5 instruction fetch even though CPU IOVA walks succeed.
	 */
	if (!ret)
		memory->privileged = true;
	return ret;
}
EXPORT_SYMBOL_GPL(camss_a5_fw_memory_alloc);

int camss_a5_fw_carveout_alloc(struct device *device,
			       struct camss_a5_fw_memory *memory,
			       phys_addr_t physical, u32 bytes)
{
	struct camss_a5_fw_memory found = {};

	if (!device || !memory || !physical || !IS_ALIGNED(physical, PAGE_SIZE) ||
	    bytes != CAMSS_A5_FW_WINDOW || physical + bytes < physical)
		return -EINVAL;
	if (memory->cpu || memory->mapped || memory->bytes)
		return -EBUSY;
	found.domain = iommu_get_domain_for_dev(device);
	if (!found.domain || !iommu_is_dma_domain(found.domain))
		return -ENODEV;
	if (!iova_reserved(device, 0, bytes))
		return -EINVAL;
	found.cpu = memremap(physical, bytes, MEMREMAP_WC);
	if (!found.cpu)
		return -ENOMEM;
	found.device = device;
	found.physical = physical;
	found.bytes = bytes;
	found.privileged = true;
	found.carveout = true;
	memset(found.cpu, 0, bytes);
	*memory = found;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_fw_carveout_alloc);

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
				       (u64)memory->iova + memory->mapped - PAGE_SIZE, PAGE_SIZE);
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

	if (!memory || !memory->cpu || !memory->bytes ||
	    !IS_ALIGNED(memory->bytes, PAGE_SIZE) ||
	    !IS_ALIGNED(memory->iova, PAGE_SIZE) ||
	    (u64)memory->iova + memory->bytes > (1ULL << 32))
		return -EINVAL;
	if (memory->mapped)
		return -EBUSY;
	if (iommu_get_domain_for_dev(memory->device) != memory->domain)
		return -EXDEV;
	for (offset = 0; offset < memory->bytes; offset += PAGE_SIZE) {
		if (iommu_iova_to_phys(memory->domain, (u64)memory->iova + offset))
			return -EBUSY;
	}
	if (dev_is_dma_coherent(memory->device))
		prot |= IOMMU_CACHE;
	if (memory->privileged)
		prot |= IOMMU_PRIV;

	for (offset = 0; offset < memory->bytes; offset += PAGE_SIZE) {
		/* DMA allocations may use scattered physical pages. Translate each
		 * original DMA page instead of assuming one physical extent.
		 */
		physical = memory->carveout ? memory->physical + offset :
			iommu_iova_to_phys(memory->domain, memory->dma + offset);
		if (!physical || !IS_ALIGNED(physical, PAGE_SIZE)) {
			ret = -EFAULT;
			goto rollback;
		}
		ret = iommu_map(memory->domain, (u64)memory->iova + offset, physical,
				PAGE_SIZE, prot, GFP_KERNEL);
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
	if (memory->carveout)
		memunmap(memory->cpu);
	else
		dma_free_coherent(memory->device, memory->bytes, memory->cpu, memory->dma);
	memset(memory, 0, sizeof(*memory));
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_fw_memory_free);
