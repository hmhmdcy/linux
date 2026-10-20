// SPDX-License-Identifier: GPL-2.0-only
/* Linear IPE images imported into the same DMA domain as A5 and its engines. */
#include <linux/dma-buf.h>
#include <linux/dma-resv.h>
#include <linux/err.h>
#include <linux/iommu.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/scatterlist.h>
#include <linux/sort.h>
#include <linux/string.h>
#include <linux/vmalloc.h>

#include "camss-icp-a5-dma.h"

/* SG entries may be physically scattered while their DMA addresses form one
 * IOVA span, as in CAMSS video_check_dma_mapping(). Clip to the image range;
 * no identity mapping or one-physical-extent assumption is made.
 */
static int image_iova(const struct sg_table *sgt, u32 offset, u32 bytes, u32 *iova)
{
	struct scatterlist *sg;
	u64 skip = offset, remaining = bytes, expected = 0, first = 0;
	unsigned int i;

	if (!sgt || !sgt->sgl || !bytes || !iova)
		return -EINVAL;
	for_each_sgtable_dma_sg(sgt, sg, i) {
		u64 length = sg_dma_len(sg), address = sg_dma_address(sg), take;

		if (!length)
			return -EINVAL;
		if (skip >= length) {
			skip -= length;
			continue;
		}
		if (address > U32_MAX || skip > U32_MAX - address)
			return -ERANGE;
		address += skip;
		take = min_t(u64, remaining, length - skip);
		if (!address || take - 1 > U32_MAX - address ||
		    (expected && address != expected))
			return -ERANGE;
		if (!first)
			first = address;
		expected = address + take;
		remaining -= take;
		if (!remaining) {
			if (first & 3)
				return -EINVAL;
			*iova = first;
			return 0;
		}
		skip = 0;
	}
	return -EINVAL;
}

int camss_a5_dma_image_release(struct camss_a5_dma_image *image)
{
	if (!image)
		return -EINVAL;
	if (!image->buffer)
		return 0;
	/* The platform owner must not switch domains while an import is live. */
	if (iommu_get_domain_for_dev(image->device) != image->domain)
		return -EXDEV;
	if (image->sgt)
		dma_buf_unmap_attachment_unlocked(image->attachment, image->sgt,
						image->direction);
	if (image->attachment)
		dma_buf_detach(image->buffer, image->attachment);
	dma_buf_put(image->buffer);
	memset(image, 0, sizeof(*image));
	return 0;
}

int camss_a5_dma_image_map(struct device *dev, struct camss_a5_dma_image *image,
			   struct dma_buf *buffer, u32 offset, u32 bytes,
			   enum dma_data_direction direction)
{
	struct iommu_domain *domain;
	enum dma_resv_usage usage;
	long waited;
	int ret, cleanup;

	if (!dev || !image || !buffer || !bytes || (offset & 3) ||
	    offset > buffer->size || bytes > buffer->size - offset ||
	    (direction != DMA_TO_DEVICE && direction != DMA_FROM_DEVICE))
		return -EINVAL;
	if (image->buffer || image->attachment || image->sgt)
		return -EBUSY;
	domain = iommu_get_domain_for_dev(dev);
	if (!domain || !iommu_is_dma_domain(domain))
		return -ENODEV;
	/* V4L2 QBUF/DQBUF remains the explicit ownership protocol. Respect
	 * pre-existing producer/read fences as well; this synchronous interface
	 * does not advertise asynchronous implicit-fence interoperability.
	 */
	usage = direction == DMA_TO_DEVICE ? DMA_RESV_USAGE_WRITE : DMA_RESV_USAGE_READ;
	waited = dma_resv_wait_timeout(buffer->resv, usage, true, msecs_to_jiffies(1000));
	if (waited <= 0)
		return waited ? (int)waited : -ETIMEDOUT;
	get_dma_buf(buffer);
	image->device = dev;
	image->domain = domain;
	image->buffer = buffer;
	image->direction = direction;
	image->offset = offset;
	image->bytes = bytes;
	image->attachment = dma_buf_attach(buffer, dev);
	if (IS_ERR(image->attachment)) {
		ret = PTR_ERR(image->attachment);
		image->attachment = NULL;
		goto fail;
	}
	image->sgt = dma_buf_map_attachment_unlocked(image->attachment, direction);
	if (IS_ERR(image->sgt)) {
		ret = PTR_ERR(image->sgt);
		image->sgt = NULL;
		goto fail;
	}
	ret = image_iova(image->sgt, offset, bytes, &image->iova);
	if (!ret)
		return 0;
fail:
	cleanup = camss_a5_dma_image_release(image);
	return cleanup ? cleanup : ret;
}

struct image_range {
	phys_addr_t first, last;
};

static int physical_ranges(const struct camss_a5_dma_image *image,
			   struct image_range *ranges, unsigned int *count)
{
	u64 address = image->iova, remaining = image->bytes;
	unsigned int used = 0;

	/* DMA-BUF importers must use DMA addresses only. DMABUF_DEBUG returns
	 * a table with page/physical-length fields removed. Resolve aliases
	 * through our own, pinned DMA domain instead of exporter SG internals.
	 */
	while (remaining) {
		u64 offset = address & (PAGE_SIZE - 1);
		u64 take = min_t(u64, remaining, PAGE_SIZE - offset);
		phys_addr_t physical = iommu_iova_to_phys(image->domain, address - offset);
		phys_addr_t first, last;

		if (!physical || !IS_ALIGNED(physical, PAGE_SIZE))
			return -EFAULT;
		if (physical + offset < physical || physical + offset + take - 1 < physical + offset)
			return -ERANGE;
		first = physical + offset;
		last = first + take - 1;
		if (used && ranges[used - 1].last != (phys_addr_t)~0ULL &&
		    ranges[used - 1].last + 1 == first)
			ranges[used - 1].last = last;
		else
			ranges[used++] = (struct image_range){ first, last };
		remaining -= take;
		address += take;
	}
	*count = used;
	return 0;
}

static int range_compare(const void *left, const void *right)
{
	const struct image_range *a = left, *b = right;

	return a->first < b->first ? -1 : a->first > b->first;
}

/* Different attachments/exports can alias the same RAM even with distinct
 * IOVAs. Reject physical overlap without a quadratic SG cross-product.
 */
int camss_a5_dma_images_disjoint(const struct camss_a5_dma_image *input,
			       const struct camss_a5_dma_image *output)
{
	struct image_range *in, *out;
	unsigned int ni, no, i = 0, j = 0;
	int ret;

	if (!input || !output || !input->buffer || !output->buffer ||
	    !input->sgt || !output->sgt || !input->iova || !output->iova ||
	    !input->bytes || !output->bytes || input->buffer == output->buffer)
		return -EINVAL;
	if (input->domain != output->domain ||
	    iommu_get_domain_for_dev(input->device) != input->domain ||
	    iommu_get_domain_for_dev(output->device) != output->domain)
		return -EXDEV;
	ni = DIV_ROUND_UP((u64)(input->iova & (PAGE_SIZE - 1)) + input->bytes, PAGE_SIZE);
	no = DIV_ROUND_UP((u64)(output->iova & (PAGE_SIZE - 1)) + output->bytes, PAGE_SIZE);
	in = kvmalloc_array(ni, sizeof(*in), GFP_KERNEL);
	out = kvmalloc_array(no, sizeof(*out), GFP_KERNEL);
	if (!in || !out) {
		ret = -ENOMEM;
		goto done;
	}
	ret = physical_ranges(input, in, &ni);
	if (!ret)
		ret = physical_ranges(output, out, &no);
	if (ret)
		goto done;
	sort(in, ni, sizeof(*in), range_compare, NULL);
	sort(out, no, sizeof(*out), range_compare, NULL);
	while (i < ni && j < no) {
		if (in[i].first <= out[j].last && out[j].first <= in[i].last) {
			ret = -EINVAL;
			goto done;
		}
		if (in[i].last < out[j].last)
			i++;
		else
			j++;
	}
done:
	kvfree(out);
	kvfree(in);
	return ret;
}
