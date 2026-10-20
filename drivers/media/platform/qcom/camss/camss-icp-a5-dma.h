/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_ICP_A5_DMA_H
#define CAMSS_ICP_A5_DMA_H

#include <linux/dma-direction.h>
#include <linux/types.h>

struct device;
struct iommu_domain;
struct dma_buf;
struct dma_buf_attachment;
struct sg_table;

/* Core-owned reference and mapping. Keep through an uncertain command until
 * the CPU/IRQ and both IPE engines are quiescent. Never copy this object.
 */
struct camss_a5_dma_image {
	struct device *device;
	struct iommu_domain *domain;
	struct dma_buf *buffer;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	enum dma_data_direction direction;
	u32 offset, bytes, iova;
};

int camss_a5_dma_image_map(struct device *dev, struct camss_a5_dma_image *image,
			   struct dma_buf *buffer, u32 offset, u32 bytes,
			   enum dma_data_direction direction);
int camss_a5_dma_image_release(struct camss_a5_dma_image *image);
int camss_a5_dma_images_disjoint(const struct camss_a5_dma_image *input,
			       const struct camss_a5_dma_image *output);

#endif
