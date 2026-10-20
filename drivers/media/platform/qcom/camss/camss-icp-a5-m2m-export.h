/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_ICP_A5_M2M_EXPORT_H
#define CAMSS_ICP_A5_M2M_EXPORT_H

/* Included after the private m2m/context definitions. Delegate storage to
 * vb2 DMA-SG while exported buffers keep this DMA-domain owner registered.
 */
struct ipe_owned_export {
	struct camss_a5_m2m *dev;
	struct module *owner;
	struct dma_buf *base;
};

static int owned_attach(struct dma_buf *buffer, struct dma_buf_attachment *attach)
{
	struct ipe_owned_export *owned = buffer->priv;
	struct dma_buf_attachment *base = dma_buf_attach(owned->base, attach->dev);

	if (IS_ERR(base))
		return PTR_ERR(base);
	attach->priv = base;
	return 0;
}

static void owned_detach(struct dma_buf *buffer, struct dma_buf_attachment *attach)
{
	struct ipe_owned_export *owned = buffer->priv;

	dma_buf_detach(owned->base, attach->priv);
}

static struct sg_table *owned_map(struct dma_buf_attachment *attach,
				enum dma_data_direction direction)
{
	/* Both exports use the same reservation lock; use the locked API. */
	return dma_buf_map_attachment(attach->priv, direction);
}

static void owned_unmap(struct dma_buf_attachment *attach, struct sg_table *sgt,
			enum dma_data_direction direction)
{
	dma_buf_unmap_attachment(attach->priv, sgt, direction);
}

static int owned_begin_cpu(struct dma_buf *buffer, enum dma_data_direction direction)
{
	struct ipe_owned_export *owned = buffer->priv;

	return dma_buf_begin_cpu_access(owned->base, direction);
}

static int owned_end_cpu(struct dma_buf *buffer, enum dma_data_direction direction)
{
	struct ipe_owned_export *owned = buffer->priv;

	return dma_buf_end_cpu_access(owned->base, direction);
}

static int owned_vmap(struct dma_buf *buffer, struct iosys_map *map)
{
	struct ipe_owned_export *owned = buffer->priv;

	return dma_buf_vmap(owned->base, map);
}

static void owned_vunmap(struct dma_buf *buffer, struct iosys_map *map)
{
	struct ipe_owned_export *owned = buffer->priv;

	dma_buf_vunmap(owned->base, map);
}

static int owned_mmap(struct dma_buf *buffer, struct vm_area_struct *vma)
{
	struct ipe_owned_export *owned = buffer->priv;

	/* The base is always our vb2 DMA-SG allocation. Its mmap callback keeps
	 * vm_file unchanged; dma_buf_mmap(base, ...) would replace the wrapper
	 * file and drop the platform pin when only the VMA remains alive.
	 */
	return owned->base->ops->mmap(owned->base, vma);
}

static void owned_free(struct ipe_owned_export *owned)
{
	struct module *owner = owned->owner;
	struct camss_a5_m2m *dev = owned->dev;

	/* Retire the allocator's DMA mapping before dropping the owner pin. */
	dma_buf_put(owned->base);
	atomic_dec(&dev->exported_buffers);
	put_device(&dev->video.dev);
	kfree(owned);
	module_put(owner);
}

static void owned_release(struct dma_buf *buffer)
{
	owned_free(buffer->priv);
}

static const struct dma_buf_ops owned_export_ops = {
	.attach = owned_attach,
	.detach = owned_detach,
	.map_dma_buf = owned_map,
	.unmap_dma_buf = owned_unmap,
	.begin_cpu_access = owned_begin_cpu,
	.end_cpu_access = owned_end_cpu,
	.vmap = owned_vmap,
	.vunmap = owned_vunmap,
	.mmap = owned_mmap,
	.release = owned_release,
};

static struct dma_buf *owned_export_buffer(struct vb2_buffer *vb, void *private,
					  unsigned long flags)
{
	struct ipe_context *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_a5_m2m *dev = ctx->dev;
	struct ipe_owned_export *owned;
	struct dma_buf *buffer;
	DEFINE_DMA_BUF_EXPORT_INFO(info);

	if (READ_ONCE(dev->removing) || !try_module_get(dev->owner))
		return NULL;
	owned = kzalloc(sizeof(*owned), GFP_KERNEL);
	if (!owned)
		goto owner;
	owned->base = vb2_dma_sg_memops.get_dmabuf(vb, private, flags);
	if (!owned->base)
		goto storage;
	owned->dev = dev;
	owned->owner = dev->owner;
	get_device(&dev->video.dev);
	atomic_inc(&dev->exported_buffers);
	info.ops = &owned_export_ops;
	info.size = owned->base->size;
	info.flags = flags;
	info.priv = owned;
	info.resv = owned->base->resv;
	buffer = dma_buf_export(&info);
	if (!IS_ERR(buffer))
		return buffer;
	owned_free(owned);
	return NULL;
storage:
	kfree(owned);
owner:
	module_put(dev->owner);
	return NULL;
}

#endif
