// SPDX-License-Identifier: GPL-2.0-only
/* SM8150 A5/IPE V4L2 queue adapter. Geometry/IQ/DMA stay in the SoC core.
 * Unity-scale NV12/NV21 uses core-owned DMA-BUF imports; no image CPU copy.
 */
#include <linux/dma-buf.h>
#include <linux/firmware.h>
#include <linux/fcntl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <media/v4l2-device.h>
#include <media/media-device.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-sg.h>

#include "camss-icp-a5-ipe.h"
#include "camss-icp-a5-m2m.h"

#define IPE_IMAGE_MAX (64U * 1024U * 1024U)

struct ipe_context {
	struct v4l2_fh fh;
	struct camss_a5_m2m *dev;
	struct camss_a5_session session;
	struct v4l2_pix_format_mplane format;
	struct work_struct work;
	struct list_head node;
	u32 sequence;
	bool aborting, closed;
};

struct camss_a5_m2m {
	struct camss_a5 *a5;
	struct module *owner;
	bool (*bound)(void);
	char *firmware;
	struct v4l2_device v4l2;
	struct video_device video;
	struct media_device media;
	struct v4l2_m2m_dev *scheduler;
	/* IOCTL/queue locks never nest inside hardware work. STREAMOFF may
	 * wait for that work while holding video_lock.
	 */
	struct mutex video_lock, hw_lock;
	struct list_head contexts;
	u32 users;
	bool running, fatal, power_pin, removing, mc_registered, media_registered;
	u64 jobs, completed, direct_completed, cancelled, failed, starts, stops;
};

static struct ipe_context *file_context(struct file *file)
{
	return container_of(file_to_v4l2_fh(file), struct ipe_context, fh);
}

static bool backing_owned(struct camss_a5 *a5)
{
	return camss_a5_backing_owned(a5) || a5->cpu_enabled ||
	       a5->irq_enabled || a5->power_held ||
	       a5->resources.enabled_groups || a5->resources.held_domains ||
	       a5->session_count || a5->ipe_dma_pending ||
	       a5->config_fw_mapped || a5->request_fw_mapped;
}

/* A closed file's session remains allocated if a failed reset still leaves
 * it linked in the core. Do not free a core request_owner/session reference.
 */
static void reap_closed(struct camss_a5_m2m *dev)
{
	struct ipe_context *ctx, *next;

	list_for_each_entry_safe(ctx, next, &dev->contexts, node) {
		if (!ctx->closed || ctx->session.a5 || ctx->session.handle)
			continue;
		list_del(&ctx->node);
		kfree(ctx);
	}
}

static int stop_backend(struct camss_a5_m2m *dev)
{
	int ret = camss_a5_stop(dev->a5);

	if (!ret && backing_owned(dev->a5))
		ret = -EIO;
	if (!ret)
		dev->stops++;
	if (!backing_owned(dev->a5)) {
		dev->running = false;
		if (dev->power_pin) {
			dev->power_pin = false;
			module_put(dev->owner);
		}
	}
	reap_closed(dev);
	return ret;
}

static int start_backend(struct camss_a5_m2m *dev)
{
	const struct firmware *firmware;
	int ret;

	if (dev->fatal)
		return -EIO;
	if (!dev->bound())
		return -EAGAIN;
	if (dev->running)
		return 0;
	if (backing_owned(dev->a5))
		return -EBUSY;
	if (!try_module_get(dev->owner))
		return -ENODEV;
	dev->power_pin = true;
	ret = request_firmware_direct(&firmware, dev->firmware, dev->a5->dev);
	if (ret)
		goto fail;
	ret = camss_a5_start(dev->a5, firmware);
	release_firmware(firmware);
	if (ret)
		goto fail;
	dev->running = true;
	dev->starts++;
	return 0;
fail:
	dev->fatal = true;
	return ret;
}

/* An MMAP export pins vb2 backing independently of queue/file lifetime;
 * imported buffers already carry a DMA-BUF. The core takes another reference
 * before starting DMA, including on uncertain completion/reset failure.
 */
static struct dma_buf *image_buffer(struct vb2_buffer *vb)
{
 struct dma_buf *buffer = vb->planes[0].dbuf;

 if (buffer) {
  get_dma_buf(buffer);
  return buffer;
 }
 if (vb->memory != V4L2_MEMORY_MMAP || !vb->vb2_queue->mem_ops->get_dmabuf)
  return ERR_PTR(-EINVAL);
 buffer = vb->vb2_queue->mem_ops->get_dmabuf(vb, vb->planes[0].mem_priv, O_RDWR);
 return buffer ?: ERR_PTR(-ENOMEM);
}

static int ensure_session(struct ipe_context *ctx)
{
	struct camss_a5_m2m *dev = ctx->dev;
	struct v4l2_pix_format_mplane *f = &ctx->format;
	struct camss_a5_ipe_io io = {
		.input = { f->width, f->height, f->plane_fmt[0].bytesperline,
			   f->height, f->pixelformat == V4L2_PIX_FMT_NV21 },
		.output_port = 8,
	};
	int ret;

	ret = start_backend(dev);
	if (ret)
		return ret;
	if (ctx->session.configured)
		return 0;
	io.output = io.input;
	ret = camss_a5_ipe_create(dev->a5, &ctx->session, CAMSS_A5_IPE_RT);
	if (!ret)
		ret = camss_a5_ipe_configure_nv12(dev->a5, &ctx->session, &io);
	return ret;
}

static void device_work(struct work_struct *work)
{
	struct ipe_context *ctx = container_of(work, struct ipe_context, work);
	struct camss_a5_m2m *dev = ctx->dev;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	struct dma_buf *input = NULL, *output = NULL;
	struct camss_a5_ipe_job job = {
		.input_bytes = ctx->format.plane_fmt[0].sizeimage,
		.output_bytes = ctx->format.plane_fmt[0].sizeimage,
		.bl_bytes = 512 * 1024, .max_cores = 2,
		.target_time_ns = 100000000,
	};
	int ret = -ECANCELED;

	if (!src || !dst) {
		if (src)
			v4l2_m2m_buf_done(v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx),
					 VB2_BUF_STATE_ERROR);
		if (dst)
			v4l2_m2m_buf_done(v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx),
					 VB2_BUF_STATE_ERROR);
		v4l2_m2m_job_finish(dev->scheduler, ctx->fh.m2m_ctx);
		return;
	}
	mutex_lock(&dev->hw_lock);
	dev->jobs++;
	if (READ_ONCE(ctx->aborting))
		goto finish;
	if (dev->fatal) {
		ret = -EIO;
		goto finish;
	}
	if (!dev->bound()) {
		ret = -EAGAIN;
		goto finish;
	}
	input = image_buffer(&src->vb2_buf);
	if (IS_ERR(input)) {
		ret = PTR_ERR(input);
		input = NULL;
		goto finish;
	}
	output = image_buffer(&dst->vb2_buf);
	if (IS_ERR(output)) {
		ret = PTR_ERR(output);
		output = NULL;
		goto finish;
	}
	job.input_dma = input;
	job.output_dma = output;
	job.input_offset = src->vb2_buf.planes[0].data_offset;
	job.output_offset = dst->vb2_buf.planes[0].data_offset;
	ret = ensure_session(ctx);
	if (ret)
		goto hw_fail;
	job.request_id = ctx->sequence + 1;
	ret = camss_a5_ipe_run(dev->a5, &ctx->session, &job);
	if (!ret)
		goto finish;
hw_fail:
	dev->fatal = true;
	/* On uncertain completion the core retains its own attachment and
	 * DMA-BUF reference. Queue teardown cannot free that backing. A failed
	 * quiesce keeps the platform owner and PM pin for explicit cleanup.
	 */
	stop_backend(dev);
finish:
	if (output)
		dma_buf_put(output);
	if (input)
		dma_buf_put(input);
	if (!ret && READ_ONCE(ctx->aborting))
		ret = -ECANCELED;
	if (ret == -ECANCELED)
		dev->cancelled++;
	else if (ret)
		dev->failed++;
	else {
		dev->completed++;
		dev->direct_completed++;
	}
	mutex_unlock(&dev->hw_lock);
	if (ret && ret != -ECANCELED) {
		vb2_queue_error(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx));
		vb2_queue_error(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx));
	}
	v4l2_m2m_buf_copy_metadata(src, dst);
	src->sequence = dst->sequence = ctx->sequence++;
	v4l2_m2m_buf_done_and_job_finish(dev->scheduler, ctx->fh.m2m_ctx,
				       ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
}

static void device_run(void *private)
{
	struct ipe_context *ctx = private;

	schedule_work(&ctx->work);
}

static void job_abort(void *private)
{
	struct ipe_context *ctx = private;

	/* Do not cancel pending work: the scheduler waits for its job_finish.
	 * The bounded synchronous core must complete/quiesce before reuse.
	 */
	WRITE_ONCE(ctx->aborting, true);
}

static int queue_setup(struct vb2_queue *q, unsigned int *buffers,
		       unsigned int *planes, unsigned int sizes[],
		       struct device *alloc_devs[])
{
	struct ipe_context *ctx = vb2_get_drv_priv(q);
	u32 size = ctx->format.plane_fmt[0].sizeimage;

	if (*planes)
		return *planes == 1 && sizes[0] >= size ? 0 : -EINVAL;
	*planes = 1;
	sizes[0] = size;
	return 0;
}

static int buffer_prepare(struct vb2_buffer *vb)
{
	struct ipe_context *ctx = vb2_get_drv_priv(vb->vb2_queue);
	u32 bytes = ctx->format.plane_fmt[0].sizeimage;
	u32 offset = vb->planes[0].data_offset;
	unsigned long size = vb2_plane_size(vb, 0);

	if (offset > size || bytes > size - offset ||
	    (u64)offset + bytes > U32_MAX)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(vb->type)) {
		if (vb2_get_plane_payload(vb, 0) < offset + bytes)
			return -EINVAL;
	} else {
		vb2_set_plane_payload(vb, 0, offset + bytes);
	}
	return 0;
}

static void buffer_queue(struct vb2_buffer *vb)
{
	struct ipe_context *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct ipe_context *ctx = vb2_get_drv_priv(q);

	WRITE_ONCE(ctx->aborting, false);
	ctx->sequence = 0;
	return 0;
}

static void stop_streaming(struct vb2_queue *q)
{
	struct ipe_context *ctx = vb2_get_drv_priv(q);
	struct camss_a5_m2m *dev = ctx->dev;
	struct vb2_v4l2_buffer *buffer;
	int ret = 0;

	/* m2m has already waited for TRANS_RUNNING; join the work function
	 * itself before releasing its context/session.
	 */
	flush_work(&ctx->work);
	while ((buffer = V4L2_TYPE_IS_OUTPUT(q->type) ?
		v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
		v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(buffer, VB2_BUF_STATE_ERROR);
	mutex_lock(&dev->hw_lock);
	if (ctx->session.handle)
		ret = camss_a5_ipe_destroy(dev->a5, &ctx->session);
	if (ret) {
		dev->fatal = true;
		stop_backend(dev);
	} else if (!dev->a5->session_count && dev->power_pin) {
		ret = stop_backend(dev);
		if (ret)
			dev->fatal = true;
	}
	mutex_unlock(&dev->hw_lock);
}

static const struct vb2_ops queue_ops = {
	.queue_setup = queue_setup,
	.buf_prepare = buffer_prepare,
	.buf_queue = buffer_queue,
	.start_streaming = start_streaming,
	.stop_streaming = stop_streaming,
};

static int queue_init(void *private, struct vb2_queue *src,
		      struct vb2_queue *dst)
{
	struct ipe_context *ctx = private;
	struct vb2_queue *queues[] = { src, dst };
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(queues); i++) {
		struct vb2_queue *q = queues[i];

		q->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE :
			      V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		q->io_modes = VB2_MMAP | VB2_DMABUF;
		q->drv_priv = ctx;
		q->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
		q->ops = &queue_ops;
		q->mem_ops = &vb2_dma_sg_memops;
		q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		q->lock = &ctx->dev->video_lock;
		q->dev = ctx->dev->a5->dev;
		ret = vb2_queue_init(q);
		if (ret)
			return ret;
	}
	return 0;
}

static int querycap(struct file *file, void *private, struct v4l2_capability *cap)
{
	struct camss_a5_m2m *dev = video_drvdata(file);

	strscpy(cap->driver, "qcom-ipe-a5", sizeof(cap->driver));
	strscpy(cap->card, "SM8150 IPE", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(dev->a5->dev));
	return 0;
}

static int enum_format(struct file *file, void *private, struct v4l2_fmtdesc *f)
{
	if (f->index > 1)
		return -EINVAL;
	f->pixelformat = f->index ? V4L2_PIX_FMT_NV21 : V4L2_PIX_FMT_NV12;
	return 0;
}

static int try_format(struct file *file, void *private, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *p = &f->fmt.pix_mp;
	u64 stride, max_stride;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	    f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;
	if (p->pixelformat != V4L2_PIX_FMT_NV12 && p->pixelformat != V4L2_PIX_FMT_NV21)
		p->pixelformat = V4L2_PIX_FMT_NV12;
	p->width = clamp_val(p->width, 128, 8192) & ~1U;
	p->height = clamp_val(p->height, 128,
			      min(8192U, ((IPE_IMAGE_MAX / 3 * 2) / p->width) & ~1U)) & ~1U;
	max_stride = ((IPE_IMAGE_MAX / 3 * 2) / p->height) & ~1U;
	stride = min_t(u64, ALIGN(max_t(u64, p->width, p->plane_fmt[0].bytesperline), 2),
		       max_stride);
	p->field = V4L2_FIELD_NONE;
	p->num_planes = 1;
	memset(p->plane_fmt, 0, sizeof(p->plane_fmt));
	p->plane_fmt[0].bytesperline = stride;
	p->plane_fmt[0].sizeimage = stride * p->height * 3 / 2;
	if (!p->colorspace)
		p->colorspace = V4L2_COLORSPACE_REC709;
	memset(p->reserved, 0, sizeof(p->reserved));
	return 0;
}

static int get_format(struct file *file, void *private, struct v4l2_format *f)
{
	f->fmt.pix_mp = file_context(file)->format;
	return 0;
}

static int set_format(struct file *file, void *private, struct v4l2_format *f)
{
	struct ipe_context *ctx = file_context(file);
	int ret;

	if (vb2_is_busy(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx)) ||
	    vb2_is_busy(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
		return -EBUSY;
	ret = try_format(file, private, f);
	if (!ret)
		ctx->format = f->fmt.pix_mp; /* bypass has a single shared geometry */
	return ret;
}

static const struct v4l2_ioctl_ops ioctl_ops = {
	.vidioc_querycap = querycap,
	.vidioc_enum_fmt_vid_cap = enum_format,
	.vidioc_enum_fmt_vid_out = enum_format,
	.vidioc_try_fmt_vid_cap_mplane = try_format,
	.vidioc_try_fmt_vid_out_mplane = try_format,
	.vidioc_g_fmt_vid_cap_mplane = get_format,
	.vidioc_g_fmt_vid_out_mplane = get_format,
	.vidioc_s_fmt_vid_cap_mplane = set_format,
	.vidioc_s_fmt_vid_out_mplane = set_format,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

static int open_context(struct file *file)
{
	struct camss_a5_m2m *dev = video_drvdata(file);
	struct v4l2_format format = {
		.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		.fmt.pix_mp = { .width = 640, .height = 480,
				.pixelformat = V4L2_PIX_FMT_NV12 },
	};
	struct ipe_context *ctx;
	int ret;

	if (mutex_lock_interruptible(&dev->video_lock))
		return -ERESTARTSYS;
	if (dev->removing || !try_module_get(dev->owner)) {
		ret = -ENODEV;
		goto unlock;
	}
	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		ret = -ENOMEM;
		goto put_owner;
	}
	ctx->dev = dev;
	INIT_WORK(&ctx->work, device_work);
	v4l2_fh_init(&ctx->fh, &dev->video);
	try_format(file, NULL, &format);
	ctx->format = format.fmt.pix_mp;
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(dev->scheduler, ctx, queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		goto put_owner;
	}
	mutex_lock(&dev->hw_lock);
	list_add_tail(&ctx->node, &dev->contexts);
	mutex_unlock(&dev->hw_lock);
	dev->users++;
	v4l2_fh_add(&ctx->fh, file);
	mutex_unlock(&dev->video_lock);
	return 0;
put_owner:
	module_put(dev->owner);
unlock:
	mutex_unlock(&dev->video_lock);
	return ret;
}

static int release_context(struct file *file)
{
	struct ipe_context *ctx = file_context(file);
	struct camss_a5_m2m *dev = ctx->dev;
	struct module *owner = dev->owner;
	int ret = 0;

	mutex_lock(&dev->video_lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	flush_work(&ctx->work);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	mutex_lock(&dev->hw_lock);
	dev->users--;
	ctx->closed = true;
	if (!dev->users && dev->power_pin)
		ret = stop_backend(dev);
	if (!dev->users && !ret && !backing_owned(dev->a5))
		dev->fatal = false;
	reap_closed(dev);
	mutex_unlock(&dev->hw_lock);
	mutex_unlock(&dev->video_lock);
	module_put(owner);
	return 0;
}

static const struct v4l2_file_operations file_ops = {
	.owner = THIS_MODULE,
	.open = open_context,
	.release = release_context,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct v4l2_m2m_ops scheduler_ops = {
	.device_run = device_run,
	.job_abort = job_abort,
};

static void release_device(struct video_device *video)
{
	struct camss_a5_m2m *dev = container_of(video, struct camss_a5_m2m, video);

	v4l2_device_unregister(&dev->v4l2);
	v4l2_m2m_release(dev->scheduler);
	media_device_cleanup(&dev->media);
	kfree(dev->firmware);
	kfree(dev);
}

struct camss_a5_m2m *camss_a5_m2m_register(struct camss_a5 *a5,
					struct module *owner, const char *firmware,
					bool (*bound)(void))
{
	struct camss_a5_m2m *dev;
	int ret;

	if (!a5 || !owner || !firmware || !bound || !a5->engine_dma_capable ||
	    backing_owned(a5))
		return ERR_PTR(-EINVAL);
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return ERR_PTR(-ENOMEM);
	dev->a5 = a5;
	dev->owner = owner;
	dev->bound = bound;
	dev->removing = true;
	dev->media.dev = a5->dev;
	strscpy(dev->media.driver_name, "qcom-ipe-a5", sizeof(dev->media.driver_name));
	strscpy(dev->media.model, "SM8150 IPE", sizeof(dev->media.model));
	snprintf(dev->media.bus_info, sizeof(dev->media.bus_info), "platform:%s",
		 dev_name(a5->dev));
	media_device_init(&dev->media);
	dev->firmware = kstrdup(firmware, GFP_KERNEL);
	if (!dev->firmware) {
		ret = -ENOMEM;
		goto free;
	}
	mutex_init(&dev->video_lock);
	mutex_init(&dev->hw_lock);
	INIT_LIST_HEAD(&dev->contexts);
	ret = v4l2_device_register(a5->dev, &dev->v4l2);
	if (ret)
		goto free;
	dev->v4l2.mdev = &dev->media;
	dev->scheduler = v4l2_m2m_init(&scheduler_ops);
	if (IS_ERR(dev->scheduler)) {
		ret = PTR_ERR(dev->scheduler);
		goto unregister;
	}
	strscpy(dev->video.name, "qcom-ipe-a5", sizeof(dev->video.name));
	dev->video.v4l2_dev = &dev->v4l2;
	dev->video.fops = &file_ops;
	dev->video.ioctl_ops = &ioctl_ops;
	dev->video.lock = &dev->video_lock;
	dev->video.vfl_dir = VFL_DIR_M2M;
	dev->video.release = release_device;
	dev->video.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	video_set_drvdata(&dev->video, dev);
	ret = video_register_device(&dev->video, VFL_TYPE_VIDEO, -1);
	if (ret) {
		v4l2_m2m_release(dev->scheduler);
		goto unregister;
	}
	ret = v4l2_m2m_register_media_controller(dev->scheduler, &dev->video,
						MEDIA_ENT_F_PROC_VIDEO_ISP);
	if (ret)
		goto unregister_video;
	dev->mc_registered = true;
	ret = media_device_register(&dev->media);
	if (ret) {
		v4l2_m2m_unregister_media_controller(dev->scheduler);
		dev->mc_registered = false;
		goto unregister_video;
	}
	dev->media_registered = true;
	mutex_lock(&dev->video_lock);
	dev->removing = false;
	mutex_unlock(&dev->video_lock);
	dev_info(a5->dev, "IPE m2m registered as %s; firmware starts on a queued job\n",
		 video_device_node_name(&dev->video));
	return dev;
unregister_video:
	video_unregister_device(&dev->video);
	return ERR_PTR(ret);
unregister:
	v4l2_device_unregister(&dev->v4l2);
free:
	media_device_cleanup(&dev->media);
	kfree(dev->firmware);
	kfree(dev);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(camss_a5_m2m_register);

int camss_a5_m2m_cleanup(struct camss_a5_m2m *dev)
{
	int ret;

	if (!dev)
		return 0;
	mutex_lock(&dev->video_lock);
	if (dev->users) {
		ret = -EBUSY;
		goto unlock;
	}
	mutex_lock(&dev->hw_lock);
	ret = stop_backend(dev);
	if (!ret)
		dev->fatal = false;
	mutex_unlock(&dev->hw_lock);
unlock:
	mutex_unlock(&dev->video_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(camss_a5_m2m_cleanup);

int camss_a5_m2m_unregister(struct camss_a5_m2m *dev)
{
	int ret;

	if (!dev)
		return 0;
	ret = camss_a5_m2m_cleanup(dev);
	if (ret)
		return ret;
	mutex_lock(&dev->video_lock);
	if (dev->users || dev->power_pin || !list_empty(&dev->contexts)) {
		mutex_unlock(&dev->video_lock);
		return -EBUSY;
	}
	dev->removing = true;
	mutex_unlock(&dev->video_lock);
	if (dev->media_registered)
		media_device_unregister(&dev->media);
	if (dev->mc_registered)
		v4l2_m2m_unregister_media_controller(dev->scheduler);
	video_unregister_device(&dev->video);
	return 0; /* release_device may already have freed dev */
}
EXPORT_SYMBOL_GPL(camss_a5_m2m_unregister);

ssize_t camss_a5_m2m_status(struct camss_a5_m2m *dev, char *buffer, size_t size)
{
	struct ipe_context *ctx;
	u32 orphans = 0;
	ssize_t ret;

	if (!dev)
		return scnprintf(buffer, size, "{\"registered\":false}\n");
	mutex_lock(&dev->video_lock);
	mutex_lock(&dev->hw_lock);
	list_for_each_entry(ctx, &dev->contexts, node)
		orphans += ctx->closed;
	ret = scnprintf(buffer, size,
		"{\"registered\":true,\"users\":%u,\"running\":%s,\"fatal\":%s,"
		"\"pinned\":%s,\"orphans\":%u,\"jobs\":%llu,\"completed\":%llu,"
		"\"direct_completed\":%llu,\"cancelled\":%llu,\"failed\":%llu,\"starts\":%llu,\"stops\":%llu}\n",
		dev->users, dev->running ? "true" : "false", dev->fatal ? "true" : "false",
		dev->power_pin ? "true" : "false", orphans,
		(unsigned long long)dev->jobs, (unsigned long long)dev->completed,
		(unsigned long long)dev->direct_completed,
		(unsigned long long)dev->cancelled, (unsigned long long)dev->failed,
		(unsigned long long)dev->starts,
		(unsigned long long)dev->stops);
	mutex_unlock(&dev->hw_lock);
	mutex_unlock(&dev->video_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(camss_a5_m2m_status);

MODULE_DESCRIPTION("SM8150 A5 IPE V4L2 mem2mem adapter");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("DMA_BUF");
