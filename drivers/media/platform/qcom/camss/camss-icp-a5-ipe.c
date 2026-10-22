// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/limits.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "camss-icp-a5-ipe.h"

int camss_a5_ipe_striping_bytes(const void *data, u32 bytes, u32 *used)
{
	const u8 *source = data;
	u64 needed = 2268;
	u32 passes, i, stripes;

	if (!source || !used || bytes < 2268)
		return -EINVAL;
	passes = get_unaligned_le32(source + 20);
	if (!passes || passes > 4)
		return -EINVAL;
	for (i = 0; i < passes; i++) {
		stripes = get_unaligned_le16(source + 428 + i * 412);
		if (!stripes)
			return -EINVAL;
		needed += (u64)stripes * 624;
	}
	if (needed > bytes)
		return -EINVAL;
	*used = needed;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_striping_bytes);

static bool valid_buffer(const struct camss_a5_ipe_buffer *buffer)
{
	return buffer->iova && !(buffer->iova & 3) && buffer->bytes &&
	       (u64)buffer->iova + buffer->bytes <= (1ULL << 32);
}

static bool buffers_overlap(const struct camss_a5_ipe_buffer *a,
			    const struct camss_a5_ipe_buffer *b)
{
	return a->iova < (u64)b->iova + b->bytes &&
	       b->iova < (u64)a->iova + a->bytes;
}

static int nv12_plane_bytes(const struct camss_a5_nv12_layout *layout,
			    const struct camss_a5_ipe_buffer *buffer, u32 *y)
{
	u64 plane;

	if (!layout->width || !layout->height ||
	    ((layout->width | layout->height | layout->stride |
	      layout->slice_height) & 1) ||
	    layout->stride < layout->width || layout->slice_height < layout->height)
		return -EINVAL;
	plane = (u64)layout->stride * layout->slice_height;
	if (plane > buffer->bytes || plane / 2 > buffer->bytes - plane)
		return -EINVAL;
	*y = plane;
	return 0;
}

int camss_a5_ipe_encode_frame(void *destination, u32 bytes,
			    const struct camss_a5_ipe_io *io,
			    const struct camss_a5_ipe_frame *frame)
{
	const struct camss_a5_ipe_buffer *buffers[7];
	struct camss_a5_pd10_layout expected;
	u8 *output = destination;
	u32 input_y, output_y, used, i, j, count = 6;
	int ret;

	if (!output || bytes < CAMSS_A5_IPE_FRAME_BYTES || !io || !frame ||
	    (io->output_port != 8 && io->output_port != 9) ||
	    !frame->max_cores || frame->max_cores > 2 || !frame->target_time_ns ||
	    !frame->required_scratch_bytes || !frame->required_bl_bytes ||
	    frame->scratch.bytes < frame->required_scratch_bytes ||
	    frame->bl.bytes < frame->required_bl_bytes || frame->bl.bytes < 32 ||
	    frame->iq.bytes < CAMSS_A5_IPE_IQ_BYTES)
		return -EINVAL;
	buffers[0] = &frame->striping;
	buffers[1] = &frame->iq;
	buffers[2] = &frame->scratch;
	buffers[3] = &frame->bl;
	buffers[4] = &frame->input;
	buffers[5] = &frame->output;
	if (frame->coarse.bytes || frame->coarse.iova) {
		ret = camss_a5_ipe_pd10_quarter(&io->input, &expected);
		if (ret || frame->coarse.bytes != expected.bytes)
			return -EINVAL;
		buffers[count++] = &frame->coarse;
	}
	for (i = 0; i < count; i++) {
		if (!valid_buffer(buffers[i]))
			return -EINVAL;
		for (j = 0; j < i; j++)
			if (buffers_overlap(buffers[i], buffers[j]))
				return -EINVAL;
	}
	ret = camss_a5_ipe_striping_bytes(frame->striping_cpu,
					frame->striping.bytes, &used);
	if (ret)
		return ret;
	ret = nv12_plane_bytes(&io->input, &frame->input, &input_y);
	if (ret)
		return ret;
	ret = nv12_plane_bytes(&io->output, &frame->output, &output_y);
	if (ret)
		return ret;
	/* All checks precede the first output write, including arithmetic and
	 * pairwise range checks. Empty command offsets are relative to base 0.
	 */
	memset(output, 0, CAMSS_A5_IPE_FRAME_BYTES);
	put_unaligned_le32(frame->striping.iova, output);
	put_unaligned_le32(frame->iq.iova, output + 4);
	put_unaligned_le32(frame->scratch.iova, output + 8);
	put_unaligned_le32(frame->scratch.bytes, output + 12);
	put_unaligned_le32(frame->bl.iova, output + 24);
	put_unaligned_le32(frame->bl.bytes, output + 28);
	put_unaligned_le32(frame->max_cores, output + 32);
	put_unaligned_le32(frame->target_time_ns, output + 36);
	put_unaligned_le32(frame->request_id, output + 84);
	put_unaligned_le32(1, output + 88);
	put_unaligned_le32(frame->input.iova, output + 92);
	put_unaligned_le32(frame->input.iova + input_y, output + 96);
	if (frame->coarse.bytes)
		put_unaligned_le32(frame->coarse.iova, output + 108);
	put_unaligned_le32(frame->output.iova, output + 92 + io->output_port * 16);
	put_unaligned_le32(frame->output.iova + output_y,
			   output + 96 + io->output_port * 16);
	put_unaligned_le64(frame->user_data, output + 3808);
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_encode_frame);

// SPDX-License-Identifier: GPL-2.0-only
/* SM8150 8-bit linear YUV: keep IQ modules disabled, while retaining the
 * mandatory ICA alignment and invalid-pixel interpolation defaults used by
 * the target HAL. IcaParameters occupies 192 bytes; each field is a u32.
 */
int camss_a5_ipe_encode_bypass_iq(void *buffer, u32 bytes)
{
	if (!buffer || bytes < CAMSS_A5_IPE_IQ_BYTES)
		return -EINVAL;
	memset(buffer, 0, CAMSS_A5_IPE_IQ_BYTES);
	put_unaligned_le32(1, buffer + 52);
	put_unaligned_le32(1, buffer + 68);
	put_unaligned_le32(1, buffer + 244);
	put_unaligned_le32(1, buffer + 260);
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_encode_bypass_iq);

// SPDX-License-Identifier: GPL-2.0-only
/* Full-frame unity-scale 8-bit YUV. Common range checks preserve output
 * and count on rejection; the two target policies differ in step and halo.
 * This is a partition primitive, not a complete LTM striping serializer.
 */
static int ipe_partitions(u32 width, u32 cores, u32 step, u32 halo,
				 struct camss_a5_ipe_partition *output,
				 u32 capacity, u32 *count)
{
	u32 stripes, i;

	if (!output || !count || width < 128 || width > 8192 || (width & 1) ||
	    !cores || cores > 2)
		return -EINVAL;
	stripes = (width + step - 1) / step;
	stripes = ((stripes + cores - 1) / cores) * cores;
	if (capacity < stripes)
		return -ENOSPC;
	for (i = 0; i < stripes; i++) {
		u32 index = stripes - i - 1;
		u32 first = ((width * index) / stripes) & ~1U;
		u32 last = (((width * (index + 1)) / stripes) & ~1U) - 1;

		output[i].output_first = first;
		output[i].output_last = last;
		output[i].fetch_first = first < halo ? 0 : first - halo;
		output[i].fetch_last = last + halo < width ? last + halo : width - 1;
	}
	*count = stripes;
	return 0;
}

int camss_a5_ipe_bypass_partitions(u32 width, u32 cores,
				 struct camss_a5_ipe_partition *output,
				 u32 capacity, u32 *count)
{
	return ipe_partitions(width, cores, 328, 4, output, capacity, count);
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_bypass_partitions);

int camss_a5_ipe_ltm_partitions(u32 width, u32 cores,
			      struct camss_a5_ipe_partition *output,
			      u32 capacity, u32 *count)
{
	return ipe_partitions(width, cores, 324, 6, output, capacity, count);
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_ltm_partitions);

// SPDX-License-Identifier: GPL-2.0-only
/* SM8150, one full-frame unity-scale pass, linear 8-bit NV12/NV21.
 * Geometry is serialized into the A5 firmware ABI; no OEM blob is copied.
 * IQ modules stay disabled. Mandatory crops, FE/WE geometry, LTM grid
 * coordinates and radial coordinates are still consumed by CDM preparation.
 */
int camss_a5_ipe_bypass_size(const struct camss_a5_ipe_io *io, u32 cores,
			   u32 *bytes)
{
	const struct camss_a5_nv12_layout *in, *out;
	u32 stripes;

	if (!io || !bytes || !cores || cores > 2 ||
	    (io->output_port != 8 && io->output_port != 9))
		return -EINVAL;
	in = &io->input;
	out = &io->output;
	if (in->width < 128 || in->width > 8192 ||
	    in->height < 128 || in->height > 8192 ||
	    in->width != out->width || in->height != out->height ||
	    in->uv_swap != out->uv_swap ||
	    ((in->width | in->height | in->stride | in->slice_height |
	      out->stride | out->slice_height) & 1) ||
	    in->stride < in->width || in->slice_height < in->height ||
	    out->stride < out->width || out->slice_height < out->height)
		return -EINVAL;
	stripes = (in->width + 327) / 328;
	stripes = ((stripes + cores - 1) / cores) * cores;
	*bytes = 2268 + 624 * stripes;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_bypass_size);

static void bypass_module_1d(u8 *data, u32 length, u32 grid, u32 first,
			     u32 port, bool horizontal, u32 fetch, u32 fetch_last)
{
	static const u16 planes[] = { 130, 162, 194, 226 };
	static const u16 full_crops[] = { 278, 286, 290, 294, 298, 302, 310, 318 };
	static const u16 half_crops[] = { 282, 306, 314 };
	u32 i;

	put_unaligned_le16(1, data + 32);
	put_unaligned_le16(length - 1, data + 34);
	put_unaligned_le16(horizontal && first ? 1 : 0xffff, data + 36);
	put_unaligned_le16(horizontal ? 1 : 2, data + 38);
	put_unaligned_le16(24, data + 42);
	put_unaligned_le16(grid, data + 84);
	put_unaligned_le16(65536 / grid, data + 86);
	put_unaligned_le16(first / grid, data + 88);
	put_unaligned_le16(first % grid, data + 90);
	put_unaligned_le16((first % grid) * (65536 / grid), data + 92);
	for (i = 0; i < ARRAY_SIZE(planes); i++) {
		u32 off = planes[i];
		bool chroma = off == (port == 9 ? 162 : 226);

		put_unaligned_le16(chroma ? 64 : 32, data + off);
		put_unaligned_le16(chroma, data + off + 2);
		put_unaligned_le16(length, data + off + 4);
		put_unaligned_le16(3, data + off + 10);
	}
	for (i = 0; i < ARRAY_SIZE(full_crops); i++)
		put_unaligned_le16(length - 1, data + full_crops[i]);
	for (i = 0; i < ARRAY_SIZE(half_crops); i++)
		put_unaligned_le16(length / 2 - 1, data + half_crops[i]);
	if (horizontal) {
		put_unaligned_le16(first - fetch, data + 276);
		put_unaligned_le16(first - fetch + length - 1, data + 278);
		put_unaligned_le16(fetch_last - fetch, data + 282);
	}
}

int camss_a5_ipe_ltm_size(const struct camss_a5_ipe_io *io, u32 cores,
			 const struct camss_a5_ipe_ltm_geometry *ltm, u32 *bytes)
{
	u32 checked, stripes;
	int ret;

	if (!ltm || !bytes)
		return -EINVAL;
	ret = camss_a5_ipe_bypass_size(io, cores, &checked);
	if (ret)
		return ret;
	if (!(ltm->image_processing && ltm->mask_filter)) {
		*bytes = checked;
		return 0;
	}
	stripes = (io->input.width + 323) / 324;
	stripes = ((stripes + cores - 1) / cores) * cores;
	*bytes = 2268 + 624 * stripes;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_ltm_size);

static int ipe_encode_striping(void *buffer, u32 capacity,
			       const struct camss_a5_ipe_io *io, u32 cores,
			       const struct camss_a5_ipe_ltm_geometry *ltm,
			       u32 *used)
{
	struct camss_a5_ipe_partition parts[26];
	u8 *output = buffer;
	bool mask;
	u32 bytes, count, i, width, height, port;
	int ret;

	if (!output || !used || !ltm)
		return -EINVAL;
	mask = ltm->image_processing && ltm->mask_filter;
	ret = camss_a5_ipe_ltm_size(io, cores, ltm, &bytes);
	if (ret)
		return ret;
	if (capacity < bytes)
		return -ENOSPC;
	ret = mask ?
		camss_a5_ipe_ltm_partitions(io->input.width, cores, parts,
					 ARRAY_SIZE(parts), &count) :
		camss_a5_ipe_bypass_partitions(io->input.width, cores, parts,
					  ARRAY_SIZE(parts), &count);
	if (ret)
		return ret;
	width = io->input.width;
	height = io->input.height;
	port = io->output_port;
	/* Every fallible check precedes writes, including the returned size. */
	memset(output, 0, bytes);
	put_unaligned_le16(20, output + (port == 9 ? 2 : 4));
	put_unaligned_le32(1, output + 20);
	bypass_module_1d(output + 24, height, (max(height, 240U) + 8) / 9,
			 0, port, false, 0, 0);
	put_unaligned_le16(height - 1, output + 376);
	put_unaligned_le16(height - 1, output + (port == 9 ? 396 : 392));
	/* This full-pass field drives the physical IP_EN bit, not DC.
	 * Honour the explicit request; DC/coarse-pass policy belongs upstream.
	 */
	put_unaligned_le32(ltm->image_processing, output + 416);
	put_unaligned_le32(22, output + 424);
	put_unaligned_le16(count, output + 428);
	put_unaligned_le32(2248, output + 432);
	for (i = 0; i < count; i++) {
		const struct camss_a5_ipe_partition *p = parts + i;
		u8 *stripe = output + 2268 + 624 * i;
		u32 first = mask && p->output_first >= 2 ?
			p->output_first - 2 : p->output_first;
		u32 last = mask && p->output_last + 2U < width ?
			p->output_last + 2 : p->output_last;
		u32 length = p->output_last - p->output_first + 1;
		s32 x = (s32)p->output_first - (s32)(width / 2);
		s32 y = -(s32)(height / 2);

		put_unaligned_le16(p->fetch_first, stripe + 4);
		put_unaligned_le16(p->fetch_last, stripe + 6);
		put_unaligned_le16(p->output_first, stripe + (port == 9 ? 24 : 20));
		put_unaligned_le16(p->output_last, stripe + (port == 9 ? 26 : 22));
		put_unaligned_le16((p->output_first == 0) |
				   ((p->output_last == width - 1) << 8), stripe + 44);
		bypass_module_1d(stripe + 48, length, (max(width, 320U) + 11) / 12,
				 p->output_first, port, true, p->fetch_first, p->fetch_last);
		if (mask) {
			u32 grid = (max(width, 320U) + 11) / 12;
			u32 phase = first % grid;

			put_unaligned_le16(last - first, stripe + 48 + 34);
			put_unaligned_le16(first / grid, stripe + 48 + 88);
			put_unaligned_le16(phase, stripe + 48 + 90);
			put_unaligned_le16(phase * (65536 / grid), stripe + 48 + 92);
			put_unaligned_le16(first - p->fetch_first, stripe + 48 + 276);
			put_unaligned_le16(last - p->fetch_first, stripe + 48 + 278);
		}
		put_unaligned_le16(p->output_first, stripe + 568);
		put_unaligned_le32(x * x + y * y, stripe + 576);
		put_unaligned_le16(x, stripe + 580);
		put_unaligned_le16(y, stripe + 582);
		put_unaligned_le32((p->fetch_last - p->fetch_first + 1) * height,
				   stripe + 600);
		put_unaligned_le16(257, stripe + 618);
		put_unaligned_le16(257, stripe + 620);
	}
	*used = bytes;
	return 0;
}

int camss_a5_ipe_encode_bypass_striping(void *buffer, u32 capacity,
				      const struct camss_a5_ipe_io *io,
				      u32 cores, u32 *used)
{
	const struct camss_a5_ipe_ltm_geometry disabled = {};

	return ipe_encode_striping(buffer, capacity, io, cores, &disabled, used);
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_encode_bypass_striping);

int camss_a5_ipe_encode_ltm_striping(void *buffer, u32 capacity,
				   const struct camss_a5_ipe_io *io, u32 cores,
				   const struct camss_a5_ipe_ltm_geometry *ltm,
				   u32 *used)
{
	return ipe_encode_striping(buffer, capacity, io, cores, ltm, used);
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_encode_ltm_striping);

/* Native SM8150 PD10 layout for an explicit quarter-resolution input.
 * This describes backing; it neither produces pixels nor starts DMA. */
int camss_a5_ipe_pd10_quarter(const struct camss_a5_nv12_layout *full,
			    struct camss_a5_pd10_layout *result)
{
	struct camss_a5_pd10_layout layout;

	if (!full || !result || full->width < 128 || full->width > 8192 ||
	    full->height < 128 || full->height > 8192 ||
	    ((full->width | full->height | full->stride | full->slice_height) & 1) ||
	    full->stride < full->width || full->slice_height < full->height)
		return -EINVAL;
	/* Even ceiling of ceil(full / 4), as in the retained native allocator. */
	layout.width = ((full->width + 7) / 4) & ~1U;
	layout.height = ((full->height + 7) / 4) & ~1U;
	/* Each PD10 2x2 block occupies eight bytes in one plane. The native
	 * format helper aligns rows to 192 bytes and packed rows to four.
	 * Newer reference sources' 256-byte/32-line rules differ. */
	layout.stride = ((layout.width * 4 + 191) / 192) * 192;
	layout.slice_height = ((layout.height + 7) / 8) * 4;
	layout.bytes = layout.stride * layout.slice_height;
	*result = layout;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_pd10_quarter);

/* SPDX-License-Identifier: GPL-2.0-only */
/* Native DC resampling and pass geometry; no copied opaque striping data. */
static u32 ltm_dc_exponent(u32 width, u32 height)
{
	if (width >= 288 && height > 215)
		return 0;
	if (width >= 144 && height >= 108)
		return 1;
	return 2;
}

int camss_a5_ipe_ltm_pd10_size(const struct camss_a5_ipe_io *io, u32 cores,
			     const struct camss_a5_ipe_ltm_geometry *ltm,
			     u32 *bytes)
{
	struct camss_a5_pd10_layout coarse;
	u32 single;
	int ret;

	ret = camss_a5_ipe_ltm_size(io, cores, ltm, &single);
	if (ret)
		return ret;
	if (io->output_port != 8 || io->input.uv_swap)
		return -EOPNOTSUPP;
	if (ltm->data_collection && !(ltm->image_processing && ltm->mask_filter))
		return -EOPNOTSUPP;
	ret = camss_a5_ipe_pd10_quarter(&io->input, &coarse);
	if (ret)
		return ret;
	*bytes = single + (ltm->data_collection ? 624 * ((coarse.width + 327) / 328) : 0);
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_ltm_pd10_size);

static void ltm_dc_axis(u8 *data, u32 input_length, u32 target_length,
		       u32 first, u32 last, u32 target_first, u32 target_last,
		       u32 fetch, u32 fetch_last, u32 exponent, bool horizontal)
{
	u32 length = last - first + 1;
	u32 grid = (max(input_length, horizontal ? 320U : 240U) +
		    (horizontal ? 11 : 8)) / (horizontal ? 12 : 9);
	u32 step = ((u64)input_length << 21) / target_length;
	u32 resample_first = ((u64)target_first * step) >> 21;
	u32 resample_offset = resample_first - first;
	u32 phase = first % grid;
	u32 table_step = 24 >> exponent, half = table_step / 2;
	u32 index = target_first < half ? 0 : (target_first - half) / table_step + 1;
	u32 table_phase = target_first < half ? target_first : (target_first - half) % table_step;
	u32 table_first = index ? index - 1 : 0;
	u32 table_end = target_last + 1 < half ? 0 : (target_last + 1 - half) / table_step + 1;
	u32 columns = 12;

	if (table_step * 12 - 1 > target_last)
		columns = table_end > table_first + 2 ? table_end - 2 : table_first;
	put_unaligned_le16(1, data + 32);
	put_unaligned_le16(length - 1, data + 34);
	put_unaligned_le16(horizontal && first ? 1 : 0xffff, data + 36);
	put_unaligned_le16(horizontal ? 1 : 2, data + 38);
	put_unaligned_le16(24, data + 42);
	put_unaligned_le16(resample_offset, data + 44);
	put_unaligned_le16(1, data + 46);
	put_unaligned_le16(resample_offset, data + 52);
	put_unaligned_le32((target_first * step) & 0x1fffff, data + 56);
	put_unaligned_le32(step, data + 60);
	put_unaligned_le16(3, data + 64);
	put_unaligned_le16(last - resample_first, data + 66);
	if (horizontal) {
		put_unaligned_le16(index, data + 68);
		put_unaligned_le16(table_phase, data + 70);
		put_unaligned_le16(!first, data + 72);
		put_unaligned_le16(table_first, data + 74);
		put_unaligned_le16(columns, data + 76);
		put_unaligned_le16(target_last - target_first, data + 80);
	}
	put_unaligned_le16(grid, data + 84);
	put_unaligned_le16(65536 / grid, data + 86);
	put_unaligned_le16(first / grid, data + 88);
	put_unaligned_le16(phase, data + 90);
	put_unaligned_le16(phase * (65536 / grid), data + 92);
	put_unaligned_le16(first - fetch, data + 276);
	put_unaligned_le16(last - fetch, data + 278);
	put_unaligned_le16(horizontal ? fetch_last - fetch : length / 2 - 1, data + 282);
}

int camss_a5_ipe_encode_ltm_pd10_striping(void *buffer, u32 capacity,
					const struct camss_a5_ipe_io *io,
					u32 cores,
					const struct camss_a5_ipe_ltm_geometry *ltm,
					u32 *used)
{
	struct camss_a5_ipe_partition parts[7];
	struct camss_a5_pd10_layout coarse;
	u8 *output = buffer;
	u32 bytes, single, count, full_count, i, exponent, target_width, target_height;
	u32 phase_step;
	int ret;

	if (!buffer || !used)
		return -EINVAL;
	ret = camss_a5_ipe_ltm_pd10_size(io, cores, ltm, &bytes);
	if (ret)
		return ret;
	if (capacity < bytes)
		return -ENOSPC;
	ret = camss_a5_ipe_pd10_quarter(&io->input, &coarse);
	if (ret)
		return ret;
	count = (coarse.width + 327) / 328;
	for (i = 0; i < count; i++) {
		u32 index = count - i - 1;

		parts[i].output_first = ((coarse.width * index) / count) & ~1U;
		parts[i].output_last = (((coarse.width * (index + 1)) / count) & ~1U) - 1;
	}
	ret = camss_a5_ipe_encode_ltm_striping(buffer, capacity, io, cores, ltm, &single);
	if (ret)
		return ret;
	if (!ltm->data_collection) {
		*used = single;
		return 0;
	}
	exponent = ltm_dc_exponent(coarse.width, coarse.height);
	target_width = 288 >> exponent;
	target_height = 216 >> exponent;
	phase_step = ((u64)coarse.width << 21) / target_width;
	full_count = get_unaligned_le16(output + 428);
	memmove(output + 2268 + count * 624, output + 2268, single - 2268);
	memmove(output + 436, output + 24, 412);
	memset(output + 24, 0, 412);
	memset(output + 2268, 0, count * 624);
	put_unaligned_le32(2, output + 20);
	ltm_dc_axis(output + 24, coarse.height, target_height, 0, coarse.height - 1,
		    0, target_height - 1, 0, coarse.height - 1, exponent, false);
	put_unaligned_le16(coarse.height - 1, output + 24 + 270);
	put_unaligned_le16(coarse.height / 2 - 1, output + 24 + 274);
	put_unaligned_le16(coarse.height - 1, output + 24 + 376);
	put_unaligned_le16(coarse.height - 1, output + 24 + 352);
	put_unaligned_le16(1, output + 24 + 390);
	put_unaligned_le16(count, output + 24 + 404);
	put_unaligned_le32(2248, output + 24 + 408);
	put_unaligned_le16(coarse.height - 1, output + 436 + 360);
	put_unaligned_le32(434, output + 436 + 400);
	put_unaligned_le32(1836 + count * 624, output + 436 + 408);
	for (i = 0; i < count; i++) {
		const struct camss_a5_ipe_partition *p = parts + i;
		u8 *stripe = output + 2268 + i * 624;
		u32 target_first = p->output_first * target_width / coarse.width;
		u32 target_last = (p->output_last + 1) * target_width / coarse.width - 1;
		u32 first, last = p->output_last, fetch, fetch_last;

		target_first = target_first > 2 ? target_first - 2 : 0;
		first = (((u64)target_first * phase_step) >> 21) & ~1U;
		fetch = (first < 4 ? 0 : first - 4) & ~1U;
		fetch_last = min(last + 4, coarse.width - 1);
		put_unaligned_le16(fetch, stripe + 4);
		put_unaligned_le16(fetch_last, stripe + 6);
		put_unaligned_le16(p->output_first, stripe + 28);
		put_unaligned_le16(p->output_last, stripe + 30);
		put_unaligned_le16((p->output_first == 0) | ((p->output_last == coarse.width - 1) << 8), stripe + 44);
		ltm_dc_axis(stripe + 48, coarse.width, target_width, first, last,
			    target_first, target_last - (last < coarse.width - 1 ? 2 : 0),
			    fetch, fetch_last, exponent, true);
		put_unaligned_le16(p->output_first - fetch, stripe + 316);
		put_unaligned_le16(last - fetch, stripe + 318);
		put_unaligned_le16(p->output_first - fetch, stripe + 320);
		put_unaligned_le16(last - fetch, stripe + 322);
		put_unaligned_le32((fetch_last - fetch + 1) * coarse.height, stripe + 600);
		put_unaligned_le32(count == 1, stripe + 604);
	}
	for (i = 0; i < full_count; i++) {
		u8 *stripe = output + 2268 + count * 624 + i * 624;
		u32 first = get_unaligned_le16(stripe + 20) / 4;
		u32 last = get_unaligned_le16(stripe + 22) / 4;

		put_unaligned_le16(first < 10 ? 0 : first - 10, stripe + 12);
		put_unaligned_le16(min(last + 10, coarse.width - 1), stripe + 14);
	}
	*used = bytes;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_encode_ltm_pd10_striping);
