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
	const struct camss_a5_ipe_buffer *buffers[6];
	u8 *output = destination;
	u32 input_y, output_y, used, i, j;
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
	for (i = 0; i < 6; i++) {
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
/* Full-frame, unity-scale, 8-bit linear YUV with IQ modules disabled.
 * The target striping policy uses 328-pixel output partitions, rounds the
 * count to the number of cores, and aligns internal boundaries to chroma.
 * A four-pixel fetch halo is retained at internal boundaries.
 * This describes ranges only; it does not serialize the full firmware blob.
 */
int camss_a5_ipe_bypass_partitions(u32 width, u32 cores,
				 struct camss_a5_ipe_partition *output,
				 u32 capacity, u32 *count)
{
	u32 stripes, i;

	if (!output || !count || width < 128 || width > 8192 || (width & 1) ||
	    !cores || cores > 2)
		return -EINVAL;
	stripes = (width + 327) / 328;
	stripes = ((stripes + cores - 1) / cores) * cores;
	if (capacity < stripes)
		return -ENOSPC;
	for (i = 0; i < stripes; i++) {
		u32 index = stripes - i - 1;
		u32 first = ((width * index) / stripes) & ~1U;
		u32 last = (((width * (index + 1)) / stripes) & ~1U) - 1;

		output[i].output_first = first;
		output[i].output_last = last;
		output[i].fetch_first = first < 4 ? 0 : first - 4;
		output[i].fetch_last = last + 4 < width ? last + 4 : width - 1;
	}
	*count = stripes;
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_bypass_partitions);

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

int camss_a5_ipe_encode_bypass_striping(void *buffer, u32 capacity,
				      const struct camss_a5_ipe_io *io,
				      u32 cores, u32 *used)
{
	struct camss_a5_ipe_partition parts[26];
	u8 *output = buffer;
	u32 bytes, count, i, width, height, port;
	int ret;

	if (!output || !used)
		return -EINVAL;
	ret = camss_a5_ipe_bypass_size(io, cores, &bytes);
	if (ret)
		return ret;
	if (capacity < bytes)
		return -ENOSPC;
	ret = camss_a5_ipe_bypass_partitions(io->input.width, cores, parts,
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
	put_unaligned_le32(22, output + 424);
	put_unaligned_le16(count, output + 428);
	put_unaligned_le32(2248, output + 432);
	for (i = 0; i < count; i++) {
		const struct camss_a5_ipe_partition *p = parts + i;
		u8 *stripe = output + 2268 + 624 * i;
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
EXPORT_SYMBOL_GPL(camss_a5_ipe_encode_bypass_striping);
