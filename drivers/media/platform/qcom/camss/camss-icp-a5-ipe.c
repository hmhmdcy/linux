// SPDX-License-Identifier: GPL-2.0-only
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/limits.h>
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
