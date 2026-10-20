// SPDX-License-Identifier: GPL-2.0-only
/* A5 HFI wire layout follows the original SM8150 GPL kernel interface.
 * See ../oem-icp-sources.json in stage352 for the pinned source revision. */
#include <linux/build_bug.h>
#include <linux/compiler.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/limits.h>
#include <linux/minmax.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "camss-icp-a5.h"

static_assert(sizeof(struct camss_a5_qhdr) == 956);
static_assert(offsetof(struct camss_a5_qhdr, start) == 124);
static_assert(offsetof(struct camss_a5_qhdr, type) == 188);
static_assert(offsetof(struct camss_a5_qhdr, words) == 252);
static_assert(offsetof(struct camss_a5_qhdr, read) == 828);
static_assert(offsetof(struct camss_a5_qhdr, write) == 892);
static_assert(sizeof(struct camss_a5_qtable) == 3848);

int camss_a5_qtable_init(struct camss_a5_qtable *table,
			 const u64 iovas[CAMSS_A5_QUEUE_COUNT],
			 const u32 bytes[CAMSS_A5_QUEUE_COUNT])
{
	unsigned int i;

	if (!table || !iovas || !bytes)
		return -EINVAL;
	for (i = 0; i < CAMSS_A5_QUEUE_COUNT; i++) {
		if (bytes[i] < 8 || bytes[i] % 4 || iovas[i] % 4 ||
		    iovas[i] > U32_MAX || bytes[i] - 1 > U32_MAX - iovas[i])
			return -EINVAL;
	}

	memset(table, 0, sizeof(*table));
	table->version = cpu_to_le32(U32_MAX);
	table->bytes = cpu_to_le32(sizeof(*table));
	table->header_offset = cpu_to_le32(offsetof(struct camss_a5_qtable, queues));
	table->header_bytes = cpu_to_le32(sizeof(struct camss_a5_qhdr));
	table->count = cpu_to_le32(CAMSS_A5_QUEUE_COUNT);
	table->active = cpu_to_le32(CAMSS_A5_QUEUE_COUNT);
	for (i = 0; i < CAMSS_A5_QUEUE_COUNT; i++) {
		struct camss_a5_qhdr *q = &table->queues[i];

		q->status = cpu_to_le32(1);
		q->start = cpu_to_le32(iovas[i]);
		q->type = cpu_to_le32(i);
		q->words = cpu_to_le32(bytes[i] / 4);
		q->rx_watermark = cpu_to_le32(1);
		q->tx_watermark = cpu_to_le32(i == CAMSS_A5_DBG ? 1024 : 1);
		q->rx_request = cpu_to_le32(i != CAMSS_A5_DBG);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_qtable_init);

int camss_a5_ring_init(struct camss_a5_ring *ring,
		      struct camss_a5_qhdr *header, void *data, u32 bytes)
{
	if (!ring || !header || !data || bytes < 8 || bytes % 4 ||
	    le32_to_cpu(READ_ONCE(header->words)) != bytes / 4)
		return -EINVAL;
	ring->header = header;
	ring->data = data;
	ring->bytes = bytes;
	mutex_init(&ring->lock);
	return 0;
}
EXPORT_SYMBOL_GPL(camss_a5_ring_init);

static int ring_state(struct camss_a5_ring *ring, u32 *read, u32 *write)
{
	u32 words = ring->bytes / 4;

	if (le32_to_cpu(READ_ONCE(ring->header->status)) != 1)
		return -ENODEV;
	if (le32_to_cpu(READ_ONCE(ring->header->words)) != words ||
	    le32_to_cpu(READ_ONCE(ring->header->packet_size)))
		return -EPROTO;
	*read = le32_to_cpu(READ_ONCE(ring->header->read));
	*write = le32_to_cpu(READ_ONCE(ring->header->write));
	if (*read >= words || *write >= words)
		return -EPROTO;
	return 0;
}

int camss_a5_ring_write(struct camss_a5_ring *ring, const void *packet,
		       size_t bytes)
{
	u32 read, write, free_words, first, count, next, words;
	int ret;

	if (!ring || !packet || bytes < 4 || bytes % 4 ||
	    bytes > U32_MAX || get_unaligned_le32(packet) != bytes)
		return -EINVAL;
	mutex_lock(&ring->lock);
	ret = ring_state(ring, &read, &write);
	if (ret)
		goto unlock;
	words = ring->bytes / 4;
	count = bytes / 4;
	free_words = write >= read ? words - (write - read) : read - write;
	if (count >= free_words) {
		ret = -ENOSPC;
		goto unlock;
	}
	/* Pair with the firmware consumer publishing its read index. */
	dma_rmb();
	first = min_t(u32, count, words - write);
	memcpy((u8 *)ring->data + write * 4, packet, first * 4);
	if (first < count)
		memcpy(ring->data, (const u8 *)packet + first * 4, (count - first) * 4);
	next = write + count;
	if (next >= words)
		next -= words;
	/* The firmware must see the complete payload before the new index. */
	dma_wmb();
	WRITE_ONCE(ring->header->write, cpu_to_le32(next));
	/* Publish the index before the owner's subsequent doorbell MMIO. */
	dma_wmb();
unlock:
	mutex_unlock(&ring->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(camss_a5_ring_write);

int camss_a5_ring_read(struct camss_a5_ring *ring, void *packet,
		      size_t capacity, size_t *bytes)
{
	u32 read, write, available, size, count, words, first, next;
	int ret;

	if (!ring || !packet || !bytes)
		return -EINVAL;
	mutex_lock(&ring->lock);
	ret = ring_state(ring, &read, &write);
	if (ret)
		goto unlock;
	if (read == write) {
		ret = -EAGAIN;
		goto unlock;
	}
	/* Pair with the producer's payload-before-write-index publication. */
	dma_rmb();
	words = ring->bytes / 4;
	available = write >= read ? write - read : words - read + write;
	size = get_unaligned_le32((u8 *)ring->data + read * 4);
	if (size < 4 || size % 4 || size / 4 >= words) {
		ret = -EBADMSG;
		goto unlock;
	}
	count = size / 4;
	if (count > available) {
		ret = -EAGAIN;
		goto unlock;
	}
	if (size > capacity) {
		ret = -ENOSPC;
		goto unlock;
	}
	first = min_t(u32, count, words - read);
	memcpy(packet, (u8 *)ring->data + read * 4, first * 4);
	if (first < count)
		memcpy((u8 *)packet + first * 4, ring->data, (count - first) * 4);
	next = read + count;
	if (next >= words)
		next -= words;
	/* Finish the copy before allowing the producer to reuse these words. */
	dma_mb();
	WRITE_ONCE(ring->header->read, cpu_to_le32(next));
	*bytes = size;
unlock:
	mutex_unlock(&ring->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(camss_a5_ring_read);
