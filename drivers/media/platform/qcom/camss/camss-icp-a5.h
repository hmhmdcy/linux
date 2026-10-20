/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_ICP_A5_H
#define CAMSS_ICP_A5_H

#include <linux/firmware.h>
#include <linux/mutex.h>
#include <linux/types.h>

#define CAMSS_A5_QUEUE_COUNT 3
#define CAMSS_A5_QUEUE_SLOTS 4
#define CAMSS_A5_FW_WINDOW (5U * 1024U * 1024U)

enum camss_a5_queue_id {
	CAMSS_A5_CMD,
	CAMSS_A5_MSG,
	CAMSS_A5_DBG,
};

/* Firmware ABI: each live word follows 60 bytes of padding. The final
 * padding is 60 bytes too, so this structure is 956, not 896 or 960 bytes.
 * Do not add cacheline alignment to these fields or this structure. */
struct camss_a5_qhdr {
	u32 pad0[15]; __le32 status;
	u32 pad1[15]; __le32 start;
	u32 pad2[15]; __le32 type;
	u32 pad3[15]; __le32 words;
	u32 pad4[15]; __le32 packet_size;
	u32 pad5[15]; __le32 dropped;
	u32 pad6[15]; __le32 rx_watermark;
	u32 pad7[15]; __le32 tx_watermark;
	u32 pad8[15]; __le32 rx_request;
	u32 pad9[15]; __le32 tx_request;
	u32 pad10[15]; __le32 rx_irq;
	u32 pad11[15]; __le32 tx_irq;
	u32 pad12[15]; __le32 read;
	u32 pad13[15]; __le32 write;
	u32 pad14[15];
};

struct camss_a5_qtable {
	__le32 version;
	__le32 bytes;
	__le32 header_offset;
	__le32 header_bytes;
	__le32 count;
	__le32 active;
	/* The original host table reserves a fourth, inactive header. */
	struct camss_a5_qhdr queues[CAMSS_A5_QUEUE_SLOTS];
};

struct camss_a5_ring {
	struct camss_a5_qhdr *header;
	void *data;
	u32 bytes;
	struct mutex lock;
};

struct camss_a5_fw_layout {
	u32 entry;
	u32 span;
	u32 segments;
};

struct device;
struct iommu_domain;

struct camss_a5_fw_memory {
	struct device *device;
	struct iommu_domain *domain;
	void *cpu;
	dma_addr_t dma;
	u32 bytes;
	u32 mapped;
	u32 iova;
	bool privileged;
	bool carveout;
	phys_addr_t physical;
};

/* The owning platform driver serializes these lifecycle operations, keeps
 * the device bound to its default DMA domain, and stops A5 before teardown.
 * Start with a zeroed structure. A map failure normally rolls back; if an
 * unmap fails, mapped remains nonzero and freeing the backing is refused.
 */
/* Allocate RAM for a fixed alias fully covered by a reserved IOVA region. */
int camss_a5_iova_memory_alloc(struct device *device,
			       struct camss_a5_fw_memory *memory,
			       u32 iova, u32 bytes);
int camss_a5_fw_memory_alloc(struct device *device,
			     struct camss_a5_fw_memory *memory);
/* Caller owns a DT-reserved, no-map firmware RAM resource. */
int camss_a5_fw_carveout_alloc(struct device *device,
			       struct camss_a5_fw_memory *memory,
			       phys_addr_t physical, u32 bytes);
int camss_a5_fw_memory_map(struct camss_a5_fw_memory *memory);
int camss_a5_fw_memory_unmap(struct camss_a5_fw_memory *memory);
int camss_a5_fw_memory_free(struct camss_a5_fw_memory *memory);

/* These prepare coherent RAM only. The owning device driver supplies
 * already-mapped 32-bit firmware IOVAs and performs the MMIO handshake. */
int camss_a5_qtable_init(struct camss_a5_qtable *table,
			 const u64 iovas[CAMSS_A5_QUEUE_COUNT],
			 const u32 bytes[CAMSS_A5_QUEUE_COUNT]);
int camss_a5_ring_init(struct camss_a5_ring *ring,
		      struct camss_a5_qhdr *header, void *data, u32 bytes);
/* Process/workqueue context only; the mutex serializes host callers.
 * Successful write is published before return; caller then rings A5.
 * A full ring or incomplete input never consumes or overwrites data. */
int camss_a5_ring_write(struct camss_a5_ring *ring, const void *packet,
		       size_t bytes);
int camss_a5_ring_read(struct camss_a5_ring *ring, void *packet,
		      size_t capacity, size_t *bytes);

int camss_a5_fw_inspect(const struct firmware *firmware, u32 window,
			struct camss_a5_fw_layout *layout);
int camss_a5_fw_load(const struct firmware *firmware, void *destination,
		    u32 window, struct camss_a5_fw_layout *layout);

#endif
