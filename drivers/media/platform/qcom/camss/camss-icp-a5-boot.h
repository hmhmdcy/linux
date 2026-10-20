/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_ICP_A5_BOOT_H
#define CAMSS_ICP_A5_BOOT_H

#include <linux/completion.h>
#include <linux/hrtimer.h>
#include <linux/list.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include "camss-icp-a5.h"
#include "camss-icp-a5-resources.h"

struct camss_a5;

enum camss_a5_ipe_type {
	CAMSS_A5_IPE_RT = 2,
	CAMSS_A5_IPE_NON_RT = 3,
};

/* Zero-initialize before first use. Keep this object alive until destroy or
 * successful stop; do not mutate its fields or use one object with two owners.
 * Stop destroys live known sessions, or invalidates them after forced reset.
 * No image buffers or frame commands are submitted by this interface yet.
 */
struct camss_a5_nv12_layout {
	u32 width, height, stride, slice_height;
	bool uv_swap;
};

struct camss_a5_ipe_io {
	struct camss_a5_nv12_layout input, output;
	u32 output_port; /* 8: display, 9: video */
};

struct camss_a5_session {
	struct list_head node;
	struct camss_a5 *a5;
	u32 handle, type;
	struct camss_a5_ipe_io io;
	u32 scratch_bytes;
	bool configured;
};

struct camss_a5 {
	struct device *dev;
	struct camss_a5_resources resources;
	struct reset_control *reset;
	phys_addr_t firmware_physical;
	struct camss_a5_fw_memory firmware, shared, heap, config;
	struct camss_a5_fw_memory request, image_input, image_output;
	struct camss_a5_ring rings[CAMSS_A5_QUEUE_COUNT];
	struct camss_a5_qtable *table;
	struct completion response;
	struct mutex lock;
	spinlock_t control_lock;
	struct hrtimer boot_timer;
	void *message, *debug;
	int irq, command_error, cleanup_error, pc_error;
	u32 expected, fw_version, init_response, last_type;
	u32 response_before;
	u32 interrupts, messages, debug_packets, watchdogs, protocol_errors;
	u32 init_bytes, init_data[32];
	u32 queue_state[6];
	char sfr[512];
	u64 ping_cookie;
	struct list_head sessions;
	u64 session_sequence, session_cookie, session_cookie2;
	u32 session_opcode, session_reply_handle;
	u32 session_count, session_creates, session_destroys, session_peak;
	bool command_queued, session_reply, session_uncertain;
	struct camss_a5_session *config_owner;
	bool config_fw_mapped;
	bool engine_dma_capable, ipe_dma_pending, request_fw_mapped;
	struct camss_a5_session *request_owner;
	u32 frames_submitted, frames_completed, transfer_maps, transfer_unmaps;
	u32 session_scratch_bytes, io_configs, config_maps, config_unmaps;
	u32 session_header_error, session_payload_status;
	bool power_held, irq_enabled, cpu_enabled, initialized;
	bool init_done, ping_done, pc_done, reset_asserted;
	bool cpu_release_attempted;
	bool boot_expired;
	void (*failure_snapshot)(struct camss_a5 *a5);
};

/* The owner installs PM callbacks which delegate to runtime_resume/suspend,
 * and enables runtime PM after init. Lifecycle/commands use a sleeping mutex.
 * Start failure attempts CPU/IRQ quiescence but retains backing and any PM
 * reference. The owner must call stop after either start success or failure.
 * Inspect firmware RAM only after successful quiescence, with CPU/IRQ off,
 * block reset asserted and all mappings/PM ownership still held.
 * An optional failure_snapshot observer runs under the lifecycle mutex,
 * before CPU/block reset and release of any backing or PM reference. Keep
 * the observer and its data valid through start; it must be bounded, avoid
 * allocation/MMIO and never re-enter lifecycle/command operations.
 * Failed teardown retains mapped backing; do not unbind until cleanup succeeds.
 */
int camss_a5_init(struct platform_device *pdev, struct camss_a5 *a5);
int camss_a5_runtime_resume(struct camss_a5 *a5);
int camss_a5_runtime_suspend(struct camss_a5 *a5);
int camss_a5_start(struct camss_a5 *a5, const struct firmware *firmware);
int camss_a5_ping(struct camss_a5 *a5, u64 cookie);
int camss_a5_stop(struct camss_a5 *a5);
int camss_a5_ipe_create(struct camss_a5 *a5, struct camss_a5_session *session,
			 u32 handle_type);
int camss_a5_ipe_destroy(struct camss_a5 *a5, struct camss_a5_session *session);
/* Configure linear NV12/NV21 without submitting any image DMA or frame. */
int camss_a5_ipe_configure_nv12(struct camss_a5 *a5,
			       struct camss_a5_session *session,
			       const struct camss_a5_ipe_io *io);
bool camss_a5_backing_owned(const struct camss_a5 *a5);

#endif
