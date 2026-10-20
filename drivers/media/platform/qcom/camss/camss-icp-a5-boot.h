/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_ICP_A5_BOOT_H
#define CAMSS_ICP_A5_BOOT_H

#include <linux/completion.h>
#include <linux/hrtimer.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include "camss-icp-a5.h"
#include "camss-icp-a5-resources.h"

struct camss_a5 {
	struct device *dev;
	struct camss_a5_resources resources;
	struct reset_control *reset;
	phys_addr_t firmware_physical;
	struct camss_a5_fw_memory firmware, shared, heap;
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
	bool power_held, irq_enabled, cpu_enabled, initialized;
	bool init_done, ping_done, pc_done, reset_asserted;
	bool cpu_release_attempted;
	bool boot_expired;
};

/* The owner installs PM callbacks which delegate to runtime_resume/suspend,
 * and enables runtime PM after init. Lifecycle/commands use a sleeping mutex.
 * Start failure attempts CPU/IRQ quiescence but retains backing and any PM
 * reference. The owner must call stop after either start success or failure.
 * Inspect firmware RAM only after successful quiescence, with CPU/IRQ off,
 * block reset asserted and all mappings/PM ownership still held.
 * Failed teardown retains mapped backing; do not unbind until cleanup succeeds.
 */
int camss_a5_init(struct platform_device *pdev, struct camss_a5 *a5);
int camss_a5_runtime_resume(struct camss_a5 *a5);
int camss_a5_runtime_suspend(struct camss_a5 *a5);
int camss_a5_start(struct camss_a5 *a5, const struct firmware *firmware);
int camss_a5_ping(struct camss_a5 *a5, u64 cookie);
int camss_a5_stop(struct camss_a5 *a5);
bool camss_a5_backing_owned(const struct camss_a5 *a5);

#endif
