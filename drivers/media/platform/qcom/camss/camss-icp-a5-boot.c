// SPDX-License-Identifier: GPL-2.0-only
/* SM8150 A5 firmware transport. No image sessions or IPE commands yet. */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/devcoredump.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#include "camss-icp-a5-boot.h"

#define A5_RESET 0x04
#define A5_CONTROL 0x08
#define A5_INT_ENABLE 0x10
#define A5_INT_CLEAR 0x18
#define A5_INT_STATUS 0x1c
#define A5_HOST_INTERRUPT 0x30
#define A5_FW_VERSION 0x44
#define A5_INIT_REQUEST 0x48
#define A5_INIT_RESPONSE 0x4c
#define A5_SHARED_PTR 0x50
#define A5_SHARED_SIZE 0x54
#define A5_QTABLE_PTR 0x58
#define A5_HEAP_PTR 0x5c
#define A5_HEAP_SIZE 0x60
#define A5_SFR_PTR 0x68
#define A5_QDSS_PTR 0x6c
#define A5_QDSS_SIZE 0x70
#define A5_IO_PTR 0x74
#define A5_IO_SIZE 0x78
#define A5_STATUS 0x200
#define A5_ENABLE BIT(9)
#define A5_WAKE BIT(4)
#define A5_WFI_GATE BIT(12)
#define A5_WFI BIT(7)

#define SHARED_IOVA 0x07400000U
#define SHARED_WINDOW 0x09600000U
#define SHARED_BYTES 0x1e000U
#define HEAP_IOVA 0x10a00000U
#define HEAP_BYTES 0x100000U
#define IO_IOVA 0x10c00000U
#define IO_BYTES 0xcf300000U
#define CMD_OFFSET 0x1000U
#define MSG_OFFSET 0x2000U
#define DBG_OFFSET 0x3000U
#define SFR_OFFSET 0x1c000U
#define CMD_BYTES 4096U
#define MSG_BYTES 4096U
#define DBG_BYTES 102400U
#define SYS_INIT 0x10001U
#define SYS_PC_PREP 0x10002U
#define SYS_PING 0x10005U
#define INIT_DONE 0x20001U
#define PC_DONE 0x20002U
#define PING_ACK 0x20006U
#define EVENT_NOTIFY 0x20008U

static enum hrtimer_restart boot_timeout(struct hrtimer *timer)
{
	struct camss_a5 *a5 = container_of(timer, struct camss_a5, boot_timer);
	unsigned long flags;
	u32 control;

	/* Enforce the same handshake deadline even if a faulting A5 keeps an
	 * SMMU IRQ busy and the sleeping startup thread cannot run promptly.
	 */
	spin_lock_irqsave(&a5->control_lock, flags);
	if (a5->power_held && a5->cpu_enabled) {
		writel(0, a5->resources.sierra + A5_INT_ENABLE);
		control = readl(a5->resources.sierra + A5_CONTROL);
		writel(control & ~(A5_ENABLE | A5_WAKE), a5->resources.sierra + A5_CONTROL);
		WRITE_ONCE(a5->boot_expired, true);
	}
	spin_unlock_irqrestore(&a5->control_lock, flags);
	return HRTIMER_NORESTART;
}

static void command_failure(struct camss_a5 *a5, int error)
{
	WRITE_ONCE(a5->command_error, error);
	complete(&a5->response);
}

static irqreturn_t a5_interrupt(int irq, void *data)
{
	struct camss_a5 *a5 = data;
	u32 status = readl(a5->resources.sierra + A5_INT_STATUS);

	if (!(status & 7))
		return IRQ_NONE;
	writel(status, a5->resources.sierra + A5_INT_CLEAR);
	a5->interrupts++;
	if (status & 6) {
		a5->watchdogs++;
		writel(0, a5->resources.sierra + A5_INT_ENABLE);
		command_failure(a5, -EIO);
	}
	return IRQ_WAKE_THREAD;
}

static void process_message(struct camss_a5 *a5, size_t bytes)
{
	u32 type, error = 0;

	if (bytes < 8)
		goto malformed;
	type = get_unaligned_le32(a5->message + 4);
	a5->messages++;
	a5->last_type = type;
	if (type == EVENT_NOTIFY) {
		command_failure(a5, -EREMOTEIO);
		return;
	}
	if (type != READ_ONCE(a5->expected))
		return;
	switch (type) {
	case INIT_DONE:
		if (bytes < 16)
			goto malformed;
		error = get_unaligned_le32(a5->message + 8);
		a5->init_bytes = min_t(size_t, bytes, sizeof(a5->init_data));
		memcpy(a5->init_data, a5->message, a5->init_bytes);
		a5->init_done = !error;
		break;
	case PC_DONE:
		if (bytes < 12)
			goto malformed;
		error = get_unaligned_le32(a5->message + 8);
		a5->pc_done = !error;
		break;
	case PING_ACK:
		if (bytes != 16 || get_unaligned_le64(a5->message + 8) != a5->ping_cookie)
			goto malformed;
		a5->ping_done = true;
		break;
	default:
		goto malformed;
	}
	WRITE_ONCE(a5->command_error, error ? -EREMOTEIO : 0);
	complete(&a5->response);
	return;
malformed:
	a5->protocol_errors++;
	command_failure(a5, -EPROTO);
}

static irqreturn_t a5_interrupt_thread(int irq, void *data)
{
	struct camss_a5 *a5 = data;
	size_t bytes;
	int ret;

	while (!(ret = camss_a5_ring_read(&a5->rings[CAMSS_A5_MSG],
					 a5->message, MSG_BYTES, &bytes)))
		process_message(a5, bytes);
	if (ret != -EAGAIN) {
		a5->protocol_errors++;
		command_failure(a5, ret);
	}
	while (!(ret = camss_a5_ring_read(&a5->rings[CAMSS_A5_DBG],
					 a5->debug, DBG_BYTES, &bytes)))
		a5->debug_packets++;
	if (ret != -EAGAIN) {
		a5->protocol_errors++;
		command_failure(a5, ret);
	}
	return IRQ_HANDLED;
}

static int send_command(struct camss_a5 *a5, u32 type, u32 response)
{
	__le32 packet[4];
	u32 bytes = type == SYS_PING ? 16 : 8;
	int ret;

	if (!a5->cpu_enabled || !a5->irq_enabled || a5->watchdogs)
		return -EIO;
	reinit_completion(&a5->response);
	WRITE_ONCE(a5->command_error, 0);
	WRITE_ONCE(a5->expected, response);
	packet[0] = cpu_to_le32(bytes);
	packet[1] = cpu_to_le32(type);
	put_unaligned_le64(a5->ping_cookie, packet + 2);
	ret = camss_a5_ring_write(&a5->rings[CAMSS_A5_CMD], packet, bytes);
	if (ret)
		goto done;
	writel(1, a5->resources.sierra + A5_HOST_INTERRUPT);
	if (!wait_for_completion_timeout(&a5->response, msecs_to_jiffies(5000)))
		ret = -ETIMEDOUT;
	else
		ret = READ_ONCE(a5->command_error);
done:
	WRITE_ONCE(a5->expected, 0);
	return ret;
}

bool camss_a5_backing_owned(const struct camss_a5 *a5)
{
	return a5->firmware.cpu || a5->shared.cpu || a5->heap.cpu;
}
EXPORT_SYMBOL_GPL(camss_a5_backing_owned);

/* Fixed little-endian diagnostic record; no image or tuning buffers. */
struct camss_a5_start_dump {
	char magic[8];
	__le32 version;
	__le32 init_response, fw_version;
	__le32 queue_state[6];
	__le32 interrupts, watchdogs, protocol_errors;
	char sfr[512];
};

static void capture_shared_state(struct camss_a5 *a5)
{
	u32 declared;
	size_t bytes;
	unsigned int i;

	if (!a5->shared.cpu || a5->table != a5->shared.cpu ||
	    a5->shared.bytes < sizeof(*a5->table))
		return;
	dma_rmb();
	for (i = 0; i < CAMSS_A5_QUEUE_COUNT; i++) {
		a5->queue_state[i * 2] = le32_to_cpu(READ_ONCE(a5->table->queues[i].read));
		a5->queue_state[i * 2 + 1] = le32_to_cpu(READ_ONCE(a5->table->queues[i].write));
	}
	memset(a5->sfr, 0, sizeof(a5->sfr));
	if (a5->shared.bytes <= SFR_OFFSET + sizeof(__le32))
		return;
	declared = get_unaligned_le32(a5->shared.cpu + SFR_OFFSET);
	if (declared <= sizeof(__le32))
		return;
	bytes = min_t(size_t, declared - sizeof(__le32), sizeof(a5->sfr) - 1);
	bytes = min_t(size_t, bytes, a5->shared.bytes - SFR_OFFSET - sizeof(__le32));
	memcpy(a5->sfr, a5->shared.cpu + SFR_OFFSET + sizeof(__le32), bytes);
}

static void publish_start_failure(struct camss_a5 *a5)
{
	struct camss_a5_start_dump *dump;
	unsigned int i;

	if (!a5->cpu_release_attempted)
		return;
	capture_shared_state(a5);
	dump = vzalloc(sizeof(*dump));
	if (!dump)
		return;
	memcpy(dump->magic, "CAMSSA5", sizeof(dump->magic));
	dump->version = cpu_to_le32(1);
	dump->init_response = cpu_to_le32(a5->init_response);
	dump->fw_version = cpu_to_le32(a5->fw_version);
	for (i = 0; i < ARRAY_SIZE(dump->queue_state); i++)
		dump->queue_state[i] = cpu_to_le32(a5->queue_state[i]);
	dump->interrupts = cpu_to_le32(READ_ONCE(a5->interrupts));
	dump->watchdogs = cpu_to_le32(READ_ONCE(a5->watchdogs));
	dump->protocol_errors = cpu_to_le32(READ_ONCE(a5->protocol_errors));
	memcpy(dump->sfr, a5->sfr, sizeof(dump->sfr));
	/* Publish before power-down or IOMMU teardown can lose the evidence. */
	dev_coredumpv(a5->dev, dump, sizeof(*dump), GFP_KERNEL);
}

static int free_memory(struct camss_a5 *a5)
{
	struct camss_a5_fw_memory *memories[] = { &a5->heap, &a5->shared, &a5->firmware };
	unsigned int i;
	int ret;

	if (a5->cpu_enabled)
		return -EBUSY;
	capture_shared_state(a5);
	for (i = 0; i < ARRAY_SIZE(memories); i++) {
		ret = camss_a5_fw_memory_unmap(memories[i]);
		if (ret)
			return ret;
		ret = camss_a5_fw_memory_free(memories[i]);
		if (ret)
			return ret;
	}
	a5->table = NULL;
	return 0;
}

static int prepare_memory(struct camss_a5 *a5, const struct firmware *firmware)
{
	struct camss_a5_fw_layout layout;
	const u32 offsets[] = { CMD_OFFSET, MSG_OFFSET, DBG_OFFSET };
	const u32 sizes[] = { CMD_BYTES, MSG_BYTES, DBG_BYTES };
	u64 iovas[CAMSS_A5_QUEUE_COUNT];
	unsigned int i;
	int ret;

	ret = camss_a5_fw_inspect(firmware, CAMSS_A5_FW_WINDOW, &layout);
	if (ret)
		return ret;
	ret = camss_a5_fw_carveout_alloc(a5->dev, &a5->firmware,
					 a5->firmware_physical, CAMSS_A5_FW_WINDOW);
	if (ret)
		return ret;
	ret = camss_a5_fw_load(firmware, a5->firmware.cpu, CAMSS_A5_FW_WINDOW, &layout);
	if (ret)
		return ret;
	ret = camss_a5_iova_memory_alloc(a5->dev, &a5->shared, SHARED_IOVA, SHARED_BYTES);
	if (ret)
		return ret;
	ret = camss_a5_iova_memory_alloc(a5->dev, &a5->heap, HEAP_IOVA, HEAP_BYTES);
	if (ret)
		return ret;
	a5->table = a5->shared.cpu;
	for (i = 0; i < CAMSS_A5_QUEUE_COUNT; i++)
		iovas[i] = SHARED_IOVA + offsets[i];
	ret = camss_a5_qtable_init(a5->table, iovas, sizes);
	if (ret)
		return ret;
	for (i = 0; i < CAMSS_A5_QUEUE_COUNT; i++) {
		ret = camss_a5_ring_init(&a5->rings[i], &a5->table->queues[i],
					 a5->shared.cpu + offsets[i], sizes[i]);
		if (ret)
			return ret;
	}
	put_unaligned_le32(4096, a5->shared.cpu + SFR_OFFSET);
	ret = camss_a5_fw_memory_map(&a5->firmware);
	if (ret)
		return ret;
	ret = camss_a5_fw_memory_map(&a5->shared);
	if (ret)
		return ret;
	return camss_a5_fw_memory_map(&a5->heap);
}

static int quiesce_locked(struct camss_a5 *a5)
{
	void __iomem *base = a5->resources.sierra;
	u32 value;
	int ret;

	hrtimer_cancel(&a5->boot_timer);
	if (a5->cpu_enabled) {
		if (a5->initialized) {
			a5->pc_error = send_command(a5, SYS_PC_PREP, PC_DONE);
			if (!a5->pc_error)
				a5->pc_error = readl_poll_timeout(base + A5_STATUS, value,
							 value & A5_WFI, 1000, 300000);
		}
		writel(0, base + A5_INT_ENABLE);
		if (a5->irq_enabled) {
			disable_irq(a5->irq);
			a5->irq_enabled = false;
		}
		value = readl(base + A5_CONTROL) & ~(A5_ENABLE | A5_WAKE);
		writel(value, base + A5_CONTROL);
		value = readl(base + A5_RESET);
		writel(value, base + A5_RESET);
		writel(0, base + A5_INIT_REQUEST);
		/* The block reset is owned exclusively. Retain all RAM and the
		 * runtime-PM reference if it cannot quiesce a failed firmware.
		 */
		ret = reset_control_assert(a5->reset);
		if (ret)
			return ret;
		a5->reset_asserted = true;
		a5->cpu_enabled = false;
		a5->initialized = false;
		dev_info(a5->dev, "A5 stopped, block reset asserted; pc_error=%d\n", a5->pc_error);
	}
	if (a5->irq_enabled) {
		disable_irq(a5->irq);
		a5->irq_enabled = false;
	}
	capture_shared_state(a5);
	return 0;
}

static int stop_locked(struct camss_a5 *a5)
{
	int ret;

	ret = quiesce_locked(a5);
	if (ret)
		return ret;
	/* Retire all firmware DMA aliases while the supplier is still powered.
	 * A failed unmap keeps the PM reference and backing for a later retry.
	 */
	dev_info(a5->dev, "A5 retiring DMA aliases\n");
	ret = free_memory(a5);
	if (ret)
		return ret;
	dev_info(a5->dev, "A5 DMA aliases retired\n");
	if (a5->power_held) {
		ret = pm_runtime_put_sync_suspend(a5->dev);
		a5->power_held = false;
		dev_info(a5->dev, "A5 PM reference released: %d\n", ret);
		if (ret < 0)
			return ret;
	}
	return 0;
}

int camss_a5_stop(struct camss_a5 *a5)
{
	int ret;

	mutex_lock(&a5->lock);
	ret = stop_locked(a5);
	a5->cleanup_error = ret;
	mutex_unlock(&a5->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(camss_a5_stop);

int camss_a5_start(struct camss_a5 *a5, const struct firmware *firmware)
{
	void __iomem *base = a5->resources.sierra;
	u32 value;
	unsigned long flags;
	int ret;

	mutex_lock(&a5->lock);
	if (camss_a5_backing_owned(a5) || a5->power_held || a5->cpu_enabled) {
		ret = -EBUSY;
		goto unlock;
	}
	a5->init_done = a5->ping_done = a5->pc_done = false;
	a5->init_response = a5->fw_version = a5->last_type = 0;
	a5->interrupts = a5->messages = a5->debug_packets = 0;
	a5->watchdogs = a5->protocol_errors = 0;
	a5->pc_error = a5->cleanup_error = 0;
	a5->cpu_release_attempted = false;
	a5->boot_expired = false;
	ret = prepare_memory(a5, firmware);
	if (ret)
		goto fail;
	dev_info(a5->dev, "A5 firmware and shared aliases ready; firmware privileged\n");
	ret = pm_runtime_resume_and_get(a5->dev);
	if (ret < 0)
		goto fail;
	a5->power_held = true;
	dev_info(a5->dev, "A5/BPS/IPE resources powered\n");
	ret = reset_control_assert(a5->reset);
	if (ret)
		goto fail;
	a5->reset_asserted = true;
	udelay(2);
	ret = reset_control_deassert(a5->reset);
	if (ret)
		goto fail;
	a5->reset_asserted = false;
	writel(0, base + A5_INT_ENABLE);
	writel(7, base + A5_INT_CLEAR);
	writel(0, base + A5_INIT_REQUEST);
	writel(0, base + A5_INIT_RESPONSE);
	a5->response_before = readl(base + A5_INIT_RESPONSE);
	writel(SHARED_IOVA, base + A5_QTABLE_PTR);
	writel(SHARED_IOVA + SFR_OFFSET, base + A5_SFR_PTR);
	writel(SHARED_IOVA, base + A5_SHARED_PTR);
	writel(SHARED_WINDOW, base + A5_SHARED_SIZE);
	writel(HEAP_IOVA, base + A5_HEAP_PTR);
	writel(HEAP_BYTES, base + A5_HEAP_SIZE);
	/* Trace transport is never enabled by this bootstrap. */
	writel(0, base + A5_QDSS_PTR);
	writel(0, base + A5_QDSS_SIZE);
	writel(IO_IOVA, base + A5_IO_PTR);
	writel(IO_BYTES, base + A5_IO_SIZE);
	dma_wmb();
	enable_irq(a5->irq);
	a5->irq_enabled = true;
	dev_info(a5->dev, "A5 CPU release, HFI request prepared\n");
	hrtimer_start(&a5->boot_timer, ms_to_ktime(500), HRTIMER_MODE_REL);
	spin_lock_irqsave(&a5->control_lock, flags);
	a5->cpu_enabled = true;
	a5->cpu_release_attempted = true;
	writel(A5_ENABLE, base + A5_CONTROL);
	writel(0x10, base + A5_RESET);
	writel(A5_ENABLE | A5_WAKE | A5_WFI_GATE, base + A5_CONTROL);
	writel(1, base + A5_INIT_REQUEST);
	spin_unlock_irqrestore(&a5->control_lock, flags);
	ret = readl_poll_timeout(base + A5_INIT_RESPONSE, value,
				 value == 1 || value == 2 || value == 3 ||
				 READ_ONCE(a5->boot_expired),
				 100, 500000);
	hrtimer_cancel(&a5->boot_timer);
	a5->init_response = value;
	a5->fw_version = readl(base + A5_FW_VERSION);
	dev_info(a5->dev, "A5 HFI response=%u firmware=%x expired=%u\n",
		 value, a5->fw_version, a5->boot_expired);
	if (a5->boot_expired)
		ret = -ETIMEDOUT;
	if (ret)
		goto fail;
	/* The retained entry reports 1/2 for ready/initialization failure.
	 * SendSfrMessage publishes 3 after its error record, so it is terminal
	 * failure too. Never send SYS_INIT after a firmware SFR error.
	 */
	if (value != 1) {
		ret = -EREMOTEIO;
		goto fail;
	}
	writel(3, base + A5_INT_ENABLE);
	ret = send_command(a5, SYS_INIT, INIT_DONE);
	if (ret)
		goto fail;
	a5->initialized = true;
	dev_info(a5->dev, "A5 SYS_INIT acknowledged\n");
	goto unlock;
fail:
	publish_start_failure(a5);
	/* Keep owned RAM, aliases and the PM reference until the owner calls
	 * stop. A failed boot can then be inspected after CPU/IRQ quiescence,
	 * before IOMMU retirement or supplier power-down loses the evidence.
	 */
	a5->cleanup_error = quiesce_locked(a5);
unlock:
	mutex_unlock(&a5->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(camss_a5_start);

int camss_a5_ping(struct camss_a5 *a5, u64 cookie)
{
	int ret;

	mutex_lock(&a5->lock);
	if (!a5->initialized) {
		ret = -ENODEV;
		goto done;
	}
	a5->ping_cookie = cookie;
	ret = send_command(a5, SYS_PING, PING_ACK);
done:
	mutex_unlock(&a5->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(camss_a5_ping);

int camss_a5_runtime_resume(struct camss_a5 *a5)
{
	int ret;

	/* ICP BCR also holds its AHB branch off. Release the block reset
	 * before the clock framework waits for that branch to report running.
	 * A5 itself remains disabled until the explicit CPU release sequence.
	 */
	ret = reset_control_deassert(a5->reset);
	if (ret)
		return ret;
	a5->reset_asserted = false;
	return camss_a5_resources_resume(&a5->resources);
}
EXPORT_SYMBOL_GPL(camss_a5_runtime_resume);

int camss_a5_runtime_suspend(struct camss_a5 *a5)
{
	int ret;

	if (a5->cpu_enabled)
		return -EBUSY;
	ret = reset_control_assert(a5->reset);
	if (ret)
		return ret;
	a5->reset_asserted = true;
	return camss_a5_resources_suspend(&a5->resources);
}
EXPORT_SYMBOL_GPL(camss_a5_runtime_suspend);

static int firmware_region_get(struct device *dev, struct resource *firmware)
{
	struct device_node *node;
	int index, ret;

	/* Firmware RAM is a dedicated board reservation, not an MMIO range
	 * inside any larger no-map pool. In particular, a generic SoC address
	 * can fall inside this board's protected-memory reservation.
	 */
	index = of_property_match_string(dev->of_node, "memory-region-names", "firmware");
	if (index < 0)
		return index;
	node = of_parse_phandle(dev->of_node, "memory-region", index);
	if (!node)
		return -EINVAL;
	ret = -EINVAL;
	if (!of_device_is_available(node) || !of_property_present(node, "no-map"))
		goto put;
	ret = of_address_to_resource(node, 0, firmware);
	if (ret)
		goto put;
	if (!firmware->start || !IS_ALIGNED(firmware->start, PAGE_SIZE) ||
	    firmware->end < firmware->start ||
	    resource_size(firmware) != CAMSS_A5_FW_WINDOW) {
		ret = -EINVAL;
		goto put;
	}
put:
	of_node_put(node);
	return ret;
}

int camss_a5_init(struct platform_device *pdev, struct camss_a5 *a5)
{
	struct device *dev = &pdev->dev;
	struct resource firmware;
	int ret;

	mutex_init(&a5->lock);
	spin_lock_init(&a5->control_lock);
	hrtimer_setup(&a5->boot_timer, boot_timeout, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	init_completion(&a5->response);
	a5->dev = dev;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	ret = camss_a5_resources_init(pdev, &a5->resources);
	if (ret)
		return ret;
	a5->reset = devm_reset_control_get_exclusive(dev, "icp");
	if (IS_ERR(a5->reset))
		return PTR_ERR(a5->reset);
	if (!a5->reset)
		return -ENODEV;
	ret = firmware_region_get(dev, &firmware);
	if (ret)
		return ret;
	if (!devm_request_mem_region(dev, firmware.start, resource_size(&firmware), dev_name(dev)))
		return -EBUSY;
	a5->firmware_physical = firmware.start;
	a5->message = devm_kzalloc(dev, MSG_BYTES, GFP_KERNEL);
	a5->debug = devm_kzalloc(dev, DBG_BYTES, GFP_KERNEL);
	if (!a5->message || !a5->debug)
		return -ENOMEM;
	a5->irq = platform_get_irq(pdev, 0);
	if (a5->irq < 0)
		return a5->irq;
	return devm_request_threaded_irq(dev, a5->irq, a5_interrupt,
					a5_interrupt_thread,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_AUTOEN,
					dev_name(dev), a5);
}
EXPORT_SYMBOL_GPL(camss_a5_init);
