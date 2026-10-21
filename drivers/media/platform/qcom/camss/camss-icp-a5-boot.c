// SPDX-License-Identifier: GPL-2.0-only
/* SM8150 A5 firmware transport and IPE session lifecycle. */
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/devcoredump.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#include "camss-icp-a5-boot.h"
#include "camss-icp-a5-ipe.h"

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
#define IPE_CREATE 0x1010008U
#define IPE_CREATE_ACK 0x1020008U
#define IPE_ASYNC_DIRECT 0x101000aU
#define IPE_ASYNC_DIRECT_ACK 0x102000aU
#define IPE_DESTROY 8U
#define IPE_CONFIG_IO 5U
#define IPE_FRAME_PROCESS 6U
#define IPE_MEM_MAP 14U
#define IPE_MEM_UNMAP 15U
#define IPE_ASYNC_INDIRECT 0x101000eU
#define IPE_ASYNC_INDIRECT_ACK 0x102000eU
#define IPE_IO_CONFIG_BYTES 1216U
/* The 100-KiB debug queue is mapped by firmware as a full 1-MiB
 * region starting at SHARED_IOVA + DBG_OFFSET. Keep commands beyond it.
 */
#define CONFIG_IOVA (SHARED_IOVA + 0x200000U)
#define CONFIG_BYTES 4096U
#define REQUEST_IOVA 0x07800000U
#define REQUEST_MAX 0x100000U
#define IMAGE_INPUT_IOVA 0x20000000U
#define IMAGE_OUTPUT_IOVA 0x24000000U
#define IMAGE_WINDOW 0x04000000U

static bool has_ipe_sid_group(struct device *dev);

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

	if (bytes < 8 || get_unaligned_le32(a5->message) != bytes)
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
	case IPE_CREATE_ACK:
		if (bytes != 32)
			goto malformed;
		if (get_unaligned_le64(a5->message + 16) != a5->session_cookie ||
		    get_unaligned_le64(a5->message + 24) != a5->session_cookie2)
			return;
		error = get_unaligned_le32(a5->message + 8);
		a5->session_reply_handle = get_unaligned_le32(a5->message + 12);
		if (!error && !a5->session_reply_handle)
			goto malformed;
		a5->session_reply = true;
		break;
	case IPE_ASYNC_DIRECT_ACK:
	case IPE_ASYNC_INDIRECT_ACK:
		/* Sizes come from the actual A5 dispatch/callbacks, which differ
		 * from the old downstream packed payload declarations.
		 */
		if (a5->session_opcode != IPE_DESTROY &&
		    a5->session_opcode != IPE_MEM_MAP &&
		    a5->session_opcode != IPE_MEM_UNMAP &&
		    a5->session_opcode != IPE_CONFIG_IO &&
		    a5->session_opcode != IPE_FRAME_PROCESS)
			goto malformed;
		if ((a5->session_opcode == IPE_DESTROY && bytes != 48) ||
		    ((a5->session_opcode == IPE_MEM_MAP ||
		      a5->session_opcode == IPE_MEM_UNMAP) && bytes != 44) ||
		    (a5->session_opcode == IPE_CONFIG_IO && bytes != 56) ||
		    (a5->session_opcode == IPE_FRAME_PROCESS && bytes != 72))
			goto malformed;
		if (get_unaligned_le64(a5->message + 12) != a5->session_cookie ||
		    get_unaligned_le64(a5->message + 20) != a5->session_cookie2)
			return;
		if (get_unaligned_le32(a5->message + 8) != a5->session_opcode)
			goto malformed;
		a5->session_header_error = get_unaligned_le32(a5->message + 28);
		/* MAP/UNMAP callbacks only initialize the ACK header. Their
		 * 12-byte tail retains allocator contents, not a status word.
		 */
		a5->session_payload_status =
			(a5->session_opcode == IPE_CONFIG_IO ||
			 a5->session_opcode == IPE_DESTROY ||
			 a5->session_opcode == IPE_FRAME_PROCESS) ?
			get_unaligned_le32(a5->message + 32) : 0;
		error = a5->session_header_error ?: a5->session_payload_status;
		if (!error && a5->session_opcode == IPE_CONFIG_IO)
			a5->session_scratch_bytes = get_unaligned_le32(a5->message + 36);
		a5->session_reply = true;
		break;
	case PING_ACK:
		if (bytes != 16 || get_unaligned_le64(a5->message + 8) != a5->ping_cookie)
			goto malformed;
		a5->ping_done = true;
		break;
	default:
		goto malformed;
	}
	/* A successful packet must not erase a concurrent watchdog/protocol
	 * failure latched by the hard IRQ or an earlier packet in this batch.
	 */
	if (error)
		command_failure(a5, -EREMOTEIO);
	else
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

static int send_packet(struct camss_a5 *a5, const void *packet, u32 bytes,
			 u32 response)
{
	int ret;

	a5->command_queued = false;
	if (!a5->cpu_enabled || !a5->irq_enabled || a5->watchdogs)
		return -EIO;
	reinit_completion(&a5->response);
	WRITE_ONCE(a5->command_error, 0);
	WRITE_ONCE(a5->expected, response);
	ret = camss_a5_ring_write(&a5->rings[CAMSS_A5_CMD], packet, bytes);
	if (ret)
		goto done;
	a5->command_queued = true;
	writel(1, a5->resources.sierra + A5_HOST_INTERRUPT);
	if (!wait_for_completion_timeout(&a5->response, msecs_to_jiffies(5000)))
		ret = -ETIMEDOUT;
	else
		ret = READ_ONCE(a5->command_error);
done:
	WRITE_ONCE(a5->expected, 0);
	/* Join an IRQ thread already parsing a response before inspecting its
	 * fields or reusing the pending token/handle state.
	 */
	synchronize_irq(a5->irq);
	if (!ret)
		ret = READ_ONCE(a5->command_error);
	return ret;
}

static int send_command(struct camss_a5 *a5, u32 type, u32 response)
{
	__le32 packet[4];
	u32 bytes = type == SYS_PING ? 16 : 8;

	packet[0] = cpu_to_le32(bytes);
	packet[1] = cpu_to_le32(type);
	put_unaligned_le64(a5->ping_cookie, packet + 2);
	return send_packet(a5, packet, bytes, response);
}

/* Caller holds the lifecycle/command mutex throughout a session command. */
static int session_token(struct camss_a5 *a5)
{
	if (a5->session_sequence == U64_MAX)
		return -EOVERFLOW;
	a5->session_cookie = ++a5->session_sequence;
	a5->session_cookie2 = a5->session_cookie ^ 0x3764955045413541ULL;
	a5->session_reply = false;
	a5->session_reply_handle = 0;
	a5->session_scratch_bytes = 0;
	a5->session_header_error = a5->session_payload_status = 0;
	return 0;
}

static void session_command_failed(struct camss_a5 *a5, int ret)
{
	/* Once enqueued, timeout or an unrecognizable response can leave an
	 * unknown live firmware handle. Do not issue another session command.
	 */
	if (a5->command_queued && (ret != -EREMOTEIO || !a5->session_reply))
		a5->session_uncertain = true;
}

/* A5/Titan17x IO wire layout verified against the retained OEM HAL and ELF.
 * No tuning tables or image buffers are embedded in this serializer.
 */
static bool nv12_layout_valid(const struct camss_a5_nv12_layout *layout)
{
	return layout->width && layout->height &&
	       !(layout->width & 1) && !(layout->height & 1) &&
	       layout->stride >= layout->width && !(layout->stride & 1) &&
	       layout->slice_height >= layout->height &&
	       !(layout->slice_height & 1) &&
	       (u64)layout->stride * layout->slice_height <= (u64)U32_MAX * 2 / 3;
}

static void encode_nv12_image(void *destination,
			      const struct camss_a5_nv12_layout *layout)
{
	put_unaligned_le32(20, destination); /* IMAGE_FORMAT_LINEAR_NV12 */
	put_unaligned_le32(layout->width, destination + 4);
	put_unaligned_le32(layout->height, destination + 8);
	put_unaligned_le32(layout->uv_swap, destination + 24);
	put_unaligned_le32(layout->stride, destination + 52);
	put_unaligned_le32(layout->slice_height, destination + 56);
	put_unaligned_le32(layout->stride, destination + 60);
	put_unaligned_le32(layout->slice_height / 2, destination + 64);
}

static int encode_ipe_io(void *destination, u32 bytes,
			 const struct camss_a5_ipe_io *io)
{
	if (!destination || !io || bytes < IPE_IO_CONFIG_BYTES ||
	    !nv12_layout_valid(&io->input) || !nv12_layout_valid(&io->output) ||
	    (io->output_port != 8 && io->output_port != 9))
		return -EINVAL;
	memset(destination, 0, IPE_IO_CONFIG_BYTES);
	encode_nv12_image(destination, &io->input);
	encode_nv12_image(destination + io->output_port * 84, &io->output);
	/* One frame per batch, ordinary non-secure topology/mux defaults.
	 * The aligned userArg at byte1208 remains zero.
	 */
	put_unaligned_le32(1, destination + 1192);
	return 0;
}

static int region_packet_locked(struct camss_a5 *a5,
				struct camss_a5_session *session,
				struct camss_a5_fw_memory *memory, bool map)
{
	u8 packet[56] = {};
	int ret;

	a5->command_queued = false;
	if (session->a5 != a5 || !a5->initialized || a5->session_uncertain)
		return -EIO;
	if (!memory || !memory->cpu || !memory->bytes ||
	    memory->mapped != memory->bytes)
		return -EINVAL;
	ret = session_token(a5);
	if (ret)
		return ret;
	a5->session_opcode = map ? IPE_MEM_MAP : IPE_MEM_UNMAP;
	put_unaligned_le32(sizeof(packet), packet);
	put_unaligned_le32(IPE_ASYNC_DIRECT, packet + 4);
	put_unaligned_le32(a5->session_opcode, packet + 8);
	put_unaligned_le64(a5->session_cookie, packet + 12);
	put_unaligned_le64(a5->session_cookie2, packet + 20);
	put_unaligned_le32(1, packet + 28);
	put_unaligned_le32(session->handle, packet + 32);
	put_unaligned_le64(a5->session_cookie, packet + 36);
	put_unaligned_le32(1, packet + 44);
	put_unaligned_le32(memory->iova, packet + 48);
	put_unaligned_le32(memory->bytes, packet + 52);
	ret = send_packet(a5, packet, sizeof(packet), IPE_ASYNC_DIRECT_ACK);
	return ret;
}

static int config_region_locked(struct camss_a5 *a5,
				struct camss_a5_session *session, bool map)
{
	int ret;

	if (session->a5 != a5 || !a5->initialized || a5->session_uncertain)
		return -EIO;
	if ((map && a5->config_fw_mapped) ||
	    (!map && (!a5->config_fw_mapped || a5->config_owner != session)))
		return -EBUSY;
	ret = region_packet_locked(a5, session, &a5->config, map);
	if (map && a5->command_queued) {
		/* A failed reply can follow a partially applied firmware mapping.
		 * Keep its backing until an acknowledged unmap or CPU quiescence.
		 */
		a5->config_fw_mapped = true;
		a5->config_owner = session;
	}
	if (ret) {
		session_command_failed(a5, ret);
		if (map && a5->command_queued)
			a5->session_uncertain = true;
		dev_info(a5->dev, "A5 IPE region opcode=%u failed ret=%d hfi=%u status=%u queued=%u reply=%u uncertain=%u\n",
			 a5->session_opcode, ret, a5->session_header_error,
			 a5->session_payload_status, a5->command_queued,
			 a5->session_reply, a5->session_uncertain);
		return ret;
	}
	if (map) {
		a5->config_maps++;
	} else {
		a5->config_fw_mapped = false;
		a5->config_owner = NULL;
		a5->config_unmaps++;
	}
	dev_info(a5->dev, "A5 IPE config region %s acknowledged\n", map ? "map" : "unmap");
	return 0;
}

int camss_a5_ipe_configure_nv12(struct camss_a5 *a5,
			       struct camss_a5_session *session,
			       const struct camss_a5_ipe_io *io)
{
	u8 packet[40] = {};
	int ret, unmap;

	if (!session || !io)
		return -EINVAL;
	mutex_lock(&a5->lock);
	if (session->a5 != a5 || !session->handle || !a5->initialized ||
	    !a5->cpu_enabled) {
		ret = -ENODEV;
		goto done;
	}
	if (a5->session_uncertain || a5->config_fw_mapped) {
		ret = -EBUSY;
		goto done;
	}
	if (!a5->config.cpu || a5->config.bytes < IPE_IO_CONFIG_BYTES ||
	    a5->config.mapped != a5->config.bytes) {
		ret = -EINVAL;
		goto done;
	}
	ret = encode_ipe_io(a5->config.cpu, a5->config.bytes, io);
	if (ret)
		goto done;
	dma_wmb();
	ret = config_region_locked(a5, session, true);
	if (ret)
		goto done;
	ret = session_token(a5);
	if (ret)
		goto unmap;
	a5->session_opcode = IPE_CONFIG_IO;
	put_unaligned_le32(sizeof(packet), packet);
	put_unaligned_le32(IPE_ASYNC_INDIRECT, packet + 4);
	put_unaligned_le32(IPE_CONFIG_IO, packet + 8);
	put_unaligned_le64(a5->session_cookie, packet + 12);
	put_unaligned_le64(a5->session_cookie2, packet + 20);
	put_unaligned_le32(1, packet + 28);
	put_unaligned_le32(session->handle, packet + 32);
	put_unaligned_le32(a5->config.iova, packet + 36);
	session->configured = false;
	session->scratch_bytes = 0;
	ret = send_packet(a5, packet, sizeof(packet), IPE_ASYNC_INDIRECT_ACK);
	if (ret) {
		session_command_failed(a5, ret);
		dev_info(a5->dev, "A5 IPE CONFIG_IO failed ret=%d hfi=%u status=%u\n",
			 ret, a5->session_header_error, a5->session_payload_status);
	} else {
		session->scratch_bytes = a5->session_scratch_bytes;
		a5->io_configs++;
		dev_info(a5->dev, "A5 IPE CONFIG_IO acknowledged scratch=%u\n",
			 session->scratch_bytes);
	}
unmap:
	if (!a5->session_uncertain) {
		unmap = config_region_locked(a5, session, false);
		if (!ret)
			ret = unmap;
	}
	if (!ret) {
		session->io = *io;
		session->configured = true;
	}
done:
	mutex_unlock(&a5->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_configure_nv12);

int camss_a5_ipe_create(struct camss_a5 *a5, struct camss_a5_session *session,
			 u32 handle_type)
{
	struct camss_a5_session *existing;
	u8 packet[28] = {};
	int ret;

	if (!session || (handle_type != CAMSS_A5_IPE_RT &&
			 handle_type != CAMSS_A5_IPE_NON_RT))
		return -EINVAL;
	mutex_lock(&a5->lock);
	if (session->a5) {
		ret = -EBUSY;
		goto done;
	}
	if (!a5->initialized || !a5->cpu_enabled) {
		ret = -ENODEV;
		goto done;
	}
	if (a5->session_uncertain) {
		ret = -EIO;
		goto done;
	}
	ret = session_token(a5);
	if (ret)
		goto done;
	put_unaligned_le32(sizeof(packet), packet);
	put_unaligned_le32(IPE_CREATE, packet + 4);
	put_unaligned_le32(handle_type, packet + 8);
	put_unaligned_le64(a5->session_cookie, packet + 12);
	put_unaligned_le64(a5->session_cookie2, packet + 20);
	ret = send_packet(a5, packet, sizeof(packet), IPE_CREATE_ACK);
	if (ret) {
		session_command_failed(a5, ret);
		goto done;
	}
	list_for_each_entry(existing, &a5->sessions, node) {
		if (existing->handle == a5->session_reply_handle) {
			a5->session_uncertain = true;
			ret = -EPROTO;
			goto done;
		}
	}
	session->a5 = a5;
	session->type = handle_type;
	session->handle = a5->session_reply_handle;
	list_add_tail(&session->node, &a5->sessions);
	a5->session_count++;
	a5->session_creates++;
	a5->session_peak = max(a5->session_peak, a5->session_count);
	dev_info(a5->dev, "A5 IPE create acknowledged type=%u active=%u\n",
		 handle_type, a5->session_count);
done:
	mutex_unlock(&a5->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_create);

static int ipe_destroy_locked(struct camss_a5 *a5,
			      struct camss_a5_session *session)
{
	u8 packet[44] = {};
	int ret;

	if (session->a5 != a5 || !session->handle)
		return -ENOENT;
	if (!a5->initialized || !a5->cpu_enabled)
		return -ENODEV;
	if (a5->session_uncertain)
		return -EIO;
	if (a5->request_fw_mapped && a5->request_owner == session)
		return -EBUSY;
	if (a5->config_fw_mapped && a5->config_owner == session) {
		ret = config_region_locked(a5, session, false);
		if (ret)
			return ret;
	}
	ret = session_token(a5);
	if (ret)
		return ret;
	a5->session_opcode = IPE_DESTROY;
	put_unaligned_le32(sizeof(packet), packet);
	put_unaligned_le32(IPE_ASYNC_DIRECT, packet + 4);
	put_unaligned_le32(IPE_DESTROY, packet + 8);
	put_unaligned_le64(a5->session_cookie, packet + 12);
	put_unaligned_le64(a5->session_cookie2, packet + 20);
	put_unaligned_le32(1, packet + 28);
	put_unaligned_le32(session->handle, packet + 32);
	/* This ABI's destroy payload has no independent echoed handle. The
	 * two unique header tokens bind this reply to the submitted handle.
	 */
	put_unaligned_le64(a5->session_cookie, packet + 36);
	ret = send_packet(a5, packet, sizeof(packet), IPE_ASYNC_DIRECT_ACK);
	if (ret) {
		session_command_failed(a5, ret);
		return ret;
	}
	list_del_init(&session->node);
	session->a5 = NULL;
	session->handle = 0;
	session->type = 0;
	session->configured = false;
	session->scratch_bytes = 0;
	a5->session_count--;
	a5->session_destroys++;
	dev_info(a5->dev, "A5 IPE destroy acknowledged active=%u\n", a5->session_count);
	return 0;
}

int camss_a5_ipe_destroy(struct camss_a5 *a5, struct camss_a5_session *session)
{
	int ret;

	if (!session)
		return -EINVAL;
	mutex_lock(&a5->lock);
	ret = ipe_destroy_locked(a5, session);
	mutex_unlock(&a5->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_destroy);

static void invalidate_sessions(struct camss_a5 *a5)
{
	struct camss_a5_session *session, *next;

	list_for_each_entry_safe(session, next, &a5->sessions, node) {
		list_del_init(&session->node);
		session->a5 = NULL;
		session->handle = 0;
		session->type = 0;
		session->configured = false;
		session->scratch_bytes = 0;
	}
	a5->session_count = 0;
	a5->session_uncertain = false;
	a5->config_fw_mapped = false;
	a5->config_owner = NULL;
	a5->request_fw_mapped = false;
	a5->request_owner = NULL;
}

bool camss_a5_backing_owned(const struct camss_a5 *a5)
{
	return a5->firmware.cpu || a5->shared.cpu || a5->heap.cpu || a5->config.cpu ||
	       a5->request.cpu || a5->image_input.cpu || a5->image_output.cpu ||
	       a5->dma_input.buffer || a5->dma_output.buffer;
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

static int free_imported_images(struct camss_a5 *a5)
{
 int ret;

 if (a5->ipe_dma_pending || a5->request_fw_mapped)
  return -EBUSY;
 ret = camss_a5_dma_image_release(&a5->dma_output);
 return ret ?: camss_a5_dma_image_release(&a5->dma_input);
}

static int free_memory(struct camss_a5 *a5)
{
	struct camss_a5_fw_memory *memories[] = {
		&a5->image_output, &a5->image_input, &a5->request,
		&a5->config, &a5->heap, &a5->shared, &a5->firmware,
	};
	unsigned int i;
	int ret;

	if (a5->cpu_enabled || a5->ipe_dma_pending)
		return -EBUSY;
	ret = free_imported_images(a5);
	if (ret)
		return ret;
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
	ret = camss_a5_iova_memory_alloc(a5->dev, &a5->config, CONFIG_IOVA, CONFIG_BYTES);
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
	ret = camss_a5_fw_memory_map(&a5->heap);
	if (ret)
		return ret;
	return camss_a5_fw_memory_map(&a5->config);
}

/* A forced reset must not leave the ICP/AHB bridge held in reset while
 * retiring its DMA aliases. Reuse the cold-start reset pulse, then prove
 * that releasing reset did not release the A5 CPU. A failed release/readback
 * retains ownership and is retried before any CSR access on the next stop.
 */
static int deassert_stopped_reset_locked(struct camss_a5 *a5)
{
	u32 control;
	int ret;

	ret = reset_control_deassert(a5->reset);
	if (ret)
		return ret;
	a5->reset_asserted = false;
	control = readl(a5->resources.sierra + A5_CONTROL);
	if (control & (A5_ENABLE | A5_WAKE)) {
		/* Treat an observed enabled CPU as live even after failed boot. */
		a5->cpu_enabled = true;
		a5->initialized = false;
		return -EIO;
	}
	return 0;
}

static int quiesce_locked(struct camss_a5 *a5)
{
	void __iomem *base = a5->resources.sierra;
	bool cooperative = false;
	u32 value;
	int ret;

	hrtimer_cancel(&a5->boot_timer);
	if (a5->reset_asserted) {
		ret = deassert_stopped_reset_locked(a5);
		if (ret)
			return ret;
		a5->cpu_enabled = false;
		a5->initialized = false;
	}
	if (a5->cpu_enabled) {
		if (a5->initialized && a5->irq_enabled &&
		    !a5->session_uncertain && !a5->session_count) {
			a5->pc_error = send_command(a5, SYS_PC_PREP, PC_DONE);
			if (!a5->pc_error)
				dev_info(a5->dev, "A5 SYS_PC_PREP acknowledged\n");
			if (!a5->pc_error)
				a5->pc_error = readl_poll_timeout(base + A5_STATUS, value,
							 value & A5_WFI, 1000, 300000);
			cooperative = !a5->pc_error;
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
		/* Confirm the CPU enable and wake requests are actually cleared.
		 * Normal firmware shutdown follows PC_PREP/WFI and the local CSR
		 * sequence. Keep the global ICP/AHB block accessible until its DMA
		 * aliases and clock/domain ownership have been retired.
		 */
		value = readl(base + A5_CONTROL);
		if (value & (A5_ENABLE | A5_WAKE)) {
			cooperative = false;
			a5->pc_error = -EIO;
		}
		if (!cooperative) {
			/* Keep the forced reset guarantee for unknown completion,
			 * but reopen the bridge before IPE quiesce or DMA unmap.
			 * Every failed step keeps RAM/PM and any pending image DMA.
			 */
			ret = reset_control_assert(a5->reset);
			if (ret)
				return ret;
			a5->reset_asserted = true;
			a5->initialized = false;
			udelay(2); /* same verified pulse as the cold-start path */
			ret = deassert_stopped_reset_locked(a5);
			if (ret)
				return ret;
			dev_info(a5->dev, "A5 forced reset pulse complete; CPU enable/wake clear\n");
		}
		a5->cpu_enabled = false;
		a5->initialized = false;
		dev_info(a5->dev, "A5 stopped, cooperative=%u block_reset=%u pc_error=%d\n",
			 cooperative, a5->reset_asserted, a5->pc_error);
	}
	if (a5->irq_enabled) {
		disable_irq(a5->irq);
		a5->irq_enabled = false;
	}
	if (a5->ipe_dma_pending) {
		ret = camss_a5_ipe_quiesce(&a5->resources);
		if (ret)
			return ret;
		a5->ipe_dma_pending = false;
	}
	invalidate_sessions(a5);
	capture_shared_state(a5);
	return 0;
}

static int stop_locked(struct camss_a5 *a5)
{
	struct camss_a5_session *session, *next;
	int ret;

	if (a5->initialized && !a5->session_uncertain) {
		list_for_each_entry_safe(session, next, &a5->sessions, node) {
			ret = ipe_destroy_locked(a5, session);
			if (ret)
				break;
		}
	}
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

static int program_hfi_parameters(struct camss_a5 *a5)
{
	void __iomem *base = a5->resources.sierra;
	static const u32 fields[][2] = {
		{ A5_QTABLE_PTR, SHARED_IOVA },
		{ A5_SFR_PTR, SHARED_IOVA + SFR_OFFSET },
		{ A5_SHARED_PTR, SHARED_IOVA },
		{ A5_SHARED_SIZE, SHARED_WINDOW },
		{ A5_HEAP_PTR, HEAP_IOVA },
		{ A5_HEAP_SIZE, HEAP_BYTES },
		{ A5_QDSS_PTR, 0 },
		{ A5_QDSS_SIZE, 0 },
		{ A5_IO_PTR, IO_IOVA },
		{ A5_IO_SIZE, IO_BYTES },
	};
	unsigned int i;
	u32 value;

	for (i = 0; i < ARRAY_SIZE(fields); i++)
		writel(fields[i][1], base + fields[i][0]);
	for (i = 0; i < ARRAY_SIZE(fields); i++) {
		value = readl(base + fields[i][0]);
		if (value != fields[i][1]) {
			dev_err(a5->dev, "A5 HFI register %x readback %x expected %x\n",
				fields[i][0], value, fields[i][1]);
			return -EIO;
		}
	}
	return 0;
}

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
	a5->session_creates = a5->session_destroys = a5->session_peak = 0;
	a5->io_configs = a5->config_maps = a5->config_unmaps = 0;
	a5->frames_submitted = a5->frames_completed = 0;
	a5->direct_frames = a5->copy_frames = 0;
	a5->transfer_maps = a5->transfer_unmaps = 0;
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
	if (a5->engine_dma_capable) {
		ret = camss_a5_ipe_quiesce(&a5->resources);
		if (ret)
			goto fail;
		dev_info(a5->dev, "A5 IPE reset ACKs verified before CPU release\n");
	}
	writel(0, base + A5_INT_ENABLE);
	writel(7, base + A5_INT_CLEAR);
	writel(0, base + A5_INIT_REQUEST);
	writel(0, base + A5_INIT_RESPONSE);
	a5->response_before = readl(base + A5_INIT_RESPONSE);

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
	/* Match the OEM cold-boot sequence: CPU reset precedes HFI parameter
	 * programming. Never request initialization with unverified pointers.
	 */
	writel(A5_ENABLE | A5_WAKE | A5_WFI_GATE, base + A5_CONTROL);
	ret = program_hfi_parameters(a5);
	if (!ret)
		writel(1, base + A5_INIT_REQUEST);
	spin_unlock_irqrestore(&a5->control_lock, flags);
	if (ret) {
		hrtimer_cancel(&a5->boot_timer);
		goto fail;
	}
	dev_info(a5->dev, "A5 HFI parameters verified after CPU reset\n");
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
	/* An optional owner observer runs with RAM/PM still owned, before any
	 * CPU/reset teardown. It must not re-enter lifecycle operations.
	 */
	if (a5->cpu_release_attempted && a5->failure_snapshot)
		a5->failure_snapshot(a5);
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
	if (!ret)
		dev_info(a5->dev, "A5 SYS_PING acknowledged\n");
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
	if (a5->cpu_enabled || a5->ipe_dma_pending || camss_a5_backing_owned(a5))
		return -EBUSY;
	/* A cooperative stop has already disabled A5 through its local CSR.
	 * Do not assert the global ICP/AHB reset in the normal PM path.
	 * A forced stop has pulsed reset and verified the CPU is disabled.
	 * Keep the bridge accessible through DMA retirement and power-down.
	 */
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
	INIT_LIST_HEAD(&a5->sessions);
	spin_lock_init(&a5->control_lock);
	hrtimer_setup(&a5->boot_timer, boot_timeout, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	init_completion(&a5->response);
	a5->dev = dev;
	a5->engine_dma_capable = has_ipe_sid_group(dev);
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


static bool has_ipe_sid_group(struct device *dev)
{
	static const u32 specs[][2] = {
		{ 0x2222, 0 }, { 0x2080, 0x20 }, { 0x20a0, 0x20 },
		{ 0x2100, 0x20 }, { 0x2120, 0x20 }, { 0x20c0, 0 }, { 0x2140, 0 },
	};
	struct device_node *provider = NULL;
	struct of_phandle_args args;
	unsigned int i;
	bool valid = false;

	if (of_count_phandle_with_args(dev->of_node, "iommus", "#iommu-cells") != (int)ARRAY_SIZE(specs))
		return false;
	for (i = 0; i < ARRAY_SIZE(specs); i++) {
		if (of_parse_phandle_with_args(dev->of_node, "iommus", "#iommu-cells", i, &args)) {
			valid = false;
			goto out;
		}
		if (!provider)
			provider = of_node_get(args.np);
		valid = args.np == provider && args.args_count == 2 &&
			args.args[0] == specs[i][0] && args.args[1] == specs[i][1];
		of_node_put(args.np);
		if (!valid)
			goto out;
	}
out:
	of_node_put(provider);
	return valid;
}

// SPDX-License-Identifier: GPL-2.0-only
static int free_transfer_memory(struct camss_a5 *a5)
{
	struct camss_a5_fw_memory *memories[] = {
		&a5->image_output, &a5->image_input, &a5->request,
	};
	unsigned int i;
	int ret;

	if (a5->ipe_dma_pending || a5->request_fw_mapped)
		return -EBUSY;
	ret = free_imported_images(a5);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(memories); i++) {
		ret = camss_a5_fw_memory_unmap(memories[i]);
		if (ret)
			return ret;
		ret = camss_a5_fw_memory_free(memories[i]);
		if (ret)
			return ret;
	}
	return 0;
}

static int transfer_region_locked(struct camss_a5 *a5,
				  struct camss_a5_session *session, bool map)
{
	int ret;

	if ((map && a5->request_fw_mapped) ||
	    (!map && (!a5->request_fw_mapped || a5->request_owner != session)))
		return -EBUSY;
	ret = region_packet_locked(a5, session, &a5->request, map);
	if (map && a5->command_queued) {
		a5->request_fw_mapped = true;
		a5->request_owner = session;
	}
	if (ret) {
		session_command_failed(a5, ret);
		if (map && a5->command_queued)
			a5->session_uncertain = true;
		return ret;
	}
	if (map) {
		a5->transfer_maps++;
	} else {
		a5->request_fw_mapped = false;
		a5->request_owner = NULL;
		a5->transfer_unmaps++;
	}
	return 0;
}

static int transfer_alloc(struct camss_a5 *a5,
			  struct camss_a5_fw_memory *memory, u32 iova, u32 bytes)
{
	int ret = camss_a5_iova_memory_alloc(a5->dev, memory, iova, bytes);

	return ret ?: camss_a5_fw_memory_map(memory);
}

static bool transfer_released(struct camss_a5 *a5)
{
	return !a5->session_uncertain && !a5->ipe_dma_pending &&
	       !a5->config_fw_mapped && !a5->request_fw_mapped &&
	       !a5->request_owner && !a5->watchdogs && !a5->protocol_errors &&
	       !a5->request.cpu && !a5->request.mapped &&
	       !a5->image_input.cpu && !a5->image_input.mapped &&
	       !a5->image_output.cpu && !a5->image_output.mapped &&
	       !a5->dma_input.buffer && !a5->dma_output.buffer;
}

static int run_colour_status(struct camss_a5 *a5, struct camss_a5_session *session,
			     const struct camss_a5_ipe_job *job,
			     const struct camss_a5_ipe_cc *cc,
			     const struct camss_a5_ipe_colour *chain,
			     const struct camss_a5_ipe_gamma *gamma,
			     const struct camss_a5_ipe_ltm *ltm,
			     enum camss_a5_ipe_result *result)
{
	struct camss_a5_ipe_frame frame = {};
	struct camss_a5_ipe_cc colour;
	struct camss_a5_ipe_colour configuration;
	struct camss_a5_ipe_gamma *gamma_copy = NULL;
	struct camss_a5_ipe_ltm *ltm_copy = NULL;
	struct camss_a5_ipe_ltm_geometry geometry = {};
	u8 colour_command[CAMSS_A5_IPE_CC_COMMAND_BYTES];
	u32 iq_bytes = ltm ? CAMSS_A5_IPE_LTM_IQ_BYTES : gamma ? CAMSS_A5_IPE_GAMMA_IQ_BYTES :
		chain ? CAMSS_A5_IPE_COLOUR_IQ_BYTES :
		cc ? CAMSS_A5_IPE_CC_IQ_BYTES : CAMSS_A5_IPE_IQ_BYTES;
	u8 packet[40] = {};
	u64 input_bytes, output_bytes, parcel;
	u32 stripe_bytes, iq_offset, scratch_offset, bl_offset;
	bool direct, local_ready = false, firmware_attempted = false;
	int ret, cleanup = 0;

	if (result)
		*result = CAMSS_A5_IPE_RESET_REQUIRED;

	if (!a5 || !session || !job)
		return -EINVAL;
	direct = job->input_dma || job->output_dma;
	mutex_lock(&a5->lock);
	if (session->a5 != a5 || !session->handle || !session->configured ||
	    !a5->initialized || !a5->cpu_enabled || !a5->irq_enabled ||
	    !a5->engine_dma_capable) {
		ret = -ENODEV;
		goto done;
	}
	if (!transfer_released(a5)) {
		ret = -EBUSY;
		goto done;
	}
	/* Only a configured, coherent engine may reject a job locally. */
	local_ready = true;
	if (cc && (chain || gamma || ltm)) {
		ret = -EINVAL;
		goto done;
	}
	if (chain) {
		configuration = *chain;
		ret = camss_a5_ipe_colour_check(&configuration);
		if (ret)
			goto done;
	}
	if (gamma) {
		gamma_copy = kmemdup(gamma, sizeof(*gamma), GFP_KERNEL);
		if (!gamma_copy) {
			ret = -ENOMEM;
			goto done;
		}
		ret = camss_a5_ipe_gamma_check(gamma_copy);
		if (ret)
			goto done;
	}
	if (ltm) {
		ltm_copy = kmemdup(ltm, sizeof(*ltm), GFP_KERNEL);
		if (!ltm_copy) {
			ret = -ENOMEM;
			goto done;
		}
		ret = camss_a5_ipe_ltm_check(ltm_copy);
		if (ret)
			goto done;
		geometry.data_collection = ltm_copy->iq.data_collection;
		geometry.image_processing = ltm_copy->iq.image_processing;
		geometry.mask_filter = ltm_copy->iq.mask_filter;
	}
	if (cc) {
		colour = *cc;
		ret = camss_a5_ipe_cc_encode(colour_command, sizeof(colour_command), &colour);
		if (ret)
			goto done;
	}
	if ((direct && (!job->input_dma || !job->output_dma || job->input || job->output)) ||
	    (!direct && (!job->input || !job->output || job->input_offset || job->output_offset)) ||
	    !job->bl_bytes || job->bl_bytes < 32 ||
	    !job->max_cores || job->max_cores > 2 || !job->target_time_ns) {
		ret = -EINVAL;
		goto done;
	}
	if (!nv12_layout_valid(&session->io.input) ||
	    !nv12_layout_valid(&session->io.output)) {
		ret = -EINVAL;
		goto done;
	}
	ret = ltm_copy ?
		camss_a5_ipe_ltm_size(&session->io, job->max_cores, &geometry, &stripe_bytes) :
		camss_a5_ipe_bypass_size(&session->io, job->max_cores, &stripe_bytes);
	if (ret)
		goto done;
	input_bytes = (u64)session->io.input.stride * session->io.input.slice_height;
	input_bytes += input_bytes / 2;
	output_bytes = (u64)session->io.output.stride * session->io.output.slice_height;
	output_bytes += output_bytes / 2;
	if (!input_bytes || !output_bytes || input_bytes > IMAGE_WINDOW ||
	    output_bytes > IMAGE_WINDOW || job->input_bytes < input_bytes ||
	    job->output_bytes < output_bytes || session->scratch_bytes > REQUEST_MAX ||
	    job->bl_bytes > REQUEST_MAX || stripe_bytes > REQUEST_MAX) {
		ret = -EINVAL;
		goto done;
	}
	/* The stripe begins on its own page after the frame packet. */
	iq_offset = PAGE_SIZE + PAGE_ALIGN(stripe_bytes);
	scratch_offset = iq_offset + PAGE_ALIGN(iq_bytes);
	bl_offset = scratch_offset + PAGE_ALIGN(session->scratch_bytes);
	parcel = PAGE_ALIGN((u64)bl_offset + job->bl_bytes);
	if (!session->scratch_bytes || parcel > REQUEST_MAX) {
		ret = -EINVAL;
		goto done;
	}
	ret = transfer_alloc(a5, &a5->request, REQUEST_IOVA, parcel);
	if (ret)
		goto release;
	if (direct) {
		ret = camss_a5_dma_image_map(a5->dev, &a5->dma_input, job->input_dma,
					     job->input_offset, input_bytes, DMA_TO_DEVICE);
		if (!ret)
			ret = camss_a5_dma_image_map(a5->dev, &a5->dma_output, job->output_dma,
						     job->output_offset, output_bytes, DMA_FROM_DEVICE);
		if (!ret)
			ret = camss_a5_dma_images_disjoint(&a5->dma_input, &a5->dma_output);
	} else {
		ret = transfer_alloc(a5, &a5->image_input, IMAGE_INPUT_IOVA, PAGE_ALIGN(input_bytes));
		if (!ret)
			ret = transfer_alloc(a5, &a5->image_output, IMAGE_OUTPUT_IOVA, PAGE_ALIGN(output_bytes));
	}
	if (ret)
		goto release;
	if (ltm_copy)
		ret = camss_a5_ipe_encode_ltm_striping(a5->request.cpu + PAGE_SIZE,
						      stripe_bytes, &session->io,
						      job->max_cores, &geometry, &stripe_bytes);
	else
		ret = camss_a5_ipe_encode_bypass_striping(a5->request.cpu + PAGE_SIZE,
						 stripe_bytes, &session->io,
						 job->max_cores, &stripe_bytes);
	if (ret)
		goto release;
	if (ltm_copy)
		ret = camss_a5_ipe_encode_ltm_iq(a5->request.cpu + iq_offset, iq_bytes,
					      REQUEST_IOVA + iq_offset,
					      chain ? &configuration : NULL,
					      gamma_copy, ltm_copy);
	else if (gamma)
		ret = camss_a5_ipe_encode_gamma_iq(a5->request.cpu + iq_offset, iq_bytes,
						 REQUEST_IOVA + iq_offset,
						 chain ? &configuration : NULL, gamma_copy);
	else if (chain)
		ret = camss_a5_ipe_encode_colour_iq(a5->request.cpu + iq_offset, iq_bytes,
						  REQUEST_IOVA + iq_offset, &configuration);
	else if (cc)
		ret = camss_a5_ipe_encode_cc_iq(a5->request.cpu + iq_offset, iq_bytes,
					      REQUEST_IOVA + iq_offset, &colour);
	else
		ret = camss_a5_ipe_encode_bypass_iq(a5->request.cpu + iq_offset, iq_bytes);
	if (ret)
		goto release;
	if (!direct) {
		memcpy(a5->image_input.cpu, job->input, input_bytes);
		memset(a5->image_output.cpu, 0xa5, output_bytes);
	}
	frame.striping = (struct camss_a5_ipe_buffer){ REQUEST_IOVA + PAGE_SIZE, stripe_bytes };
	frame.iq = (struct camss_a5_ipe_buffer){ REQUEST_IOVA + iq_offset, iq_bytes };
	frame.scratch = (struct camss_a5_ipe_buffer){ REQUEST_IOVA + scratch_offset, session->scratch_bytes };
	frame.bl = (struct camss_a5_ipe_buffer){ REQUEST_IOVA + bl_offset, job->bl_bytes };
	frame.input = (struct camss_a5_ipe_buffer){ direct ? a5->dma_input.iova : IMAGE_INPUT_IOVA, input_bytes };
	frame.output = (struct camss_a5_ipe_buffer){ direct ? a5->dma_output.iova : IMAGE_OUTPUT_IOVA, output_bytes };
	frame.striping_cpu = a5->request.cpu + PAGE_SIZE;
	frame.required_scratch_bytes = session->scratch_bytes;
	frame.required_bl_bytes = job->bl_bytes;
	frame.request_id = job->request_id;
	frame.max_cores = job->max_cores;
	frame.target_time_ns = job->target_time_ns;
	ret = camss_a5_ipe_encode_frame(a5->request.cpu, a5->request.bytes,
					 &session->io, &frame);
	if (ret)
		goto release;
	if (ltm_copy) {
		put_unaligned_le32(REQUEST_IOVA + iq_offset + CAMSS_A5_IPE_COLOUR_ARRAY_OFFSET,
				   a5->request.cpu + 40);
		put_unaligned_le32(CAMSS_A5_IPE_LTM_PRE_OFFSET, a5->request.cpu + 44);
		put_unaligned_le32(CAMSS_A5_IPE_LTM_POST_OFFSET, a5->request.cpu + 48);
	} else if (gamma || chain) {
		put_unaligned_le32(REQUEST_IOVA + iq_offset + CAMSS_A5_IPE_COLOUR_ARRAY_OFFSET,
				   a5->request.cpu + 40);
		put_unaligned_le32(CAMSS_A5_IPE_COLOUR_PRE_OFFSET, a5->request.cpu + 44);
		put_unaligned_le32(CAMSS_A5_IPE_COLOUR_POST_OFFSET, a5->request.cpu + 48);
	} else if (cc) {
		put_unaligned_le32(REQUEST_IOVA + iq_offset + CAMSS_A5_IPE_CC_ARRAY_OFFSET,
				   a5->request.cpu + 40);
		put_unaligned_le32(CAMSS_A5_IPE_CC_POST_OFFSET, a5->request.cpu + 48);
	}
	dma_wmb();
	/* Any attempted firmware MAP crosses the conservative reset boundary. */
	firmware_attempted = true;
	ret = transfer_region_locked(a5, session, true);
	if (ret)
		goto release;
	ret = session_token(a5);
	if (ret)
		goto unmap;
	a5->session_opcode = IPE_FRAME_PROCESS;
	put_unaligned_le32(sizeof(packet), packet);
	put_unaligned_le32(IPE_ASYNC_INDIRECT, packet + 4);
	put_unaligned_le32(IPE_FRAME_PROCESS, packet + 8);
	put_unaligned_le64(a5->session_cookie, packet + 12);
	put_unaligned_le64(a5->session_cookie2, packet + 20);
	put_unaligned_le32(1, packet + 28);
	put_unaligned_le32(session->handle, packet + 32);
	put_unaligned_le32(REQUEST_IOVA, packet + 36);
	a5->ipe_dma_pending = true;
	ret = send_packet(a5, packet, sizeof(packet), IPE_ASYNC_INDIRECT_ACK);
	if (a5->command_queued)
		a5->frames_submitted++;
	if (!a5->command_queued || !ret)
		a5->ipe_dma_pending = false;
	if (ret) {
		session_command_failed(a5, ret);
		if (a5->ipe_dma_pending)
			a5->session_uncertain = true;
		dev_info(a5->dev, "A5 IPE frame failed ret=%d hfi=%u status=%u queued=%u pending=%u\n",
			 ret, a5->session_header_error, a5->session_payload_status,
			 a5->command_queued, a5->ipe_dma_pending);
	} else {
		a5->frames_completed++;
		dma_rmb();
		if (direct)
			a5->direct_frames++;
		else {
			a5->copy_frames++;
			memcpy(job->output, a5->image_output.cpu, output_bytes);
		}
		dev_info(a5->dev, "A5 IPE frame completed request=%u\n", job->request_id);
	}
unmap:
	if (!a5->session_uncertain && !a5->ipe_dma_pending) {
		cleanup = transfer_region_locked(a5, session, false);
		if (!ret)
			ret = cleanup;
	}
release:
	if (!a5->request_fw_mapped && !a5->ipe_dma_pending) {
		cleanup = free_transfer_memory(a5);
		if (!ret)
			ret = cleanup;
	}
done:
	if (result) {
		if (!ret)
			*result = CAMSS_A5_IPE_COMPLETE;
		else if (local_ready && !firmware_attempted && !cleanup &&
			 transfer_released(a5))
			*result = CAMSS_A5_IPE_REJECTED;
	}
	kfree(ltm_copy);
	kfree(gamma_copy);
	mutex_unlock(&a5->lock);
	return ret;
}
int camss_a5_ipe_run_status(struct camss_a5 *a5, struct camss_a5_session *session,
			  const struct camss_a5_ipe_job *job,
			  enum camss_a5_ipe_result *result)
{
	return run_colour_status(a5, session, job, NULL, NULL, NULL, NULL, result);
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_run_status);

int camss_a5_ipe_run_colour_status(struct camss_a5 *a5,
				 struct camss_a5_session *session,
				 const struct camss_a5_ipe_job *job,
				 const struct camss_a5_ipe_cc *cc,
				 enum camss_a5_ipe_result *result)
{
	return run_colour_status(a5, session, job, cc, NULL, NULL, NULL, result);
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_run_colour_status);

int camss_a5_ipe_run_colour_chain_status(struct camss_a5 *a5,
				       struct camss_a5_session *session,
				       const struct camss_a5_ipe_job *job,
				       const struct camss_a5_ipe_colour *colour,
				       enum camss_a5_ipe_result *result)
{
	return run_colour_status(a5, session, job, NULL, colour, NULL, NULL, result);
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_run_colour_chain_status);

int camss_a5_ipe_run_gamma_status(struct camss_a5 *a5,
				struct camss_a5_session *session,
				const struct camss_a5_ipe_job *job,
				const struct camss_a5_ipe_colour *colour,
				const struct camss_a5_ipe_gamma *gamma,
				enum camss_a5_ipe_result *result)
{
	return run_colour_status(a5, session, job, NULL, colour, gamma, NULL, result);
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_run_gamma_status);

int camss_a5_ipe_run_ltm_status(struct camss_a5 *a5,
			      struct camss_a5_session *session,
			      const struct camss_a5_ipe_job *job,
			      const struct camss_a5_ipe_colour *colour,
			      const struct camss_a5_ipe_gamma *gamma,
			      const struct camss_a5_ipe_ltm *ltm,
			      enum camss_a5_ipe_result *result)
{
	return run_colour_status(a5, session, job, NULL, colour, gamma, ltm, result);
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_run_ltm_status);

/* Preserve existing in-kernel callers and their conservative errno contract. */
int camss_a5_ipe_run(struct camss_a5 *a5, struct camss_a5_session *session,
		     const struct camss_a5_ipe_job *job)
{
	return camss_a5_ipe_run_status(a5, session, job, NULL);
}
EXPORT_SYMBOL_GPL(camss_a5_ipe_run);
