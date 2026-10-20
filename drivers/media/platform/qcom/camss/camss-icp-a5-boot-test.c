// SPDX-License-Identifier: GPL-2.0-only
/* Temporary A5/IPE synthetic frame diagnostic; no sensor changes. */
#include <linux/device/bus.h>
#include <linux/dma-mapping.h>
#include <linux/crc32.h>
#include <linux/vmalloc.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "camss-icp-a5-boot.h"
#include "camss-icp-a5-ipe.h"
#include "camss-icp-a5-boot-test-blob.h"

static DEFINE_MUTEX(test_lock);
static struct camss_a5 *device_a5;
static struct camss_a5_session session_rt, session_non_rt;
static u32 scratch_observed[2];
static const struct camss_a5_ipe_io test_io = {
	.input = { .width = 640, .height = 480, .stride = 640, .slice_height = 480 },
	.output = { .width = 640, .height = 480, .stride = 640, .slice_height = 480 },
	.output_port = 8,
};
static char result[2048] = "{\"terminal\":false}";
static char sfr[512];
module_param_string(result, result, sizeof(result), 0444);
module_param_string(sfr, sfr, sizeof(sfr), 0444);
static int overlay_id, test_error;
static bool finished, pinned, held_failure;
static bool bound_seen;
module_param(bound_seen, bool, 0444);
static bool hold_start_failure;
module_param(hold_start_failure, bool, 0444);
MODULE_PARM_DESC(hold_start_failure, "Keep quiesced failed boot RAM and PM until cleanup");
static const char *phase = "waiting";
static unsigned int group_devices;

/* Temporary diagnostic ranges are supplied from the verified ELF profile.
 * No OEM symbol address is part of the reusable SoC transport API.
 */
static unsigned int snapshot_offsets[4], snapshot_words[4];
static unsigned int snapshot_offset_count, snapshot_word_count;
static const u8 *snapshot_boot_code;
module_param_array(snapshot_offsets, uint, &snapshot_offset_count, 0444);
module_param_array(snapshot_words, uint, &snapshot_word_count, 0444);

#define TEST_IMAGE_BYTES (640U * 480U * 3U / 2U)
static void *test_output;
static u32 test_output_bytes, test_input_crc, test_output_crc;
static bool enable_frame;
module_param(enable_frame, bool, 0400);

static int test_frame(struct device *dev, struct camss_a5 *a5)
{
	const struct firmware *stripe = NULL;
	struct camss_a5_ipe_job job = {};
	u8 *input;
	unsigned int x, y;
	int ret;

	test_output_bytes = test_input_crc = test_output_crc = 0;
	kvfree(test_output);
	test_output = NULL;
	input = kvzalloc(TEST_IMAGE_BYTES, GFP_KERNEL);
	if (!input)
		return -ENOMEM;
	test_output = kvzalloc(TEST_IMAGE_BYTES, GFP_KERNEL);
	if (!test_output) {
		ret = -ENOMEM;
		goto out;
	}
	ret = request_firmware_direct(&stripe, "qcom/sm8150/samurai-ipe382-stripe.bin", dev);
	if (ret)
		goto out;
	for (y = 0; y < 480; y++)
		for (x = 0; x < 640; x++)
			input[y * 640 + x] = 16 + ((x / 8 + y / 8 * 3) % 28) * 8;
	for (y = 0; y < 240; y++) {
		for (x = 0; x < 640; x += 2) {
			input[640 * 480 + y * 640 + x] = 96 + (x / 80 % 2) * 64;
			input[640 * 480 + y * 640 + x + 1] = 64 + (y / 60 % 2) * 96;
		}
	}
	test_input_crc = crc32_le(0, input, TEST_IMAGE_BYTES);
	job.striping = stripe->data;
	job.striping_bytes = stripe->size;
	job.input = input;
	job.input_bytes = TEST_IMAGE_BYTES;
	job.output = test_output;
	job.output_bytes = TEST_IMAGE_BYTES;
	job.bl_bytes = 156512;
	job.request_id = 1;
	job.max_cores = 1;
	job.target_time_ns = 100000000;
	ret = camss_a5_ipe_run(a5, &session_rt, &job);
	if (!ret) {
		test_output_bytes = TEST_IMAGE_BYTES;
		test_output_crc = crc32_le(0, test_output, TEST_IMAGE_BYTES);
	}
out:
	if (stripe)
		release_firmware(stripe);
	kvfree(input);
	return ret;
}

static ssize_t image_read(struct file *file, struct kobject *kobj,
			  const struct bin_attribute *attr, char *buffer,
			  loff_t offset, size_t count)
{
	ssize_t ret = -ENODATA;

	mutex_lock(&test_lock);
	if (!finished || test_error || !test_output || !test_output_bytes)
		goto out;
	if (offset < 0) {
		ret = -EINVAL;
		goto out;
	}
	ret = 0;
	if (offset >= test_output_bytes)
		goto out;
	count = min_t(size_t, count, test_output_bytes - offset);
	memcpy(buffer, test_output + offset, count);
	ret = count;
out:
	mutex_unlock(&test_lock);
	return ret;
}
static const struct bin_attribute image_attribute = {
	.attr = { .name = "image", .mode = 0400 },
	.size = TEST_IMAGE_BYTES,
	.read = image_read,
};

static bool snapshot_ranges_valid(u32 bytes)
{
	unsigned int i;

	if (!snapshot_offset_count || snapshot_offset_count > ARRAY_SIZE(snapshot_offsets) ||
	    snapshot_offset_count != snapshot_word_count)
		return false;
	for (i = 0; i < snapshot_offset_count; i++) {
		if (!IS_ALIGNED(snapshot_offsets[i], sizeof(u32)) ||
		    !snapshot_words[i] || snapshot_words[i] > 96 ||
		    snapshot_offsets[i] > bytes ||
		    snapshot_words[i] > (bytes - snapshot_offsets[i]) / sizeof(u32))
			return false;
	}
	return true;
}

static void failure_snapshot(struct camss_a5 *a5)
{
	unsigned int i, j, k, count;
	u32 words[8], offset;
	bool match;

	dev_info(a5->dev, "A5 pre-reset snapshot begin ranges=%u\n", snapshot_offset_count);
	if (!a5->power_held || !a5->firmware.cpu || !snapshot_boot_code ||
	    a5->firmware.bytes < 4096 || a5->firmware.mapped != a5->firmware.bytes ||
	    !snapshot_ranges_valid(a5->firmware.bytes)) {
		dev_info(a5->dev, "A5 pre-reset snapshot unavailable\n");
		return;
	}
	dma_rmb();
	match = !memcmp(a5->firmware.cpu, snapshot_boot_code, 4096);
	dev_info(a5->dev, "A5 pre-reset snapshot code-match=%u\n", match);
	if (!match)
		return;
	for (i = 0; i < snapshot_offset_count; i++) {
		for (j = 0; j < snapshot_words[i]; j += ARRAY_SIZE(words)) {
			count = min_t(unsigned int, ARRAY_SIZE(words), snapshot_words[i] - j);
			offset = snapshot_offsets[i] + j * sizeof(u32);
			memset(words, 0, sizeof(words));
			for (k = 0; k < count; k++)
				words[k] = get_unaligned_le32(a5->firmware.cpu + offset + k * sizeof(u32));
			dev_info(a5->dev, "A5 pre-reset snapshot data offset=%x count=%u values=%08x,%08x,%08x,%08x,%08x,%08x,%08x,%08x\n",
				 offset, count, words[0], words[1], words[2], words[3],
				 words[4], words[5], words[6], words[7]);
		}
	}
	dev_info(a5->dev, "A5 pre-reset snapshot end\n");
}

static int configure_snapshot(struct camss_a5 *a5, const struct firmware *firmware)
{
	struct camss_a5_fw_layout layout;
	u32 table;
	unsigned int i, count;
	int ret;

	a5->failure_snapshot = NULL;
	snapshot_boot_code = NULL;
	if (!snapshot_offset_count && !snapshot_word_count)
		return 0;
	ret = camss_a5_fw_inspect(firmware, CAMSS_A5_FW_WINDOW, &layout);
	if (ret)
		return ret;
	if (!snapshot_ranges_valid(layout.span))
		return -EINVAL;
	table = get_unaligned_le32(firmware->data + 28);
	count = get_unaligned_le16(firmware->data + 44);
	for (i = 0; i < count; i++) {
		const u8 *header = firmware->data + table + i * sizeof(Elf32_Phdr);

		if (get_unaligned_le32(header) == PT_LOAD &&
		    !get_unaligned_le32(header + 8) &&
		    get_unaligned_le32(header + 16) >= 4096 &&
		    (get_unaligned_le32(header + 24) & PF_X)) {
			snapshot_boot_code = firmware->data + get_unaligned_le32(header + 4);
			a5->failure_snapshot = failure_snapshot;
			return 0;
		}
	}
	return -ENOEXEC;
}


static int group_device(struct device *dev, void *target)
{
	group_devices++;
	return dev == target ? 0 : -EBUSY;
}

static void update_result(void)
{
	struct camss_a5 *a5 = device_a5;

	if (!a5) {
		scnprintf(result, sizeof(result),
			  "{\"terminal\":%s,\"phase\":\"%s\",\"error\":%d,\"backing_owned\":false,\"cpu_enabled\":false}",
			  finished ? "true" : "false", phase, test_error);
		return;
	}
	strscpy(sfr, a5->sfr, sizeof(sfr));
	scnprintf(result, sizeof(result),
		  "{\"terminal\":%s,\"phase\":\"%s\",\"error\":%d,\"cleanup_error\":%d,\"pc_error\":%d,"
		  "\"cpu_release_attempted\":%s,\"init_response\":%u,\"fw_version\":%u,\"response_before\":%u,\"init_bytes\":%u,"
		  "\"init_done\":%s,\"ping_done\":%s,\"pc_done\":%s,\"reset_asserted\":%s,"
		  "\"cpu_enabled\":%s,\"irq_enabled\":%s,\"power_held\":%s,\"backing_owned\":%s,"
		  "\"mapped_bytes\":%u,\"remaining_groups\":%u,\"remaining_domains\":%u,"
		  "\"interrupts\":%u,\"messages\":%u,\"debug_packets\":%u,\"watchdogs\":%u,\"protocol_errors\":%u,"
		  "\"last_type\":%u,\"queue_state\":[%u,%u,%u,%u,%u,%u],\"image_sessions\":%u,\"session_creates\":%u,\"session_destroys\":%u,\"session_peak\":%u,\"session_uncertain\":%s,\"io_configs\":%u,\"config_maps\":%u,\"config_unmaps\":%u,\"config_fw_mapped\":%s,\"scratch_bytes\":[%u,%u],\"image_frames\":%u,\"frames_completed\":%u,\"transfer_maps\":%u,\"transfer_unmaps\":%u,\"ipe_dma_pending\":%s,\"request_fw_mapped\":%s,\"engine_dma_capable\":%s,\"ipe_reset_acks\":[%u,%u],\"image_bytes\":%u,\"input_crc\":%u,\"output_crc\":%u}",
		  finished ? "true" : "false", phase, test_error, a5->cleanup_error, a5->pc_error,
		  a5->cpu_release_attempted ? "true" : "false", a5->init_response, a5->fw_version,
		  a5->response_before, a5->init_bytes,
		  a5->init_done ? "true" : "false", a5->ping_done ? "true" : "false",
		  a5->pc_done ? "true" : "false", a5->reset_asserted ? "true" : "false",
		  a5->cpu_enabled ? "true" : "false", a5->irq_enabled ? "true" : "false",
		  a5->power_held ? "true" : "false", camss_a5_backing_owned(a5) ? "true" : "false",
		  a5->firmware.mapped + a5->shared.mapped + a5->heap.mapped + a5->config.mapped +
		  a5->request.mapped + a5->image_input.mapped + a5->image_output.mapped,
		  a5->resources.enabled_groups, a5->resources.held_domains,
		  a5->interrupts, a5->messages, a5->debug_packets, a5->watchdogs, a5->protocol_errors,
		  a5->last_type, a5->queue_state[0], a5->queue_state[1], a5->queue_state[2],
		  a5->queue_state[3], a5->queue_state[4], a5->queue_state[5],
		  a5->session_count, a5->session_creates, a5->session_destroys, a5->session_peak,
		  a5->session_uncertain ? "true" : "false", a5->io_configs, a5->config_maps, a5->config_unmaps,
		  a5->config_fw_mapped ? "true" : "false", scratch_observed[0], scratch_observed[1],
		  a5->frames_submitted, a5->frames_completed, a5->transfer_maps, a5->transfer_unmaps,
		  a5->ipe_dma_pending ? "true" : "false", a5->request_fw_mapped ? "true" : "false",
		  a5->engine_dma_capable ? "true" : "false", a5->resources.ipe_reset_acks[0],
		  a5->resources.ipe_reset_acks[1], test_output_bytes, test_input_crc, test_output_crc);
}

static int cleanup(void)
{
	int ret = device_a5 ? camss_a5_stop(device_a5) : 0;

	if (!ret)
		held_failure = false;
	if (!ret && pinned && !camss_a5_backing_owned(device_a5) &&
	    !device_a5->cpu_enabled && !device_a5->power_held &&
	    !device_a5->resources.enabled_groups && !device_a5->resources.held_domains) {
		pinned = false;
		module_put(THIS_MODULE);
	}
	return ret;
}

static ssize_t cleanup_store(struct device *dev, struct device_attribute *attr,
			     const char *buffer, size_t count)
{
	int ret;

	if (!sysfs_streq(buffer, "1"))
		return -EINVAL;
	mutex_lock(&test_lock);
	ret = cleanup();
	if (ret)
		test_error = ret;
	else
		phase = "cleaned";
	update_result();
	mutex_unlock(&test_lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(cleanup);
/* Temporary root-only diagnostic view. No /dev/mem access and no device
 * access after cleanup: reads are serialized against the same owner lock.
 */
static ssize_t firmware_read(struct file *file, struct kobject *kobj,
			     const struct bin_attribute *attr, char *buffer,
			     loff_t offset, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct camss_a5 *a5;
	ssize_t ret = -ENODATA;

	mutex_lock(&test_lock);
	a5 = device_a5;
	if (!held_failure || !a5 || a5 != dev_get_drvdata(dev))
		goto done;
	mutex_lock(&a5->lock);
	if (a5->cpu_enabled || a5->irq_enabled || !a5->reset_asserted ||
	    !a5->power_held || !a5->firmware.cpu ||
	    a5->firmware.mapped != a5->firmware.bytes)
		goto unlock;
	if (offset < 0) {
		ret = -EINVAL;
		goto unlock;
	}
	ret = 0;
	if (offset >= a5->firmware.bytes)
		goto unlock;
	count = min_t(size_t, count, a5->firmware.bytes - offset);
	dma_rmb();
	memcpy(buffer, a5->firmware.cpu + offset, count);
	ret = count;
unlock:
	mutex_unlock(&a5->lock);
done:
	mutex_unlock(&test_lock);
	return ret;
}
static const struct bin_attribute firmware_attribute = {
	.attr = { .name = "firmware", .mode = 0400 },
	.size = CAMSS_A5_FW_WINDOW,
	.read = firmware_read,
};
static const struct bin_attribute *test_bin_attributes[] = { &firmware_attribute, &image_attribute, NULL };
static void run_locked(struct device *dev);

static ssize_t run_store(struct device *dev, struct device_attribute *attr,
			 const char *buffer, size_t count)
{
	int ret = 0;

	if (!sysfs_streq(buffer, "1"))
		return -EINVAL;
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;
	mutex_lock(&test_lock);
	if (!device_a5 || device_a5 != dev_get_drvdata(dev) || !finished ||
	    !READ_ONCE(bound_seen) || camss_a5_backing_owned(device_a5) ||
	    device_a5->cpu_enabled || device_a5->power_held ||
	    device_a5->resources.enabled_groups || device_a5->resources.held_domains) {
		ret = -EBUSY;
	} else {
		run_locked(dev);
	}
	mutex_unlock(&test_lock);
	module_put(THIS_MODULE);
	if (ret)
		return ret;
	return count;
}
static DEVICE_ATTR_WO(run);
static struct attribute *test_attributes[] = { &dev_attr_cleanup.attr, &dev_attr_run.attr, NULL };
static const struct attribute_group test_group = {
	.attrs = test_attributes,
	.bin_attrs = test_bin_attributes,
};
static const struct attribute_group *test_groups[] = { &test_group, NULL };

static int test_resume(struct device *dev)
{
	struct camss_a5 *a5 = dev_get_drvdata(dev);

	return camss_a5_runtime_resume(a5);
}

static int test_suspend(struct device *dev)
{
	struct camss_a5 *a5 = dev_get_drvdata(dev);

	return camss_a5_runtime_suspend(a5);
}

static const struct dev_pm_ops test_ops = {
	RUNTIME_PM_OPS(test_suspend, test_resume, NULL)
};

static int boot_test_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct camss_a5 *a5;
	struct iommu_group *group;
	struct device_node *memory;
	int ret;

	a5 = devm_kzalloc(dev, sizeof(*a5), GFP_KERNEL);
	if (!a5)
		return -ENOMEM;
	mutex_lock(&test_lock);
	phase = "iommu-group";
	group_devices = 0;
	group = iommu_group_get(dev);
	ret = group ? iommu_group_for_each_dev(group, dev, group_device) : -ENODEV;
	if (group)
		iommu_group_put(group);
	if (ret || group_devices != 1) {
		test_error = ret ? ret : -EBUSY;
		finished = true;
		update_result();
		mutex_unlock(&test_lock);
		return test_error;
	}
	/* An offline fdtoverlay merge does not prove live phandle resolution. */
	phase = "firmware-reference";
	memory = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (!memory) {
		test_error = -ENXIO;
		finished = true;
		update_result();
		mutex_unlock(&test_lock);
		return dev_err_probe(dev, test_error, "Firmware memory phandle is unresolved\n");
	}
	dev_info(dev, "Firmware memory resolves to %pOF, no-map=%d\n",
		 memory, of_property_present(memory, "no-map"));
	of_node_put(memory);
	phase = "resources";
	ret = camss_a5_init(pdev, a5);
	if (ret) {
		test_error = ret;
		finished = true;
		update_result();
		mutex_unlock(&test_lock);
		return ret;
	}
	device_a5 = a5;
	platform_set_drvdata(pdev, a5);
	pm_runtime_set_suspended(dev);
	pm_runtime_enable(dev);
	phase = "ready";
	finished = true;
	test_error = 0;
	update_result();
	dev_info(dev, "A5 diagnostic probe ready; no firmware started\n");
	mutex_unlock(&test_lock);
	return 0;
}

static void run_locked(struct device *dev)
{
	struct camss_a5 *a5 = device_a5;
	const struct firmware *firmware;
	int ret, stop;

	finished = false;
	dev_info(dev, "A5 diagnostic run begin after binding\n");
	phase = "request-firmware";
	ret = request_firmware_direct(&firmware, "qcom/sm8150/samurai-icp356.elf", dev);
	if (ret)
		goto done;
	phase = "snapshot-profile";
	ret = configure_snapshot(a5, firmware);
	if (!ret) {
		phase = "start";
		ret = camss_a5_start(a5, firmware);
	}
	a5->failure_snapshot = NULL;
	snapshot_boot_code = NULL;
	release_firmware(firmware);
	if (ret)
		goto done;
	phase = "ping";
	ret = camss_a5_ping(a5, 0x353536706d636173ULL);
	if (ret)
		goto done;
	phase = "create-ipe-rt";
	ret = camss_a5_ipe_create(a5, &session_rt, CAMSS_A5_IPE_RT);
	if (ret)
		goto done;
	phase = "configure-ipe-rt";
	ret = camss_a5_ipe_configure_nv12(a5, &session_rt, &test_io);
	if (ret)
		goto done;
	scratch_observed[0] = session_rt.scratch_bytes;
	phase = "create-ipe-non-rt";
	ret = camss_a5_ipe_create(a5, &session_non_rt, CAMSS_A5_IPE_NON_RT);
	if (ret)
		goto done;
	phase = "configure-ipe-non-rt";
	ret = camss_a5_ipe_configure_nv12(a5, &session_non_rt, &test_io);
	if (ret)
		goto done;
	scratch_observed[1] = session_non_rt.scratch_bytes;
	if (enable_frame) {
		phase = "process-ipe-rt";
		ret = test_frame(dev, a5);
		if (ret)
			goto done;
	}
	phase = "destroy-ipe-non-rt";
	ret = camss_a5_ipe_destroy(a5, &session_non_rt);
	if (ret)
		goto done;
	phase = "destroy-ipe-rt";
	ret = camss_a5_ipe_destroy(a5, &session_rt);
done:
	if (ret && hold_start_failure && a5->cpu_release_attempted &&
	    !a5->cleanup_error && !a5->cpu_enabled && !a5->irq_enabled &&
	    a5->reset_asserted && a5->power_held && a5->firmware.cpu &&
	    a5->firmware.mapped == a5->firmware.bytes) {
		held_failure = true;
		phase = "failed-held";
	} else {
		stop = cleanup();
		if (stop)
			ret = stop;
	}
	if ((camss_a5_backing_owned(a5) || a5->cpu_enabled || a5->power_held ||
	     a5->resources.enabled_groups || a5->resources.held_domains) && !pinned) {
		__module_get(THIS_MODULE);
		pinned = true;
	}
	if (!ret && a5->pc_error)
		ret = a5->pc_error;
	test_error = ret;
	finished = true;
	if (!ret)
		phase = "done";
	update_result();
	dev_info(dev, "A5 diagnostic run end ret=%d init=%u ping=%u pc=%u irqs=%u msgs=%u wd=%u proto=%u backing=%u power=%u groups=%u domains=%u\n",
		 test_error, a5->init_done, a5->ping_done, a5->pc_done,
		 a5->interrupts, a5->messages, a5->watchdogs, a5->protocol_errors,
		 camss_a5_backing_owned(a5), a5->power_held,
		 a5->resources.enabled_groups, a5->resources.held_domains);
}

static void boot_test_remove(struct platform_device *pdev)
{
	mutex_lock(&test_lock);
	cleanup();
	pm_runtime_disable(&pdev->dev);
	/* The driver core may runtime-resume the device before ->remove. */
	if (device_a5->resources.enabled_groups || device_a5->resources.held_domains)
		camss_a5_runtime_suspend(device_a5);
	camss_a5_resources_release(&device_a5->resources);
	kvfree(test_output);
	test_output = NULL;
	test_output_bytes = 0;
	device_a5 = NULL;
	mutex_unlock(&test_lock);
}

static const struct of_device_id boot_test_match[] = {
	{ .compatible = "realme,samurai-a5-icp-boot-test" }, {}
};
MODULE_DEVICE_TABLE(of, boot_test_match);

static struct platform_driver boot_test_driver = {
	.probe = boot_test_probe,
	.remove = boot_test_remove,
	.driver = {
		.name = "samurai-a5-icp-boot-test",
		.of_match_table = boot_test_match,
		.pm = pm_ptr(&test_ops),
		.dev_groups = test_groups,
		.probe_type = PROBE_FORCE_SYNCHRONOUS,
		.suppress_bind_attrs = true,
	},
};

/* Observe the normal platform binding notification without altering links. */
static int test_bus_event(struct notifier_block *block, unsigned long action, void *data)
{
	struct device *dev = data;
	const char *event;

	if (!of_device_is_compatible(dev->of_node, "realme,samurai-a5-icp-boot-test"))
		return NOTIFY_DONE;
	switch (action) {
	case BUS_NOTIFY_BIND_DRIVER:
		event = "bind";
		break;
	case BUS_NOTIFY_BOUND_DRIVER:
		WRITE_ONCE(bound_seen, true);
		event = "bound";
		break;
	case BUS_NOTIFY_UNBIND_DRIVER:
		WRITE_ONCE(bound_seen, false);
		event = "unbind";
		break;
	case BUS_NOTIFY_UNBOUND_DRIVER:
	case BUS_NOTIFY_DRIVER_NOT_BOUND:
		WRITE_ONCE(bound_seen, false);
		event = "unbound";
		break;
	default:
		return NOTIFY_DONE;
	}
	dev_info(dev, "A5 diagnostic bus event=%s\n", event);
	return NOTIFY_DONE;
}
static struct notifier_block test_bus_notifier = { .notifier_call = test_bus_event };

static int __init boot_test_init(void)
{
	int ret;

	ret = bus_register_notifier(&platform_bus_type, &test_bus_notifier);
	if (ret)
		return ret;
	ret = platform_driver_register(&boot_test_driver);
	if (ret) {
		bus_unregister_notifier(&platform_bus_type, &test_bus_notifier);
		return ret;
	}
	pr_info("A5 diagnostic overlay apply begin\n");
	ret = of_overlay_fdt_apply(a5_boot_test_overlay,
				  sizeof(a5_boot_test_overlay), &overlay_id, NULL);
	pr_info("A5 diagnostic overlay apply end ret=%d bound=%u\n", ret, READ_ONCE(bound_seen));
	if (ret) {
		if (pinned)
			return 0;
		if (overlay_id > 0)
			of_overlay_remove(&overlay_id);
		platform_driver_unregister(&boot_test_driver);
		bus_unregister_notifier(&platform_bus_type, &test_bus_notifier);
	}
	return ret;
}

static void __exit boot_test_exit(void)
{
	int ret;

	pr_info("A5 diagnostic module exit begin\n");
	ret = of_overlay_remove(&overlay_id);
	pr_info("A5 diagnostic overlay remove ret=%d\n", ret);
	platform_driver_unregister(&boot_test_driver);
	bus_unregister_notifier(&platform_bus_type, &test_bus_notifier);
	pr_info("A5 diagnostic module exit end\n");
}

module_init(boot_test_init);
module_exit(boot_test_exit);
MODULE_DESCRIPTION("Temporary SM8150 A5 firmware bootstrap diagnostic");
MODULE_LICENSE("GPL");
