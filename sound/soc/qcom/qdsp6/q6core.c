// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2011-2017, The Linux Foundation. All rights reserved.
// Copyright (c) 2018, Linaro Limited

#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/jiffies.h>
#include <linux/soc/qcom/apr.h>
#include "q6core.h"
#include <trace/events/samurai_diag.h>
#include "q6dsp-errno.h"

#define ADSP_STATE_READY_TIMEOUT_MS    3000
#define Q6_READY_TIMEOUT_MS 100
#define AVCS_CMD_ADSP_EVENT_GET_STATE		0x0001290C
#define AVCS_CMDRSP_ADSP_EVENT_GET_STATE	0x0001290D
#define AVCS_GET_VERSIONS       0x00012905
#define AVCS_GET_VERSIONS_RSP   0x00012906
#define AVCS_CMD_GET_FWK_VERSION	0x001292c
#define AVCS_CMDRSP_GET_FWK_VERSION	0x001292d

struct avcs_svc_info {
	uint32_t service_id;
	uint32_t version;
} __packed;

struct avcs_cmdrsp_get_version {
	uint32_t build_id;
	uint32_t num_services;
	struct avcs_svc_info svc_api_info[];
} __packed;

/* for ADSP2.8 and above */
struct avcs_svc_api_info {
	uint32_t service_id;
	uint32_t api_version;
	uint32_t api_branch_version;
} __packed;

struct avcs_cmdrsp_get_fwk_version {
	uint32_t build_major_version;
	uint32_t build_minor_version;
	uint32_t build_branch_version;
	uint32_t build_subbranch_version;
	uint32_t num_services;
	struct avcs_svc_api_info svc_api_info[];
} __packed;

struct q6core {
	struct apr_device *adev;
	wait_queue_head_t wait;
	uint32_t avcs_state;
	struct mutex lock;
	struct completion response;
	spinlock_t response_lock;
	u32 pending_command;
	u32 token;
	int response_error;
	uint32_t num_services;
	struct avcs_cmdrsp_get_fwk_version *fwk_version;
	struct avcs_cmdrsp_get_version *svc_version;
	bool fwk_version_supported;
	bool get_state_supported;
	bool get_version_supported;
	bool is_version_requested;
};

static struct q6core *g_core;

/* Only the matching transaction may complete a synchronous query. */
static int q6core_callback(struct apr_device *adev, const struct apr_resp_pkt *data)
{
	struct q6core *core = dev_get_drvdata(&adev->dev);
	const struct apr_hdr *hdr = &data->hdr;
	const struct aprv2_ibasic_rsp_result_t *result = data->payload;
	unsigned long flags;
	u32 command;
	int ret = 0;

	switch (hdr->opcode) {
	case APR_BASIC_RSP_RESULT:
		if (data->payload_size < sizeof(*result))
			return -EINVAL;
		command = result->opcode;
		break;
	case AVCS_CMDRSP_GET_FWK_VERSION:
		command = AVCS_CMD_GET_FWK_VERSION;
		break;
	case AVCS_GET_VERSIONS_RSP:
		command = AVCS_GET_VERSIONS;
		break;
	case AVCS_CMDRSP_ADSP_EVENT_GET_STATE:
		command = AVCS_CMD_ADSP_EVENT_GET_STATE;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&core->response_lock, flags);
	if (command != core->pending_command || hdr->token != core->token) {
		spin_unlock_irqrestore(&core->response_lock, flags);
		return -EINVAL;
	}
	switch (hdr->opcode) {
	case APR_BASIC_RSP_RESULT:
		/* A successful basic acknowledgement is not a version/state reply. */
		if (result->status == ADSP_EOK) {
			spin_unlock_irqrestore(&core->response_lock, flags);
			return 0;
		}
		ret = result->status == ADSP_EUNSUPPORTED ? -ENOTSUPP : -EIO;
		break;
	case AVCS_CMDRSP_GET_FWK_VERSION: {
		const struct avcs_cmdrsp_get_fwk_version *v = data->payload;
		struct avcs_cmdrsp_get_fwk_version *copy;

		if (data->payload_size < sizeof(*v) || !v->num_services ||
		    v->num_services > (data->payload_size - sizeof(*v)) /
				      sizeof(v->svc_api_info[0])) {
			ret = -EBADMSG;
			break;
		}
		copy = kmemdup(v, struct_size(v, svc_api_info, v->num_services),
			       GFP_ATOMIC);
		if (!copy) {
			ret = -ENOMEM;
			break;
		}
		kfree(core->fwk_version);
		core->fwk_version = copy;
		core->fwk_version_supported = true;
		break;
	}
	case AVCS_GET_VERSIONS_RSP: {
		const struct avcs_cmdrsp_get_version *v = data->payload;
		struct avcs_cmdrsp_get_version *copy;

		if (data->payload_size < sizeof(*v) || !v->num_services ||
		    v->num_services > (data->payload_size - sizeof(*v)) /
				      sizeof(v->svc_api_info[0])) {
			ret = -EBADMSG;
			break;
		}
		copy = kmemdup(v, struct_size(v, svc_api_info, v->num_services),
			       GFP_ATOMIC);
		if (!copy) {
			ret = -ENOMEM;
			break;
		}
		kfree(core->svc_version);
		core->svc_version = copy;
		core->get_version_supported = true;
		break;
	}
	case AVCS_CMDRSP_ADSP_EVENT_GET_STATE:
		if (data->payload_size < sizeof(u32)) {
			ret = -EBADMSG;
			break;
		}
		core->get_state_supported = true;
		core->avcs_state = *(const u32 *)data->payload;
		break;
	}
	core->response_error = ret;
	core->pending_command = 0;
	complete(&core->response);
	spin_unlock_irqrestore(&core->response_lock, flags);
	if (of_machine_is_compatible("realme,samurai"))
		trace_samurai_core_transaction(1, command, hdr->token,
			hdr->opcode, data->payload_size,
			hdr->opcode == APR_BASIC_RSP_RESULT ? result->status : 0,
			ret);
	return ret;
}

/* The caller holds core->lock. Tokens separate late and new replies. */
static int q6core_query(struct q6core *core, u32 command)
{
	struct apr_device *adev = core->adev;
	struct apr_pkt pkt = {};
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&core->response_lock, flags);
	reinit_completion(&core->response);
	core->pending_command = command;
	core->response_error = -ETIMEDOUT;
	if (!++core->token)
		++core->token;
	pkt.hdr.token = core->token;
	spin_unlock_irqrestore(&core->response_lock, flags);
	pkt.hdr.hdr_field = APR_HDR_FIELD(APR_MSG_TYPE_SEQ_CMD,
				      APR_HDR_LEN(APR_HDR_SIZE), APR_PKT_VER);
	pkt.hdr.pkt_size = APR_HDR_SIZE;
	pkt.hdr.opcode = command;
	ret = apr_send_pkt(adev, &pkt);
	if (ret >= 0) {
		if (wait_for_completion_timeout(&core->response,
				msecs_to_jiffies(Q6_READY_TIMEOUT_MS)))
			ret = READ_ONCE(core->response_error);
		else
			ret = -ETIMEDOUT;
	}
	spin_lock_irqsave(&core->response_lock, flags);
	core->pending_command = 0;
	spin_unlock_irqrestore(&core->response_lock, flags);
	if (of_machine_is_compatible("realme,samurai"))
		trace_samurai_core_transaction(2, command, pkt.hdr.token,
			0, 0, 0, ret);
	return ret;
}

static int q6core_get_fwk_versions(struct q6core *core)
{
	return q6core_query(core, AVCS_CMD_GET_FWK_VERSION);
}

static int q6core_get_svc_versions(struct q6core *core)
{
	return q6core_query(core, AVCS_GET_VERSIONS);
}

static bool __q6core_is_adsp_ready(struct q6core *core)
{
	int ret = q6core_query(core, AVCS_CMD_ADSP_EVENT_GET_STATE);

	/* Only an explicit unsupported response permits the legacy assumption. */
	return ret == -ENOTSUPP || (!ret && core->avcs_state);
}

/**
 * q6core_get_svc_api_info() - Get version number of a service.
 *
 * @svc_id: service id of the service.
 * @ainfo: Valid struct pointer to fill svc api information.
 *
 * Return: zero on success and error code on failure or unsupported
 */
int q6core_get_svc_api_info(int svc_id, struct q6core_svc_api_info *ainfo)
{
	int i;
	int ret = -ENOTSUPP;

	if (!ainfo)
		return -EINVAL;
	if (!g_core)
		return -EPROBE_DEFER;

	mutex_lock(&g_core->lock);
	if (!g_core->is_version_requested) {
		ret = q6core_get_fwk_versions(g_core);
		if (ret == -ENOTSUPP)
			ret = q6core_get_svc_versions(g_core);
		if (ret) {
			mutex_unlock(&g_core->lock);
			return ret;
		}
		g_core->is_version_requested = true;
	}

	ret = -ENOTSUPP;
	if (g_core->fwk_version_supported) {
		for (i = 0; i < g_core->fwk_version->num_services; i++) {
			struct avcs_svc_api_info *info;

			info = &g_core->fwk_version->svc_api_info[i];
			if (svc_id != info->service_id)
				continue;

			ainfo->api_version = info->api_version;
			ainfo->api_branch_version = info->api_branch_version;
			ret = 0;
			break;
		}
	} else if (g_core->get_version_supported) {
		for (i = 0; i < g_core->svc_version->num_services; i++) {
			struct avcs_svc_info *info;

			info = &g_core->svc_version->svc_api_info[i];
			if (svc_id != info->service_id)
				continue;

			ainfo->api_version = info->version;
			ainfo->api_branch_version = 0;
			ret = 0;
			break;
		}
	}

	mutex_unlock(&g_core->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(q6core_get_svc_api_info);

/**
 * q6core_is_adsp_ready() - Get status of adsp
 *
 * Return: Will be an true if adsp is ready and false if not.
 */
bool q6core_is_adsp_ready(void)
{
	unsigned long  timeout;
	bool ret = false;

	if (!g_core)
		return false;

	mutex_lock(&g_core->lock);
	timeout = jiffies + msecs_to_jiffies(ADSP_STATE_READY_TIMEOUT_MS);
	for (;;) {
		if (__q6core_is_adsp_ready(g_core)) {
			ret = true;
			break;
		}

		msleep(20);
		if (!time_after(timeout, jiffies)) {
			ret = false;
			break;
		}
	}

	mutex_unlock(&g_core->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(q6core_is_adsp_ready);

/* A reply-backed diagnostic for this handset, without opening a PCM route. */
static ssize_t samurai_api_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct q6core_svc_api_info afe = { .api_version = U32_MAX,
					 .api_branch_version = U32_MAX };
	struct q6core_svc_api_info asm_info = { .api_version = U32_MAX,
					      .api_branch_version = U32_MAX };
	int ret;

	ret = q6core_get_svc_api_info(4, &afe);
	if (ret)
		return ret;
	if (afe.api_version == U32_MAX || afe.api_branch_version == U32_MAX)
		return -ETIMEDOUT;
	ret = q6core_get_svc_api_info(7, &asm_info);
	if (ret)
		return ret;
	if (asm_info.api_version == U32_MAX ||
	    asm_info.api_branch_version == U32_MAX)
		return -ETIMEDOUT;

	return sysfs_emit(buf, "native_version_reply service4_api=%u service4_branch=%u service7_api=%u service7_branch=%u\n",
			  afe.api_version, afe.api_branch_version,
			  asm_info.api_version, asm_info.api_branch_version);
}
static DEVICE_ATTR_RO(samurai_api);

static int q6core_probe(struct apr_device *adev)
{
	int ret;

	g_core = kzalloc_obj(*g_core);
	if (!g_core)
		return -ENOMEM;

	dev_set_drvdata(&adev->dev, g_core);

	mutex_init(&g_core->lock);
	g_core->adev = adev;
	init_waitqueue_head(&g_core->wait);
	init_completion(&g_core->response);
	spin_lock_init(&g_core->response_lock);
	if (of_machine_is_compatible("realme,samurai")) {
		ret = device_create_file(&adev->dev, &dev_attr_samurai_api);
		if (ret) {
			kfree(g_core);
			g_core = NULL;
			return ret;
		}
	}
	return 0;
}

static void q6core_exit(struct apr_device *adev)
{
	struct q6core *core = dev_get_drvdata(&adev->dev);

	if (of_machine_is_compatible("realme,samurai"))
		device_remove_file(&adev->dev, &dev_attr_samurai_api);

	if (core->fwk_version_supported)
		kfree(core->fwk_version);
	if (core->get_version_supported)
		kfree(core->svc_version);

	g_core = NULL;
	kfree(core);
}

#ifdef CONFIG_OF
static const struct of_device_id q6core_device_id[]  = {
	{ .compatible = "qcom,q6core" },
	{},
};
MODULE_DEVICE_TABLE(of, q6core_device_id);
#endif

static struct apr_driver qcom_q6core_driver = {
	.probe = q6core_probe,
	.remove = q6core_exit,
	.callback = q6core_callback,
	.driver = {
		.name = "qcom-q6core",
		.of_match_table = of_match_ptr(q6core_device_id),
	},
};

module_apr_driver(qcom_q6core_driver);
MODULE_DESCRIPTION("q6 core");
MODULE_LICENSE("GPL v2");
