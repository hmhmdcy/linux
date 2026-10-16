// SPDX-License-Identifier: GPL-2.0-only
/* SM8150 CPAS: own active camera_8150 v175_100/101 IFE QoS contract.
 * Native register tables Copyright (c) 2018 The Linux Foundation.
 */
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <dt-bindings/power/qcom,rpmhpd.h>
#include "camss.h"
#include "camss-cpas.h"

#define SM8150_CPAS_CLOCKS 7

struct camss_cpas {
	struct device *dev;
	struct device *mmcx;
	struct device_link *mmcx_link;
	bool performance_requested;
	void __iomem *top;
	void __iomem *noc;
	struct clk_bulk_data clocks[SM8150_CPAS_CLOCKS];
	bool enabled;
};

struct camss_cpas_reg {
	u32 offset;
	u32 mask;
	u32 value;
};

static const struct camss_cpas_reg sm8150_ife_qos[] = {
	/* CAM_CAMNOC_IFE02 */
	{ 0x0430, 0xffffffff, 0x66666543 },
	{ 0x0434, 0xffffffff, 0x66666666 },
	{ 0x0438, 0x00000070, 0x00000030 },
	{ 0x0440, 0xffffffff, 0xffffff00 },
	{ 0x0448, 0xffffffff, 0x00000001 },
	/* CAM_CAMNOC_IFE13 */
	{ 0x0830, 0xffffffff, 0x66666543 },
	{ 0x0834, 0xffffffff, 0x66666666 },
	{ 0x0838, 0x00000070, 0x00000030 },
	{ 0x0840, 0xffffffff, 0xffffff00 },
	{ 0x0848, 0xffffffff, 0x00000001 },
};

static int camss_sm8150_check_iova(struct device *dev)
{
	struct iommu_domain *domain = iommu_get_domain_for_dev(dev);
	struct iommu_resv_region *region;
	bool low = false, high = false;
	LIST_HEAD(regions);

	if (!domain || (domain->type != IOMMU_DOMAIN_DMA &&
			domain->type != IOMMU_DOMAIN_DMA_FQ))
		return -ENODEV;
	iommu_get_resv_regions(dev, &regions);
	list_for_each_entry(region, &regions, list) {
		if (region->type != IOMMU_RESV_RESERVED)
			continue;
		if (region->start == 0 && region->length == 0x07400000)
			low = true;
		if (region->start == 0xe0000000 && region->length == 0x20000000)
			high = true;
	}
	iommu_put_resv_regions(dev, &regions);
	/* Do not replace the safe mask without both reservation callback policies. */
	return low && high ? 0 : -EINVAL;
}

static int camss_cpas_stop(struct camss_cpas *cpas)
{
	int ret;

	if (cpas->performance_requested) {
		ret = dev_pm_genpd_set_performance_state(cpas->mmcx, 0);
		if (ret)
			return ret;
		cpas->performance_requested = false;
	}
	if (cpas->enabled) {
		clk_bulk_disable_unprepare(ARRAY_SIZE(cpas->clocks), cpas->clocks);
		cpas->enabled = false;
	}
	return 0;
}

static void camss_cpas_release(void *data)
{
	struct camss_cpas *cpas = data;
	int ret;

	ret = camss_cpas_stop(cpas);
	if (ret) {
		dev_err(cpas->dev, "failed to release CAMSS MMCX vote: %d\n", ret);
		if (cpas->enabled)
			clk_bulk_disable_unprepare(ARRAY_SIZE(cpas->clocks), cpas->clocks);
		cpas->enabled = false;
	}
	device_link_del(cpas->mmcx_link);
	dev_pm_domain_detach(cpas->mmcx, true);
}

int camss_cpas_init(struct camss *camss)
{
	static const char * const names[] = {
		"gcc_camera_ahb", "gcc_camera_axi", "gcc_camera_sf_axi",
		"slow_ahb_src", "cpas_ahb", "camnoc_axi_src", "camnoc_axi"
	};
	struct device *dev = camss->dev;
	struct platform_device *pdev = to_platform_device(dev);
	struct resource *res;
	struct camss_cpas *cpas;
	int i, ret;

	if (camss->res->version != CAMSS_8150)
		return 0;
	ret = camss_sm8150_check_iova(dev);
	if (ret)
		return dev_err_probe(dev, ret, "native IFE IOVA reservations missing\n");
	cpas = devm_kzalloc(dev, sizeof(*cpas), GFP_KERNEL);
	if (!cpas)
		return -ENOMEM;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "cpas_top");
	if (!res || resource_size(res) != 0x1000)
		return -EINVAL;
	cpas->top = devm_ioremap_resource(dev, res);
	if (IS_ERR(cpas->top))
		return PTR_ERR(cpas->top);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "camnoc");
	if (!res || resource_size(res) != 0x5000)
		return -EINVAL;
	cpas->noc = devm_ioremap_resource(dev, res);
	if (IS_ERR(cpas->noc))
		return PTR_ERR(cpas->noc);
	for (i = 0; i < ARRAY_SIZE(names); i++)
		cpas->clocks[i].id = names[i];
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(cpas->clocks), cpas->clocks);
	if (ret)
		return ret;
	cpas->dev = dev;
	cpas->mmcx = dev_pm_domain_attach_by_name(dev, "mmcx");
	if (IS_ERR_OR_NULL(cpas->mmcx))
		return dev_err_probe(dev, cpas->mmcx ? PTR_ERR(cpas->mmcx) : -ENODEV,
				     "MMCX power domain is required for camera clocks\n");
	cpas->mmcx_link = device_link_add(dev, cpas->mmcx,
					  DL_FLAG_STATELESS | DL_FLAG_PM_RUNTIME |
					  DL_FLAG_RPM_ACTIVE);
	if (!cpas->mmcx_link) {
		dev_pm_domain_detach(cpas->mmcx, true);
		return -EINVAL;
	}
	ret = devm_add_action_or_reset(dev, camss_cpas_release, cpas);
	if (ret)
		return ret;
	camss->cpas = cpas;
	return 0;
}

int camss_cpas_resume(struct camss *camss)
{
	struct camss_cpas *cpas = camss->cpas;
	u32 camera, version, value;
	int i, ret;

	if (camss->res->version != CAMSS_8150)
		return 0;
	if (!cpas)
		return -ENODEV;
	/* The resource tables may select 950 MHz: vote before any clock change. */
	ret = dev_pm_genpd_set_performance_state(cpas->mmcx, RPMH_REGULATOR_LEVEL_TURBO);
	if (ret)
		return ret;
	cpas->performance_requested = true;
	ret = clk_set_rate(cpas->clocks[3].clk, 80000000);
	if (ret)
		goto reset_performance;
	/* Native CPAS minimum bandwidth vote: 3GB/s, 32-byte bus, margin. */
	ret = clk_set_rate(cpas->clocks[5].clk, 150000000);
	if (ret)
		goto reset_performance;
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(cpas->clocks), cpas->clocks);
	if (ret)
		goto reset_performance;
	/* Top domain and interconnects are active before these ordered reads. */
	camera = readl(cpas->top) & 0xffffff;
	version = readl(cpas->top + 4);
	if (camera != 0x010705 || (version != 0x10000000 && version != 0x10000001)) {
		dev_err(camss->dev, "unsupported CPAS camera %#x revision %#x\n",
			camera, version);
		ret = -EOPNOTSUPP;
		goto disable_clocks;
	}
	for (i = 0; i < ARRAY_SIZE(sm8150_ife_qos); i++) {
		const struct camss_cpas_reg *reg = &sm8150_ife_qos[i];

		value = reg->value;
		if (reg->mask != U32_MAX)
			value |= readl(cpas->noc + reg->offset) & ~reg->mask;
		writel(value, cpas->noc + reg->offset);
		if ((readl(cpas->noc + reg->offset) & reg->mask) != reg->value) {
			ret = -EIO;
			goto disable_clocks;
		}
	}
	cpas->enabled = true;
	return 0;

disable_clocks:
	clk_bulk_disable_unprepare(ARRAY_SIZE(cpas->clocks), cpas->clocks);
reset_performance:
	if (dev_pm_genpd_set_performance_state(cpas->mmcx, 0))
		dev_err(camss->dev, "failed to unwind CAMSS MMCX vote\n");
	else
		cpas->performance_requested = false;
	return ret;
}

int camss_cpas_suspend(struct camss *camss)
{
	return camss->cpas ? camss_cpas_stop(camss->cpas) : 0;
}
