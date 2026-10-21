# SPDX-License-Identifier: GPL-2.0-only
# Makefile for Qualcomm CAMSS driver

qcom-camss-objs += \
		camss.o \
		camss-cpas.o \
		camss-csid.o \
		camss-csid-4-1.o \
		camss-csid-4-7.o \
		camss-csid-340.o \
		camss-csid-680.o \
		camss-csid-gen2.o \
		camss-csid-gen3.o \
		camss-csiphy.o \
		camss-csiphy-2ph-1-0.o \
		camss-csiphy-3ph-1-0.o \
		camss-format.o \
		camss-ispif.o \
		camss-tpg.o \
		camss-tpg-gen1.o \
		camss-vfe.o \
		camss-vfe-4-1.o \
		camss-vfe-4-7.o \
		camss-vfe-4-8.o \
		camss-vfe-17x.o \
		camss-vfe-175-camif.o \
		camss-vfe-175-output.o \
		camss-vfe-175-pixel.o \
		camss-vfe-175-lsc.o \
		camss-vfe-175-ccm.o \
		camss-vfe-175-stats.o \
		camss-vfe-175-demosaic.o \
		camss-vfe-175-cst.o \
		camss-vfe-175-round.o \
		camss-vfe-175-gamma.o \
		camss-vfe-175-demux.o \
		camss-vfe-175-mnds.o \
		camss-vfe-175-ds4.o \
		camss-vfe-340.o \
		camss-vfe-480.o \
		camss-vfe-680.o \
		camss-vfe-690.o \
		camss-vfe-780.o \
		camss-vfe-880-pp.o \
		camss-vfe-gen1.o \
		camss-vfe-vbif.o \
		camss-video.o

obj-$(CONFIG_VIDEO_QCOM_CAMSS) += qcom-camss.o

qcom-camss-icp-a5-core-y := camss-icp-a5-core.o camss-icp-a5-hfi.o camss-icp-a5-fw.o camss-icp-a5-memory.o
obj-$(CONFIG_VIDEO_QCOM_CAMSS_ICP_A5_CORE) += qcom-camss-icp-a5-core.o
obj-$(CONFIG_VIDEO_QCOM_CAMSS_ICP_A5_MEMORY_TEST) += camss-icp-a5-memory-test.o
CFLAGS_camss-icp-a5-memory-test.o += -I$(obj)

qcom-camss-icp-a5-core-y += camss-icp-a5-resources.o
obj-$(CONFIG_VIDEO_QCOM_CAMSS_ICP_A5_POWER_TEST) += camss-icp-a5-power-test.o

qcom-camss-icp-a5-core-y += camss-icp-a5-boot.o camss-icp-a5-ipe.o
obj-$(CONFIG_VIDEO_QCOM_CAMSS_ICP_A5_BOOT_TEST) += camss-icp-a5-boot-test.o

obj-$(CONFIG_VIDEO_QCOM_CAMSS_ICP_A5_M2M) += camss-icp-a5-m2m.o

qcom-camss-icp-a5-core-y += camss-icp-a5-dma.o
qcom-camss-icp-a5-core-y += camss-icp-a5-colour.o

qcom-camss-$(CONFIG_VIDEO_QCOM_CAMSS_ICP) += camss-icp-platform.o
