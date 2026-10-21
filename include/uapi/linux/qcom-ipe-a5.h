/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_QCOM_IPE_A5_H
#define _UAPI_QCOM_IPE_A5_H
#include <linux/types.h>
#include <linux/v4l2-controls.h>

/* Experimental local control allocation; not an upstream-reserved CID. */
#define V4L2_CID_QCOM_IPE_A5_FRAME_PARAMS (V4L2_CID_USER_BASE + 0x1240)
#define QCOM_IPE_A5_PARAMS_VERSION 1U
#define QCOM_IPE_A5_GAMMA 1U
#define QCOM_IPE_A5_LTM 2U
#define QCOM_IPE_A5_COLOUR_CST 1U
#define QCOM_IPE_A5_COLOUR_CC 2U
#define QCOM_IPE_A5_COLOUR_CV 4U

struct qcom_ipe_a5_cst {
	__s16 matrix[9], subtract[3];
	__u16 offset[3], minimum[3], maximum[3];
};
struct qcom_ipe_a5_cc {
	__s16 matrix[9], offset[3];
	__u32 shift;
};
struct qcom_ipe_a5_cv {
	__s16 luma[3], y_offset, chroma[8], offset[2];
};
struct qcom_ipe_a5_colour {
	__u32 modules;
	struct qcom_ipe_a5_cst cst;
	/* Explicit alignment bytes: callers must zero padding. */
	__u16 padding;
	struct qcom_ipe_a5_cc cc;
	struct qcom_ipe_a5_cv cv;
};
struct qcom_ipe_a5_gamma {
	__u16 base[3][256];
	__s16 delta[3][256];
};
struct qcom_ipe_a5_ltm_unsigned { __u16 base; __s16 delta; };
struct qcom_ipe_a5_ltm_signed { __s16 base, delta; };
struct qcom_ipe_a5_ltm_iq {
	__u32 reverse_gamma, local_adaptation, mask_filter, bin_init_count;
	__u32 rgb2y[4], lce_threshold, y_ratio_max, mask_kernel[6];
	__u32 mask_shift, mask_scale, horizontal_termination, vertical_termination;
	__u32 data_collection, image_processing;
};
struct qcom_ipe_a5_ltm {
	struct qcom_ipe_a5_ltm_iq iq;
	__u16 weight[12];
	struct qcom_ipe_a5_ltm_unsigned la[2][64], curve[64];
	struct qcom_ipe_a5_ltm_signed scale[64];
	struct qcom_ipe_a5_ltm_unsigned mask[64];
	struct qcom_ipe_a5_ltm_signed lce[2][16];
	struct qcom_ipe_a5_ltm_unsigned reverse_gamma[5][64];
};
/* A compound U8 control stored in a Media Request with one OUTPUT buffer.
 * All-zero payload means bypass. Otherwise version1 and size6008 are required.
 * Values are native-endian fixed-width quantized fields. No pointers, device
 * addresses, commands or arbitrary registers are accepted. CAPTURE buffers
 * are queued normally. Direct non-request buffers always retain bypass.
 */
struct qcom_ipe_a5_frame_params {
	__u32 version, size, flags, reserved[5];
	struct qcom_ipe_a5_colour colour;
	struct qcom_ipe_a5_gamma gamma;
	struct qcom_ipe_a5_ltm ltm;
};
#endif
