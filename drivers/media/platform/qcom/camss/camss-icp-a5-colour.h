/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_A5_IPE_COLOUR_H
#define CAMSS_A5_IPE_COLOUR_H

#include <linux/types.h>

#define CAMSS_A5_IPE_CC_COMMAND_BYTES 44
#define CAMSS_A5_IPE_CC_ARRAY_OFFSET 2592U
#define CAMSS_A5_IPE_CC_POST_OFFSET 32U
#define CAMSS_A5_IPE_CC_PACKET_OFFSET 2656U
#define CAMSS_A5_IPE_CC_IQ_BYTES 2700U

/* RGB row-major matrix at Q(7 + shift); offsets are RGB signed 11-bit integers.
 * The target IPE caller uses HAL output type 2 (type 1 has 13-bit offsets). */
struct camss_a5_ipe_cc {
	s16 matrix[9];
	s16 offset[3];
	u32 shift;
};

int camss_a5_ipe_cc_encode(void *destination, u32 bytes,
			 const struct camss_a5_ipe_cc *cc);

/* IQ and both program arrays live in one caller-owned, mapped request page.
 * Every invalid argument leaves the destination unchanged. No arbitrary
 * command, register address or image pointer is accepted.
 */
int camss_a5_ipe_encode_cc_iq(void *destination, u32 bytes, u32 iq_iova,
			    const struct camss_a5_ipe_cc *cc);

/* Quantized target IPE component transform; matrix Q10, signed13.
 * Subtract is signed11; additive offsets and clamps are unsigned10.
 */
struct camss_a5_ipe_cst {
	s16 matrix[9];
	s16 subtract[3];
	u16 offset[3];
	u16 minimum[3];
	u16 maximum[3];
};

/* Target native CV leaf order, verified through its deserializer and Execute.
 * Coefficients are signed12/Q8; offsets have signed11 numerical range.
 * Chroma pairs are ap/am, bp/bm, cp/cm, dp/dm; cb/cr offsets follow packet order.
 */
struct camss_a5_ipe_cv {
	s16 luma[3];
	s16 y_offset;
	s16 chroma[8];
	s16 offset[2];
};

#define CAMSS_A5_IPE_COLOUR_CST 1U
#define CAMSS_A5_IPE_COLOUR_CC 2U
#define CAMSS_A5_IPE_COLOUR_CV 4U
#define CAMSS_A5_IPE_COLOUR_MODULES 7U

struct camss_a5_ipe_colour {
	u32 modules;
	struct camss_a5_ipe_cst cst;
	struct camss_a5_ipe_cc cc;
	struct camss_a5_ipe_cv cv;
};

#define CAMSS_A5_IPE_COLOUR_ARRAY_OFFSET 2592U
#define CAMSS_A5_IPE_COLOUR_PRE_OFFSET 32U
#define CAMSS_A5_IPE_COLOUR_POST_OFFSET 64U
#define CAMSS_A5_IPE_CST_PACKET_OFFSET 2720U
#define CAMSS_A5_IPE_COLOUR_CC_PACKET_OFFSET 2776U
#define CAMSS_A5_IPE_CV_PACKET_OFFSET 2820U
#define CAMSS_A5_IPE_COLOUR_IQ_BYTES 2860U

int camss_a5_ipe_colour_check(const struct camss_a5_ipe_colour *colour);
int camss_a5_ipe_encode_colour_iq(void *destination, u32 bytes, u32 iq_iova,
				const struct camss_a5_ipe_colour *colour);

/* Target LUT selector order is G, B, R. Values are quantized Gamma15
 * base10 and signed delta10, not arbitrary CDM words or device addresses.
 */
struct camss_a5_ipe_gamma {
	u16 base[3][256];
	s16 delta[3][256];
};

#define CAMSS_A5_IPE_PROGRAM_GENERIC 20U
#define CAMSS_A5_IPE_PROGRAM_GAMMA 12U
#define CAMSS_A5_IPE_GAMMA_CST_PACKET_OFFSET 2752U
#define CAMSS_A5_IPE_GAMMA_CC_PACKET_OFFSET 2808U
#define CAMSS_A5_IPE_GAMMA_CV_PACKET_OFFSET 2852U
#define CAMSS_A5_IPE_GAMMA_DMI_OFFSET 2896U
#define CAMSS_A5_IPE_GAMMA_LUT_OFFSET 2944U
#define CAMSS_A5_IPE_GAMMA_IQ_BYTES 6016U

int camss_a5_ipe_gamma_check(const struct camss_a5_ipe_gamma *gamma);
int camss_a5_ipe_encode_gamma_iq(void *destination, u32 bytes, u32 iq_iova,
			       const struct camss_a5_ipe_colour *colour,
			       const struct camss_a5_ipe_gamma *gamma);

#endif
