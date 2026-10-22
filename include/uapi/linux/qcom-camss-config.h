/* SPDX-License-Identifier: GPL-2.0-only WITH Linux-syscall-note */
/*
 * Qualcomm CAMSS ISP Driver - Userspace API
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#ifndef _UAPI_LINUX_QCOM_CAMSS_CONFIG_H
#define _UAPI_LINUX_QCOM_CAMSS_CONFIG_H

#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/media/v4l2-isp.h>

/**
 * enum camss_params_block_type - CAMSS ISP parameter block identifiers
 *
 * Each value identifies one ISP processing block.  The value is placed in
 * the @type field of &struct v4l2_isp_params_block_header.
 *
 * @CAMSS_PARAMS_WB_GAIN: white balance gains and offsets (CLC_WB),
 *	&struct camss_params_wb_gain
 * @CAMSS_PARAMS_CHROMA_ENHAN: RGB to YUV colour transfer matrix
 *	(CLC_CHROMA_ENHAN), &struct camss_params_chroma_enhan
 * @CAMSS_PARAMS_COLOR_CORRECT: colour correction matrix (CLC_CC),
 *	&struct camss_params_color_correct
 * @CAMSS_PARAMS_GAMMA: per-channel gamma correction curves (CLC_GLUT),
 *	&struct camss_params_gamma
 * @CAMSS_PARAMS_LSC: regular lens shading mesh, &struct camss_params_lsc
 * @CAMSS_PARAMS_IFE_GAMMA: segmented IFE RGB gamma,
 *	&struct camss_params_ife_gamma
 * @CAMSS_PARAMS_AEC_BHIST: region of the AEC Bayer histogram statistics,
 *	&struct camss_params_aec_bhist
 */
enum camss_params_block_type {
	CAMSS_PARAMS_WB_GAIN = 1,
	CAMSS_PARAMS_CHROMA_ENHAN = 2,
	CAMSS_PARAMS_COLOR_CORRECT = 3,
	CAMSS_PARAMS_GAMMA = 4,
	CAMSS_PARAMS_AEC_BHIST = 5,
	CAMSS_PARAMS_LSC = 6,
	CAMSS_PARAMS_IFE_GAMMA = 7,
	CAMSS_PARAMS_IFE_DS4 = 8,
	CAMSS_PARAMS_IFE_GTM = 9,
};

/* Local experimental GTM parameter/statistics types, not reserved upstream.
 * Flags must be zero; disabling also requires the full payload and cookie.
 * cookie is a caller token, not proof of application. Statistics acknowledge
 * it only after PIX REG_UPDATE ACK and snapshot it at the buffer's SOF.
 * base is unsigned Q12 (18 bits), slope is signed Q20 (26 bits).
 * CPU DMI uses canonical low/high words, without the HW-CDM word swap.
 */
#define CAMSS_IFE_GTM_ENTRIES 64
struct camss_ife_gtm_segment {
	__u32 base;
	__s32 slope;
};
struct camss_params_ife_gtm {
	struct v4l2_isp_block_header header;
	__u64 cookie;
	__u32 enabled;
	__u32 reserved;
	struct camss_ife_gtm_segment segments[CAMSS_IFE_GTM_ENTRIES];
} __attribute__((aligned(8)));

#define CAMSS_IFE_GTM_STATUS_ACKED (1U << 0)
#define CAMSS_IFE_GTM_STATUS_ENABLED (1U << 1)
struct camss_stats_ife_gtm {
	struct v4l2_isp_block_header header;
	__u64 cookie;
	__u64 generation;
	__u32 status;
	__u32 bank;
} __attribute__((aligned(8)));

/* Local experimental identifier, not reserved upstream. Single-plane PD10:
 * each 2x2 YUV420 block occupies eight bytes. SM8150 rows are aligned to
 * 192 bytes and the packed row count to four; bpl describes packed rows. */
#define V4L2_PIX_FMT_QCOM_PD10_LOCAL v4l2_fourcc('Q', 'P', '1', '0')

/* DS4 configuration only. Flags and reserved must be zero. Stream lifetime
 * controls the hardware enable; coefficients come from selected tuning. */
struct camss_params_ife_ds4 {
	struct v4l2_isp_block_header header;
	__u16 coefficient[3];
	__u16 reserved;
} __attribute__((aligned(8)));

/* Number of entries in each gamma channel LUT. */
#define CAMSS_GAMMA_LUT_SIZE	256

/**
 * struct camss_params_wb_gain - White Balance gains
 *
 * Implements the CLC_WB pipeline module.  The pipeline applies three
 * sequential operations per channel:
 *   1. Subtract sub-offset (black-level subtraction)
 *   2. Multiply by gain    (colour balance)
 *   3. Add add-offset      (output pedestal)
 *
 * Gains are 15uQ10 (15-bit unsigned, 10 fractional bits). Offsets
 * are 16-bit unsigned, normalised to full input scale (65535 = 1.0)
 *
 * @header:   block header; @header.type = CAMSS_PARAMS_WB_GAIN
 * @g_gain:   green channel gain (15uQ10, 1024 = 1.0)
 * @b_gain:   blue  channel gain (15uQ10, 1024 = 1.0)
 * @r_gain:   red   channel gain (15uQ10, 1024 = 1.0)
 * @g_sub:    green sub-offset, subtracted before gain (16u)
 * @b_sub:    blue  sub-offset, subtracted before gain (16u)
 * @r_sub:    red   sub-offset, subtracted before gain (16u)
 * @g_add:    green add-offset, added after gain (16u)
 * @b_add:    blue  add-offset, added after gain (16u)
 * @r_add:    red   add-offset, added after gain (16u)
 * @_pad:     padding for 64-bit alignment, must be zero
 */
struct camss_params_wb_gain {
	struct v4l2_isp_params_block_header header;
	__u16 g_gain;
	__u16 b_gain;
	__u16 r_gain;
	__u16 g_sub;
	__u16 b_sub;
	__u16 r_sub;
	__u16 g_add;
	__u16 b_add;
	__u16 r_add;
	__u16 _pad[3];
} __attribute__((aligned(8)));

/**
 * struct camss_params_chroma_enhan - RGB to YUV colour transfer matrix
 *
 * Implements the CLC_CHROMA_ENHAN pipeline module. All coefficients are
 * signed 12-bit fixed-point Q3.8 (range roughly -8.0 to +7.996).
 *
 * RGB2Y - Luma (Y) coefficients
 * Y = v0 * R + v1 * G + v2 * B
 *
 * @luma_v0:  R-to-Y coefficient (12sQ8)
 * @luma_v1:  G-to-Y coefficient (12sQ8)
 * @luma_v2:  B-to-Y coefficient (12sQ8)
 * @luma_k:   Y output offset    (9s,  0 = no offset)
 *
 * RGB2Cb - Chroma (Cb) coefficients
 * Cb = a x ((B - G) + b(R - G)) + KCb
 * with:
 *   a = ap, when (B-G) + b(R-G) > 0; a = am, when (B-G) + b(R-G) ≤ 0;
 *   b = bp when (R-G) > 0; b = bm when (R-G) ≤ 0
 *
 * @coeff_ap: Cb positive coefficient (12sQ8)
 * @coeff_am: Cb negative coefficient (12sQ8)
 * @coeff_bp: Cb positive coefficient (12sQ8)
 * @coeff_bm: Cb negative coefficient (12sQ8)
 * @kcb:      Cb output offset        (11s)
 *
 * RGB2Cr - Chroma (Cr) coefficients:
 * Cr = c x ((R - G) + d(B - G)) + KCr
 * with:
 *   c = cp, when (R-G) + d(B-G) > 0; c = cm, when (R-G) + d(B-G) ≤ 0
 *   d = dp when (B-G) > 0; d = dm when (B-G) ≤ 0
 *
 * @coeff_cp: Cr positive coefficient (12sQ8)
 * @coeff_cm: Cr negative coefficient (12sQ8)
 * @coeff_dp: Cr positive coefficient (12sQ8)
 * @coeff_dm: Cr negative coefficient (12sQ8)
 * @kcr:      Cr output offset        (11s)
 *
 * @header: generic block header; @header.type = CAMSS_PARAMS_CHROMA_ENHAN
 * @_pad:   padding for 64-bit alignment, must be zero
 */
struct camss_params_chroma_enhan {
	struct v4l2_isp_params_block_header header;
	__u16 luma_v0;
	__u16 luma_v1;
	__u16 luma_v2;
	__u16 luma_k;
	__u16 coeff_ap;
	__u16 coeff_am;
	__u16 coeff_bp;
	__u16 coeff_bm;
	__u16 coeff_cp;
	__u16 coeff_cm;
	__u16 coeff_dp;
	__u16 coeff_dm;
	__u16 kcb;
	__u16 kcr;
	__u16 _pad[2];
} __attribute__((aligned(8)));

/**
 * struct camss_params_color_correct - colour correction matrix
 *
 * Implements the CLC_CC pipeline module.  The matrix computes:
 *   Out_ch0 (G) = a0*G + b0*B + c0*R + k0
 *   Out_ch1 (B) = a1*G + b1*B + c1*R + k1
 *   Out_ch2 (R) = a2*G + b2*B + c2*R + k2
 *
 * @header:  block header; @header.type = CAMSS_PARAMS_COLOR_CORRECT
 * @a:       G-input coefficients per output channel (12s;
 *           a[0]=Out_G, a[1]=Out_B, a[2]=Out_R)
 * @b:       B-input coefficients (12s)
 * @c:       R-input coefficients (12s)
 * @k:       per-output-channel offsets (VFE175 uses 11 signed bits)
 * @qfactor: Q-format selector (2u):
 *           0 = 12sQ7  (range ~-16.0 .. +15.992)
 *           1 = 12sQ8  (range  ~-8.0 ..  +7.996)
 *           2 = 12sQ9  (range  ~-4.0 ..  +3.998)
 *           3 = 12sQ10 (range  ~-2.0 ..  +1.999)
 * @_pad:    padding for 64-bit alignment, must be zero
 */
struct camss_params_color_correct {
	struct v4l2_isp_params_block_header header;
	__u16 a[3];
	__u16 b[3];
	__u16 c[3];
	__u16 k[3];
	__u16 qfactor;
	__u16 _pad[3];
} __attribute__((aligned(8)));

/**
 * struct camss_params_gamma - per-channel gamma correction curves
 *
 * Implements the CLC_GLUT pipeline module, applied in the RGB domain.  It
 * holds one independent lookup table per colour channel. Each table is a
 * direct (not segmented) map of input level to output level.
 *
 * Each table has @CAMSS_GAMMA_LUT_SIZE (256) entries of 16-bit unsigned
 * output. The number of significant output bits follows the width of the
 * engine's internal bus; the upper bits are ignored by the hardware.
 *
 * The module maps a 12-bit input to an 8-bit output: the top 8 input bits
 * index the table and the low 4 bits are used to linearly interpolate
 * between adjacent entries. Entry i therefore represents the output for
 * input level i/255 of full scale.
 * Output range: 0x00 = black, 0xFF = white.
 *
 * How to fill a curve (same on all three channels for pure luminance
 * gamma; different curves per channel additionally shift colour balance):
 *
 *   Identity (pass-through, gamma 1.0):
 *     lut[i] = i;                                // i = 0..255
 *
 *   Encode with gamma g (e.g. sRGB-like, g = 2.2):
 *     lut[i] = round(pow(i / 255.0, 1.0 / g) * 255.0);
 *
 * @header:  block header; @header.type = CAMSS_PARAMS_GAMMA
 * @glut:    green channel gamma curve
 * @blut:    blue  channel gamma curve
 * @rlut:    red   channel gamma curve
 */
struct camss_params_gamma {
	struct v4l2_isp_params_block_header header;
	__u16 glut[CAMSS_GAMMA_LUT_SIZE];
	__u16 blut[CAMSS_GAMMA_LUT_SIZE];
	__u16 rlut[CAMSS_GAMMA_LUT_SIZE];
} __attribute__((aligned(8)));

#define CAMSS_PARAMS_OPE_MAX_PAYLOAD		\
	(sizeof(struct camss_params_wb_gain)	+\
	 sizeof(struct camss_params_chroma_enhan)	+\
	 sizeof(struct camss_params_color_correct)	+\
	 sizeof(struct camss_params_gamma))

/**
 * struct camss_params_aec_bhist - AEC Bayer histogram configuration
 *
 * Selects the region of the frame the AEC Bayer histogram,
 * &struct camss_stats_bhist, counts. The region is given in pixels of the
 * Bayer frame entering the ISP; every value must be even, as the histogram
 * counts 2x2 Bayer quads. A region reaching past the frame is clipped to it.
 *
 * @header: block header; @header.type = CAMSS_PARAMS_AEC_BHIST
 * @left:   horizontal offset of the region
 * @top:    vertical offset of the region
 * @width:  width of the region, at least 2
 * @height: height of the region, at least 2
 */
struct camss_params_aec_bhist {
	struct v4l2_isp_params_block_header header;
	__u16 left;
	__u16 top;
	__u16 width;
	__u16 height;
} __attribute__((aligned(8)));

#define CAMSS_PARAMS_IFE_MAX_PAYLOAD		\
	(sizeof(struct camss_params_aec_bhist) +	\
	 sizeof(struct camss_params_wb_gain))

/* Fixed mesh, expressed in Bayer input coordinates before image scaling. */
#define CAMSS_LSC_MESH_WIDTH	17
#define CAMSS_LSC_MESH_HEIGHT	13
#define CAMSS_LSC_MESH_POINTS	(CAMSS_LSC_MESH_WIDTH * CAMSS_LSC_MESH_HEIGHT)
#define CAMSS_LSC_CHANNELS	4

/**
 * struct camss_params_lsc - Lens shading correction mesh
 *
 * @header: block header; type is CAMSS_PARAMS_LSC
 * @cell_width: horizontal mesh cell spacing, in 2-pixel Bayer quads
 * @cell_height: vertical mesh cell spacing, in 2-line Bayer quads
 * @x_start: first input quad's horizontal position relative to mesh node (0,0)
 * @y_start: first input quad's vertical position relative to mesh node (0,0)
 * @subdivision: log2 of the number of interpolation subgrids per cell (0..3)
 * @_pad: reserved; must be zero
 * @gains: row-major R, GR, GB, B gains, Q10 in [1024, 8191] (1024 is unity)
 *
 * The last node is at ((WIDTH-1)*cell_width, (HEIGHT-1)*cell_height).
 * The mesh must cover the active Bayer sink image. Width and height must
 * be divisible by 2^subdivision; hardware interpolation subgrids have
 * minimum dimensions of 9 quads. Calibration and crop/mode resampling
 * are userspace operations; these fields describe the resulting mesh.
 */
struct camss_params_lsc {
	struct v4l2_isp_params_block_header header;
	__u16 cell_width;
	__u16 cell_height;
	__u16 x_start;
	__u16 y_start;
	__u8 subdivision;
	__u8 _pad[7];
	__u16 gains[CAMSS_LSC_CHANNELS][CAMSS_LSC_MESH_POINTS];
} __attribute__((aligned(8)));

#define CAMSS_PARAMS_IFE_LSC_MAX_PAYLOAD \
	(CAMSS_PARAMS_IFE_MAX_PAYLOAD + sizeof(struct camss_params_lsc))

#define CAMSS_PARAMS_IFE_COLOR_MAX_PAYLOAD \
	(CAMSS_PARAMS_IFE_LSC_MAX_PAYLOAD + sizeof(struct camss_params_color_correct))

/* IFE Gamma uses uniformly spaced segments, unlike the OPE direct LUT. */
#define CAMSS_IFE_GAMMA_CHANNELS 3
#define CAMSS_IFE_GAMMA_SEGMENTS 64

/**
 * struct camss_ife_gamma_segment - one linear segment in a 10-bit output domain
 * @base: output at the segment's lower endpoint, 0 through 1023
 * @delta: signed output change over the segment, -512 through 511
 *
 * Each segment covers 1/64 of the full input range. The last delta describes
 * the change toward the full-scale endpoint, which may reach 1024. Segments
 * are independent: no continuity or monotonicity is imposed by this ABI.
 */
struct camss_ife_gamma_segment {
	__u16 base;
	__s16 delta;
};

/**
 * struct camss_params_ife_gamma - segmented RGB gamma curves for IFE
 * @header: block header; type is CAMSS_PARAMS_IFE_GAMMA
 * @segments: 64 segments per channel, in R, G, B channel order
 *
 * ENABLE uploads and enables. With neither flag, upload retains the current
 * enable state. DISABLE accepts a header-only block and leaves stored curves
 * intact. Reopening a stream restores its format's initial transfer curve.
 */
struct camss_params_ife_gamma {
	struct v4l2_isp_params_block_header header;
	struct camss_ife_gamma_segment
		segments[CAMSS_IFE_GAMMA_CHANNELS][CAMSS_IFE_GAMMA_SEGMENTS];
} __attribute__((aligned(8)));

#define CAMSS_PARAMS_IFE_GAMMA_MAX_PAYLOAD \
	(CAMSS_PARAMS_IFE_COLOR_MAX_PAYLOAD + sizeof(struct camss_params_ife_gamma))


/**
 * enum camss_stats_block_type - CAMSS ISP statistics block identifiers
 *
 * Each value identifies one statistics block in a
 * V4L2_META_FMT_QCOM_ISP_STATS buffer. The value is placed in the @type field
 * of &struct v4l2_isp_block_header.
 *
 * @CAMSS_STATS_AEC_BHIST: AEC Bayer histogram, &struct camss_stats_bhist
 * @CAMSS_STATS_AWB_BG: Bayer RGB grid, &struct camss_stats_awb_bg
 */
enum camss_stats_block_type {
	CAMSS_STATS_AEC_BHIST = 1,
	CAMSS_STATS_AWB_BG = 2,
	CAMSS_STATS_IFE_GTM = 3,
};

/* Number of bins in a Bayer histogram. */
#define CAMSS_STATS_BHIST_BINS	1024

/**
 * struct camss_stats_bhist - Bayer histogram
 *
 * Histogram of the luma of each 2x2 Bayer quad inside the histogram's region,
 * over CAMSS_STATS_BHIST_BINS equal bins spanning its input range. Quads
 * whose luma falls outside the input range are not counted.
 *
 * @header: block header; @header.type = CAMSS_STATS_AEC_BHIST
 * @bins:   number of quads per bin
 */
struct camss_stats_bhist {
	struct v4l2_isp_block_header header;
	__u32 bins[CAMSS_STATS_BHIST_BINS];
} __attribute__((aligned(8)));

/**
 * struct camss_stats_bg_region - Bayer channel sums and sample counts
 *
 * Each channel sum includes only samples admitted by the statistics threshold.
 * Counts are per Bayer channel, not the number of pixels in the region.
 *
 * @r_sum: red sum
 * @b_sum: blue sum
 * @gr_sum: green-on-red-row sum
 * @gb_sum: green-on-blue-row sum
 * @r_count: red sample count
 * @b_count: blue sample count
 * @gr_count: green-on-red-row sample count
 * @gb_count: green-on-blue-row sample count
 */
struct camss_stats_bg_region {
	__u32 r_sum;
	__u32 b_sum;
	__u32 gr_sum;
	__u32 gb_sum;
	__u16 r_count;
	__u16 b_count;
	__u16 gr_count;
	__u16 gb_count;
};

/**
 * struct camss_stats_awb_bg - Bayer RGB grid statistics for one input frame
 *
 * The driver selects an even, centred grid in the input Bayer frame.
 * Regions are ordered left to right, top to bottom. All dimensions and
 * offsets refer to input pixels before image scaling. The sum/count ratio
 * gives the channel mean in the scale described by @input_bits.
 *
 * @header: block header; @header.type = CAMSS_STATS_AWB_BG
 * @left: horizontal grid offset
 * @top: vertical grid offset
 * @region_width: width of each region
 * @region_height: height of each region
 * @columns: number of regions per row
 * @rows: number of region rows
 * @high_threshold: common channel threshold in the input scale
 * @input_bits: number of significant input bits
 * @regions: @columns * @rows channel sum/count records
 */
struct camss_stats_awb_bg {
	struct v4l2_isp_block_header header;
	__u16 left;
	__u16 top;
	__u16 region_width;
	__u16 region_height;
	__u16 columns;
	__u16 rows;
	__u16 high_threshold;
	__u16 input_bits;
	struct camss_stats_bg_region regions[];
} __attribute__((aligned(8)));

/* Maximum grid exported by the current VFE175 statistics configuration. */
#define CAMSS_STATS_BG_COLUMNS		32
#define CAMSS_STATS_BG_ROWS		24
#define CAMSS_STATS_BG_MAX_REGIONS	(CAMSS_STATS_BG_COLUMNS * CAMSS_STATS_BG_ROWS)
#define CAMSS_STATS_BG_MAX_PAYLOAD	(sizeof(struct camss_stats_awb_bg) + \
	CAMSS_STATS_BG_MAX_REGIONS * sizeof(struct camss_stats_bg_region))
#define CAMSS_STATS_MAX_PAYLOAD		(sizeof(struct camss_stats_bhist) + \
	CAMSS_STATS_BG_MAX_PAYLOAD + sizeof(struct camss_stats_ife_gtm))

#endif /* _UAPI_LINUX_QCOM_CAMSS_CONFIG_H */
