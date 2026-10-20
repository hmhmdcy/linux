/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef CAMSS_ICP_A5_IPE_H
#define CAMSS_ICP_A5_IPE_H

#include "camss-icp-a5-boot.h"

#define CAMSS_A5_IPE_FRAME_BYTES 3816U
#define CAMSS_A5_IPE_IQ_BYTES 2584U
#define CAMSS_A5_IPE_FRAME_ACK_BYTES 72U

struct camss_a5_ipe_buffer {
	u32 iova, bytes;
};

/* A wire builder, not a DMA mapping or submission interface. The future
 * request owner must map every range for both A5 and the relevant engines,
 * preserve buffers through completion/reset, and validate settings/commands.
 * Initial fixtures disable IQ modules; this builder does not inspect IQ
 * content. Command arrays remain empty. No hardware frame is submitted here.
 */
struct camss_a5_ipe_frame {
	struct camss_a5_ipe_buffer striping, iq, scratch, bl, input, output;
	const void *striping_cpu;
	u32 required_scratch_bytes, required_bl_bytes;
	u32 request_id, max_cores, target_time_ns;
	u64 user_data;
};

int camss_a5_ipe_striping_bytes(const void *data, u32 bytes, u32 *used);
int camss_a5_ipe_encode_frame(void *destination, u32 bytes,
			    const struct camss_a5_ipe_io *io,
			    const struct camss_a5_ipe_frame *frame);


/* Sleeping synchronous bring-up transfer. Only the verified all-zero IQ
 * fixture is accepted. The core owns all DMA backing and retains it after
 * uncertain completion until A5 and both IPE engines are quiescent.
 */
struct camss_a5_ipe_job {
	const void *striping, *iq, *input;
	void *output;
	u32 striping_bytes, iq_bytes, input_bytes, output_bytes;
	u32 bl_bytes, request_id, max_cores, target_time_ns;
};
int camss_a5_ipe_run(struct camss_a5 *a5, struct camss_a5_session *session,
		     const struct camss_a5_ipe_job *job);

#endif
