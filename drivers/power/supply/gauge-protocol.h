/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SAMURAI_GAUGE_PROTOCOL_H
#define SAMURAI_GAUGE_PROTOCOL_H
#ifdef __KERNEL__
#include <linux/errno.h>
#else
#include <errno.h>
#endif

/* TI SLUUA65E 12.1.28-31: 2 command + 0..32 data + checksum + length.
 * The checksum/length always occupy offsets 34/35 of a 36-byte read.
 */
static inline unsigned int sm_le16(const unsigned char *p)
{
	return p[0] | (unsigned int)p[1] << 8;
}

static inline unsigned int sm_le32(const unsigned char *p)
{
	return sm_le16(p) | sm_le16(p + 2) << 16;
}

static inline int sm_mac_data_size(unsigned int command)
{
	switch (command) {
	case 0x0001: return 2;
	case 0x0051: case 0x0053: case 0x0054: return 4;
	case 0x0057: return 2;
	case 0x0071: return 32;
	default: return -EINVAL; /* No arbitrary MAC, FET, unseal or NVM commands. */
	}
}

static inline int sm_mac_validate(const unsigned char *p, unsigned int bytes,
				  unsigned int command)
{
	unsigned int sum = 0, i;
	int size = sm_mac_data_size(command);

	if (size < 0) return size;
	if (bytes != 36 || p[35] != size + 4) return -EMSGSIZE;
	if (sm_le16(p) != command) return -EAGAIN;
	for (i = 0; i < (unsigned int)size + 2; i++) sum += p[i];
	return ((sum + p[34]) & 255) == 255 ? 0 : -EBADMSG;
}

/* A valid protocol response is not a claim that the silicon is a standard
 * TI 2610. The installed factory-family protocol reports device type 2719.
 * Keep the observed identity explicit; do not silently accept unknown packs.
 */
static inline int sm_gauge_protection_state(unsigned int device_type,
	unsigned int safety, unsigned int permanent, unsigned int operation,
	unsigned int manufacturing)
{
	if (device_type != 0x2719) return -ENODEV;
	if (safety || permanent) return -EIO;
	/* Require sealed state, autonomous FET action and gauging. Exclude test,
	 * calibration, shutdown, disabled-charge and active safety/PF modes.
	 * PF_EN is reported separately; its inherited value must not be changed.
	 */
	if ((operation & 0x300) != 0x300 ||
	    (operation & ((1U << 20) | (1U << 16) | 0x7c00)) ||
	    (manufacturing & 0x18) != 0x18 || (manufacturing & 0x8006))
		return -EACCES;
	return 0;
}
#endif
