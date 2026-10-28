/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SAMURAI_GAUGE_I2C_H
#define SAMURAI_GAUGE_I2C_H
#include <linux/delay.h>
#include <linux/i2c.h>
#include "gauge-protocol.h"

/* Caller holds I2C_LOCK_SEGMENT. No other MAC client may interleave. */
static inline int sm_gauge_word_locked(struct i2c_client *g, unsigned int reg,
				       bool write, unsigned int value)
{
	union i2c_smbus_data data = { .word = value };
	int ret = __i2c_smbus_xfer(g->adapter, g->addr, g->flags,
		write ? I2C_SMBUS_WRITE : I2C_SMBUS_READ, reg,
		I2C_SMBUS_WORD_DATA, &data);
	return ret < 0 ? ret : write ? 0 : data.word;
}

static inline int sm_gauge_mac_locked(struct i2c_client *g, unsigned int command,
				      unsigned char *packet)
{
	unsigned char start = 0x3e;
	struct i2c_msg messages[] = {
		{ .addr = g->addr, .len = 1, .buf = &start },
		{ .addr = g->addr, .flags = I2C_M_RD, .len = 36, .buf = packet },
	};
	int ret;

	if (sm_mac_data_size(command) < 0) return -EINVAL;
	ret = sm_gauge_word_locked(g, 0x3e, true, command);
	if (ret) return ret;
	usleep_range(1000, 1500); /* matched OEM query delay, no NVM transaction */
	ret = __i2c_transfer(g->adapter, messages, ARRAY_SIZE(messages));
	if (ret != ARRAY_SIZE(messages)) return ret < 0 ? ret : -EIO;
	return sm_mac_validate(packet, 36, command);
}
#endif
