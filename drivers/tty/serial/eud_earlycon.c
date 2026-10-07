// SPDX-License-Identifier: GPL-2.0-only
/*
 * Early console over the Qualcomm Embedded USB Debugger (EUD) COM peripheral.
 *
 * EUD is a debug hub that takes over the USB port and exposes a small set of
 * USB devices to a host PC, one of which is a transmit-only COM FIFO.  It is a
 * register FIFO and not an 8250 compatible UART, so the generic
 * earlycon=uart,mmio,... path cannot drive it.  This driver stays small on
 * purpose: no baud rate, no line control, no receive path.
 *
 * Usage:
 *
 *     earlycon=eud,mmio,0x88e0000
 *
 * The host reassembles the [ID][LEN][DATA] frames, so run eudtool com-up and a
 * frame reassembler on the PC before booting the device.
 *
 * The register semantics come from the downstream Android driver
 * drivers/soc/qcom/eud.c, which is the reference for this hardware.
 */

#include <linux/console.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/serial_core.h>

#define EUD_REG_CSR_EUD_EN    0x1014 /* BIT(0) enables the EUD block */
#define EUD_REG_INT_STATUS_1  0x0044 /* BIT(1) is the TX flow control bit */
#define EUD_REG_COM_TX_ID     0x0000
#define EUD_REG_COM_TX_LEN    0x0004
#define EUD_REG_COM_TX_DAT    0x0008

#define EUD_COM_UART_ID       0x90
#define EUD_INT_TX            BIT(1)

/*
 * The TX FIFO is shallower than a dozen bytes and truncates a longer burst,
 * so keep the chunk size that is proven on this hardware.
 */
#define EUD_COM_CHUNK         6u

/*
 * Bounded poll: the timer is not calibrated this early, so there is nothing to
 * sleep with, and a wrong status readback must never hang the boot.
 */
#define EUD_TX_POLL_LIMIT     200000u

static void eud_wait_tx(void __iomem *base)
{
	unsigned int i;

	for (i = 0; i < EUD_TX_POLL_LIMIT; i++) {
		if (readl_relaxed(base + EUD_REG_INT_STATUS_1) & EUD_INT_TX)
			return;
		barrier();
	}
}

static void eud_write(struct console *con, const char *s, unsigned int n)
{
	struct earlycon_device *dev = con->data;
	void __iomem *base = dev->port.membase;
	unsigned int i;

	while (n) {
		unsigned int chunk = min(n, EUD_COM_CHUNK);

		writel_relaxed(EUD_COM_UART_ID, base + EUD_REG_COM_TX_ID);
		writel_relaxed(chunk, base + EUD_REG_COM_TX_LEN);
		for (i = 0; i < chunk; i++)
			writel_relaxed(s[i], base + EUD_REG_COM_TX_DAT);

		s += chunk;
		n -= chunk;
		eud_wait_tx(base);
	}
}

static int __init eud_setup(struct earlycon_device *device, const char *opt)
{
	void __iomem *base = device->port.membase;

	if (!base)
		return -ENODEV;

	/*
	 * Enable the EUD block if an earlier boot stage did not.  The EDK2 port
	 * of this device enables it in BDS, but the Android boot chain does not
	 * touch it at all.  The interrupt mask is deliberately left alone:
	 * arming interrupt sources before the interrupt controller and a driver
	 * exist would only risk a spurious interrupt later.
	 */
	writel_relaxed(1, base + EUD_REG_CSR_EUD_EN);

	device->con->write = eud_write;
	return 0;
}

EARLYCON_DECLARE(eud, eud_setup);
