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
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/serial_core.h>

#include <asm/early_ioremap.h>

#define EUD_REG_CSR_EUD_EN    0x1014 /* BIT(0) enables the EUD block */
#define EUD_REG_INT_STATUS_1  0x0044 /* BIT(1) is the TX flow control bit */
#define EUD_REG_COM_TX_ID     0x0000
#define EUD_REG_COM_TX_LEN    0x0004
#define EUD_REG_COM_TX_DAT    0x0008

#define EUD_COM_UART_ID       0x90
#define EUD_INT_TX            BIT(1)

/*
 * One register write is one FIFO entry (ID, LEN, one per data byte), and the
 * TX FIFO is only about seven entries deep.  Six data bytes made the frame
 * eight entries, so the tail was truncated and every following frame was
 * misaligned; four data bytes keep the frame at six entries.  Frames are only
 * started when the FIFO reports room; nothing is written mid-frame.
 */
#define EUD_COM_CHUNK        4u

/*
 * Bounded poll: the timer is not calibrated this early, so there is nothing to
 * sleep with, and a wrong status readback must never hang the boot.
 */
#define EUD_TX_POLL_LIMIT     200000u

/*
 * The status bit cannot be trusted on this unit: INT_STATUS_1 BIT(1) reads
 * back stuck-at-set, so eud_wait_tx() returns at once and the FIFO is written
 * at MMIO speed.  It is only about seven entries deep, so whole frames are
 * dropped - measured on hardware 2026-10-08, the firmware's paced frames
 * arrived intact while the kernel's lost every few frames, which shows up as
 * four bytes missing from the middle of a log line.
 *
 * Pace by time instead, with the numbers the EDK2 port measured on the same
 * FIFO: 200 us per register write, 2 ms per frame, 1.2 KB/s, zero overflow.
 */
#define EUD_TX_BYTE_US               200
#define EUD_TX_FRAME_US              2000

static inline void eud_put(void __iomem *base, unsigned int off, unsigned int v)
{
        udelay(EUD_TX_BYTE_US);
        writel_relaxed(v, base + off);
}

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

	        eud_put(base, EUD_REG_COM_TX_ID, EUD_COM_UART_ID);
	        eud_put(base, EUD_REG_COM_TX_LEN, chunk);
	        for (i = 0; i < chunk; i++)
	                eud_put(base, EUD_REG_COM_TX_DAT, s[i]);

	        s += chunk;
	        n -= chunk;
	        udelay(EUD_TX_FRAME_US);
	}
}

static int __init eud_setup(struct earlycon_device *device, const char *opt)
{
	void __iomem *csr;

	if (!device->port.membase)
		return -ENODEV;

	/*
	 * Enable the EUD block if an earlier boot stage did not.  The EDK2 port
	 * of this device enables it in BDS, the Android boot chain does not
	 * touch it at all.
	 *
	 * CSR_EUD_EN sits one 4 KiB page above the COM FIFO, outside the window
	 * the earlycon framework maps for us - it maps 64 bytes, which is the
	 * page the FIFO lives in - so it needs a mapping of its own.  Writing
	 * through the framebuffer pointer would take a data abort here and kill
	 * the boot before any output.
	 */
	/* setup_earlycon() runs before slab and ordinary ioremap are ready. */
	csr = early_ioremap(device->port.mapbase + EUD_REG_CSR_EUD_EN, sizeof(u32));
	if (csr) {
		writel_relaxed(1, csr);
		/* Complete the one-shot enable before releasing the temporary mapping. */
		readl_relaxed(csr);
		early_iounmap(csr, sizeof(u32));
	}

	/*
	 * The interrupt mask is deliberately left alone: arming interrupt
	 * sources before the interrupt controller and a driver exist would only
	 * risk a spurious interrupt later.
	 */
	device->con->write = eud_write;
	return 0;
}

EARLYCON_DECLARE(eud, eud_setup);
