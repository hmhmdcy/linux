// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm EUD COM console.
 *
 * EUD (Embedded USB Debugger) takes over the USB port and exposes a handful of
 * debug devices to a host PC.  One of them is a transmit-only register FIFO
 * which the host reads back as a serial port.  On this board (realme X2 Pro /
 * samurai) the MTP UART is not wired, so that FIFO is the only console there
 * is.  The register semantics come from the downstream Android driver
 * drivers/soc/qcom/eud.c (ttyEUD), which is the reference for this hardware.
 *
 * eud_earlycon.c covers the very beginning of the boot.  This driver is the
 * real console counterpart: it registers a struct console named eud, so the
 * log channel is selected and controlled from the kernel command line:
 *
 *     earlycon=eud,mmio,0x88e0000 console=tty0 console=eud
 *
 * List console=eud last.  The last console= option is the preferred console,
 * so this console gets CON_CONSDEV; the printk core then drops CON_PRINTBUFFER
 * (no second copy of a log the host already has) and unregisters the early
 * console.  keep_bootcon is therefore not needed any more - it only exists to
 * keep a CON_BOOT console alive.
 *
 * Three hardware quirks shape the code below:
 *
 *  - One register write is one FIFO entry: ID, LEN and one entry per data byte.
 *    The FIFO is only about seven entries deep and silently truncates a larger
 *    burst.  The host reassembles [ID][LEN][DATA] frames, so a single dropped
 *    byte desynchronises the stream for good, which is exactly what garbled
 *    the logs before (handover section 21).  Hence EUD_COM_CHUNK data bytes per
 *    frame, i.e. six entries, and a frame is never started unless the FIFO
 *    reports room.
 *  - A whole string must not be written in one go; it has to be chopped into
 *    frames and paced.
 *  - Console callbacks can run from atomic context.  That is why this is an
 *    nbcon console: write_thread() runs in task context and may sleep between
 *    frames, while write_atomic() must not and drops what it cannot send.
 *    Losing the tail of a log line is recoverable, wedging the boot is not.
 */

#include <linux/atomic.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>

#define EUD_REG_COM_TX_ID    0x0000
#define EUD_REG_COM_TX_LEN   0x0004
#define EUD_REG_COM_TX_DAT   0x0008
#define EUD_REG_INT_STATUS_1 0x0044
#define EUD_REG_CSR_EUD_EN   0x1014

#define EUD_COM_UART_ID      0x90
#define EUD_INT_TX           BIT(1)

/*
 * ID + LEN + EUD_COM_CHUNK data = six entries, comfortably below the ~7 entry
 * FIFO depth.  The early console used six data bytes, i.e. eight entries, and
 * that is what truncated every frame.
 */
#define EUD_COM_CHUNK        4u

/* Task context waits for the FIFO to drain; 2 ms is the EDK2 proven ceiling. */
#define EUD_TX_POLL_US       50
#define EUD_TX_TIMEOUT_US    2000
#define EUD_FRAME_GAP_MIN_US 200
#define EUD_FRAME_GAP_MAX_US 1000

/* Atomic context gets one short attempt per frame, then drops the rest. */
#define EUD_TX_ATOMIC_DELAY_US   1
#define EUD_TX_ATOMIC_TIMEOUT_US 200

struct eud_com {
    struct device *dev;
    void __iomem *base;
    atomic_t dropped_bytes;
};

static struct eud_com *eud;

/*
 * INT_STATUS_1 BIT(1) is the vendor driver's eud_tx_empty(): set means the TX
 * FIFO has drained.  Readbacks of this block are not always trustworthy - the
 * handover warns that TX_ID can read back 0x99999999 - so a timeout is treated
 * as unknown rather than full, see eud_write_frames().
 */
static bool eud_tx_idle(bool atomic)
{
    u32 val;

    if (atomic)
        return readl_poll_timeout_atomic(eud->base + EUD_REG_INT_STATUS_1,
                         val, val & EUD_INT_TX,
                         EUD_TX_ATOMIC_DELAY_US,
                         EUD_TX_ATOMIC_TIMEOUT_US) == 0;

    return readl_poll_timeout(eud->base + EUD_REG_INT_STATUS_1, val,
                  val & EUD_INT_TX, EUD_TX_POLL_US,
                  EUD_TX_TIMEOUT_US) == 0;
}
static unsigned int eud_write_frames(const char *s, unsigned int n, bool atomic)
{
    unsigned int written = 0;

    while (n) {
        unsigned int i, chunk = min(n, EUD_COM_CHUNK);
        bool idle = eud_tx_idle(atomic);

        /*
         * Atomic context cannot wait, so an unreadable status bit means stop
         * here: starting a frame we cannot finish would corrupt the stream
         * instead of merely cutting the log.
         */
        if (!idle && atomic)
            break;

        /*
         * Task context falls back to the timing the EDK2 port uses on the same
         * FIFO - six entries per frame, 2 ms between frames, no status readback
         * at all - so a flaky status bit costs speed, not correctness.
         */
        if (!idle)
            usleep_range(EUD_TX_TIMEOUT_US, EUD_TX_TIMEOUT_US + 1000);

        writel_relaxed(EUD_COM_UART_ID, eud->base + EUD_REG_COM_TX_ID);
        writel_relaxed(chunk, eud->base + EUD_REG_COM_TX_LEN);
        for (i = 0; i < chunk; i++)
            writel_relaxed(s[i], eud->base + EUD_REG_COM_TX_DAT);

        s += chunk;
        n -= chunk;
        written += chunk;

        if (!atomic)
            usleep_range(EUD_FRAME_GAP_MIN_US, EUD_FRAME_GAP_MAX_US);
    }

    return written;
}

static void eud_console_write_atomic(struct console *con,
                     struct nbcon_write_context *wctxt)
{
    unsigned int written;

    if (!nbcon_enter_unsafe(wctxt))
        return;

    written = eud_write_frames(wctxt->outbuf, wctxt->len, true);
    nbcon_exit_unsafe(wctxt);

    if (written < wctxt->len)
        atomic_add(wctxt->len - written, &eud->dropped_bytes);
}

static void eud_console_write_thread(struct console *con,
                     struct nbcon_write_context *wctxt)
{
    unsigned int written;

    if (!nbcon_enter_unsafe(wctxt))
        return;

    written = eud_write_frames(wctxt->outbuf, wctxt->len, false);
    nbcon_exit_unsafe(wctxt);

    if (written < wctxt->len)
        atomic_add(wctxt->len - written, &eud->dropped_bytes);
}

/*
 * This console is the only user of the FIFO, so the device lock only has to
 * satisfy the contract: it must disable migration and it serialises against
 * driver code - of which there is none here.
 */
static void eud_console_device_lock(struct console *con, unsigned long *flags)
{
    migrate_disable();
    *flags = 0;
}

static void eud_console_device_unlock(struct console *con, unsigned long flags)
{
    migrate_enable();
}

static struct console eud_console = {
    .name          = "eud",
    .write_atomic  = eud_console_write_atomic,
    .write_thread  = eud_console_write_thread,
    .device_lock   = eud_console_device_lock,
    .device_unlock = eud_console_device_unlock,
    .flags         = CON_PRINTBUFFER | CON_NBCON,
};

static ssize_t dropped_bytes_show(struct device *dev,
                  struct device_attribute *attr, char *buf)
{
    struct eud_com *chip = dev_get_drvdata(dev);

    return sysfs_emit(buf, "%u\n", atomic_read(&chip->dropped_bytes));
}
static DEVICE_ATTR_RO(dropped_bytes);

static struct attribute *eud_attrs[] = {
    &dev_attr_dropped_bytes.attr,
    NULL,
};
ATTRIBUTE_GROUPS(eud);

static int eud_probe(struct platform_device *pdev)
{
    struct eud_com *chip;

    if (eud)
        return -EBUSY;

    chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
    if (!chip)
        return -ENOMEM;

    chip->dev = &pdev->dev;
    chip->base = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(chip->base))
        return PTR_ERR(chip->base);

    /*
     * Enable the block if an earlier boot stage did not.  The EDK2 port of this
     * device enables EUD in BDS; the Android boot chain does not touch it at
     * all.  The interrupt mask is deliberately left alone, see eud_earlycon.c.
     */
    writel_relaxed(1, chip->base + EUD_REG_CSR_EUD_EN);

    platform_set_drvdata(pdev, chip);
    eud = chip;
    eud_console.data = chip;

    register_console(&eud_console);

    return 0;
}

static void eud_remove(struct platform_device *pdev)
{
    struct eud_com *chip = platform_get_drvdata(pdev);

    unregister_console(&eud_console);
    /* Make sure a late write cannot touch an unmapped window. */
    eud = NULL;
    wmb();
    devm_kfree(&pdev->dev, chip);
}

static const struct of_device_id eud_com_dt_match[] = {
    { .compatible = "qcom,sm8150-eud-com" },
    { }
};
MODULE_DEVICE_TABLE(of, eud_com_dt_match);

static struct platform_driver eud_com_driver = {
    .probe  = eud_probe,
    .remove = eud_remove,
    .driver = {
        .name           = "qcom_eud_com",
        .of_match_table = eud_com_dt_match,
        .dev_groups     = eud_groups,
    },
};
module_platform_driver(eud_com_driver);

MODULE_DESCRIPTION("Qualcomm EUD COM console");
MODULE_LICENSE("GPL");