// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm EUD COM: console, tty and command channel (realme X2 Pro).
 *
 * EUD presents a small register FIFO that a host PC sees as a serial port.  On
 * this board (RMX1931 / samurai) the MTP UART is not wired, so this FIFO is the
 * only console there is. TX pacing and the RX41 whole-frame method were
 * measured on hardware; RX46 adds a bounded IRQ diagnostic for receipt loss.
 *
 *  - TX: one register write is one FIFO entry: ID (0x90), LEN, then one entry
 *    per data byte.  The FIFO is about seven entries deep and INT_STATUS_1
 *    BIT(1) reads back stuck-at-set on this unit, so the only flow control that
 *    works is timing: 200 us per register write, 2 ms per frame.  Faster than
 *    that and whole frames are dropped, which loses four bytes out of the
 *    middle of a log line.
 *  - RX: id 0x90 carries tty payload of length 1 or 3..14; length 2 remains
 *    the header-only bootloader command. SM8150 SOUTH AHB2PHY TOP_CFG must
 *    be set to 0x11 before consecutive DAT reads. Session 41 verifies native
 *    ABC/DEFG in UEFI across reboot and in Linux. Collect the whole frame
 *    under the TX lock before printk or tty delivery; interleaved TX corrupts
 *    unread data. The host still needs receipt/retry for unaccepted OUTs.
 *  - COM fields occupy the low byte. Upper lanes may reflect different
 *    entries as the FIFO advances; mask with 0xff, never unpack four bytes.
 *  - The console is registered only when the command line asks for it: this
 *    firmware uses "earlycon=eud,... console=eud" without keep_bootcon, so the
 *    real console retires the early writer. FIFO accesses share the port lock.
 */

#include <linux/circ_buf.h>
#include <linux/console.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/jiffies.h>
#include <linux/kfifo.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/workqueue.h>
#include <linux/reboot.h>
#include <dt-bindings/interrupt-controller/arm-gic.h>

/* offsets inside the 0x2000 register window */
#define EUD_REG_COM_TX_ID      0x0000
#define EUD_REG_COM_TX_LEN     0x0004
#define EUD_REG_COM_TX_DAT     0x0008
#define EUD_REG_COM_RX_ID      0x000C
#define EUD_REG_COM_RX_LEN     0x0010
#define EUD_REG_COM_RX_DAT     0x0014
#define EUD_REG_INT0_EN_MASK   0x0020
#define EUD_REG_INT1_EN_MASK   0x0024
#define EUD_REG_INT_STATUS_0   0x0040
#define EUD_REG_INT_STATUS_1   0x0044
#define EUD_REG_CSR_EUD_EN     0x1014

/* SM8150 SOUTH SWMAN, confirmed by the stock RMX1931 DAL map. */
#define EUD_AHB2PHY_TOP_CFG    0x088ee010
#define EUD_AHB2PHY_ONE_WAIT   0x11

#define EUD_COM_UART_ID        0x90    /* the id this port transmits with */
#define EUD_TX_CHUNK           4u      /* data bytes per frame: six entries */
#define EUD_TX_BYTE_US         200
#define EUD_TX_FRAME_US        2000
#define EUD_TX_JOURNAL_DEPTH   512
#define EUD_TX_SOURCE_TTY      1
#define EUD_TX_SOURCE_CONSOLE  2

struct eud_tx_record {
        u8 len;
        u8 source;
        u8 data[EUD_TX_CHUNK];
};

/* Version 1 binary snapshot: 48-byte LE header, then oldest-first records.
 * CRC32/IEEE covers the entire snapshot with the crc field zeroed.
 */
struct eud_tx_journal_header {
        u8 magic[8];
        __le32 version, size, count, record_size;
        __le64 first_seq, last_seq;
        __le32 crc, reserved;
} __packed;

#define EUD_RX_POLL_MS         20
#define EUD_RX_MAX_FRAME       14    /* MAX_FIFO_SIZE in kernel/msm */
#define EUD_RX_UART_ID         0x90    /* upstream UART_ID: the channel */
#define EUD_RX_CMD_ID          0x81    /* APPS exec-env: our command channel */
#define EUD_RX_CHAR_ID         0x82    /* payload is tty input */
#define EUD_RX_CHAR_ID2        0x83    /* alternate id for the next char */
#define EUD_INT_RX_PENDING     BIT(0)  /* INT_STATUS_1: RX data available */
#define EUD_INT_RX             BIT(0)  /* INT_STATUS_1: RX data pending */
#define EUD_RX_REPORTS         40      /* bounded diagnostic verbosity */
#define EUD_RX_QUEUE_DEPTH     32
#define EUD_RX_GIC_SPI         492     /* matching RMX1931 vendor DT */
#define EUD_RX_IRQ_EMPTY_MAX   8
#define EUD_RX_IRQ_GRACE_MS    100     /* five status periods without progress */

/* RX46 diagnostic fault reasons: retain polling/F1 if IRQ capture cannot run. */
#define EUD_IRQ_FAULT_EMPTY    1
#define EUD_IRQ_FAULT_HEADER   2
#define EUD_IRQ_FAULT_QUEUE    3
#define EUD_IRQ_FAULT_WATCHDOG 4
#define EUD_IRQ_FAULT_MASK     5
#define EUD_IRQ_FAULT_SETUP    6

struct eud_rx_frame {
        u32 after;
        u8 id;
        u8 len;
        u8 data[EUD_RX_MAX_FRAME];
        u8 source;
};

enum eud_rx_source {
        EUD_RX_SOURCE_POLL,
        EUD_RX_SOURCE_IRQ,
        EUD_RX_SOURCE_CONSOLE,
};

struct eud_rx_snapshot {
        u32 polls, pending, bad, frames, bytes, tty_before, no_tty, overrun;
        u32 bad_id, bad_len, irqs, irq_frames, poll_frames, empty;
        u32 watchdog, drops, fault, active;
};

/* one FIFO entry is consumed per poll; the id byte selects the state */
#define EUD_RX_STATE_IDLE      0
#define EUD_RX_STATE_CHAR      1
#define EUD_RX_STATE_CMD       2

#define EUD_PORT_TYPE          124     /* PORT_EUD; no UAPI number assigned */

struct eud_port {
        struct uart_port port;
        struct work_struct tx_work;
	struct mutex tx_mutex;
	bool tx_stopping;
        struct delayed_work rx_work;
        u64 tx_journal_total;
        u32 tx_journal_head;
        u32 tx_journal_count;
        struct eud_tx_record tx_journal[EUD_TX_JOURNAL_DEPTH];
        u32 rx_polls;
        u32 rx_reports;
        u32 rx_last_status;
        u32 rx_last_id;
        u32 rx_last_len;
        u32 rx_last_after;
        u32 rx_pending;
        u32 rx_bad_headers;
        u32 rx_frames;
        u32 rx_f1;
        u32 rx_bytes;
        u32 rx_tty_bytes;
        u32 rx_no_tty;
        u32 rx_bad_id;
        u32 rx_bad_len;
        u8  rx_state;   /* EUD_RX_STATE: idle / expect char / expect cmd */
        u8  rx_collect; /* collecting a payload right now */
        u8  rx_have;    /* payload bytes collected so far */
        u8  rx_len;     /* payload bytes this message should have */
        u8  rx_id;      /* id of the message being collected */
        u8  rx_buf[EUD_RX_MAX_FRAME];
        u8  rx_delivered;
        bool console_registered;
        void __iomem *ahb2phy_cfg;
        u32 original_ahb2phy_cfg;
        int rx_irq;
        bool rx_irq_requested;
        bool rx_irq_mapping_owned;
        bool rx_irq_enabled;
        bool rx_irq_started;
        bool rx_stopping;
        bool rx_bootloader_pending;
        u8 rx_bootloader_source;
        bool rx_mask_changed;
        bool rx_fault_reported;
        u32 original_int1_mask;
        u32 rx_irq_events;
        u32 rx_irq_frames;
        u32 rx_poll_frames;
        u32 rx_console_frames;
        u32 rx_console_checks;
        u32 rx_console_f1;
        u32 rx_console_empty_irqs;
        bool rx_console_irq_credit;
        u32 rx_irq_empty;
        u32 rx_irq_empty_streak;
        u32 rx_watchdog_misses;
        bool rx_irq_waiting;
        unsigned long rx_irq_wait_start;
        u32 rx_irq_waits;
        u32 rx_irq_recovered;
        u32 rx_irq_wait_cleared;
        u32 rx_irq_wait_peak_ms;
        u8 rx_gic_first;
        u8 rx_gic_last;
        int rx_gic_error;
        u32 rx_queue_drops;
        u32 rx_irq_fault;
        u32 rx_queue_head;
        u32 rx_queue_tail;
        u32 rx_queue_count;
        struct eud_rx_frame rx_queue[EUD_RX_QUEUE_DEPTH];
};

static struct eud_port *eud;   /* one COM FIFO per board */

static int eud_console_rx_locked(struct eud_port *up);

static const char *eud_rx_source_name(u8 source)
{
        switch (source) {
        case EUD_RX_SOURCE_IRQ:
                return "irq";
        case EUD_RX_SOURCE_CONSOLE:
                return "console";
        default:
                return "poll";
        }
}

static inline struct eud_port *to_eud_port(struct uart_port *port)
{
        return container_of(port, struct eud_port, port);
}

/*
 * One register write is one FIFO entry.  udelay() is the only flow control that
 * works on this unit, so every write is paced.
 */
static inline void eud_put(void __iomem *base, unsigned int off, unsigned int v)
{
        udelay(EUD_TX_BYTE_US);
        writel_relaxed(v, base + off);
}

static void eud_send_frame(struct uart_port *port, const u8 *s, unsigned int n,
                            u8 source)
{
        struct eud_port *up = to_eud_port(port);
        struct eud_tx_record *record;
        void __iomem *base = port->membase;
        unsigned int i;

	/* The reboot handoff sets this under the UART lock. */
	if (up->tx_stopping)
		return;

        eud_put(base, EUD_REG_COM_TX_ID, EUD_COM_UART_ID);
        eud_put(base, EUD_REG_COM_TX_LEN, n);
        for (i = 0; i < n; i++)
                eud_put(base, EUD_REG_COM_TX_DAT, s[i]);

        udelay(EUD_TX_FRAME_US);

        /* Software evidence of the issued MMIO values, not a device ACK.
         * Both writers hold the UART lock; do not print or change pacing.
         */
        record = &up->tx_journal[up->tx_journal_head];
        memset(record, 0, sizeof(*record));
        record->len = n;
        record->source = source;
        memcpy(record->data, s, n);
        up->tx_journal_head = (up->tx_journal_head + 1) % EUD_TX_JOURNAL_DEPTH;
        up->tx_journal_count = min_t(u32, up->tx_journal_count + 1,
                                    EUD_TX_JOURNAL_DEPTH);
        up->tx_journal_total++;
}

/* tty write path: a workqueue, because the pacing sleeps between frames */
static void eud_tx_work(struct work_struct *work)
{
        struct eud_port *up = container_of(work, struct eud_port, tx_work);
        struct uart_port *port = &up->port;
        struct tty_port *tport;

        if (!port->state)
                return;
        tport = &port->state->port;
	mutex_lock(&up->tx_mutex);

        for (;;) {
                unsigned long flags;
                u8 buf[EUD_TX_CHUNK];
                unsigned int n = 0;
                u8 ch;

                uart_port_lock_irqsave(port, &flags);
                if (uart_tx_stopped(port)) {
                        uart_port_unlock_irqrestore(port, flags);
                        break;
                }
                while (n < EUD_TX_CHUNK && kfifo_get(&tport->xmit_fifo, &ch)) {
                        buf[n++] = ch;
                        port->icount.tx++;
                }
                uart_port_unlock_irqrestore(port, flags);

                if (!n)
                        break;

                uart_port_lock_irqsave(port, &flags);
            eud_send_frame(port, buf, n, EUD_TX_SOURCE_TTY);
            uart_port_unlock_irqrestore(port, flags);
        }

	mutex_unlock(&up->tx_mutex);

        if (kfifo_len(&tport->xmit_fifo) < WAKEUP_CHARS)
                uart_write_wakeup(port);
}

static unsigned int eud_tx_empty(struct uart_port *port)
{
        return TIOCSER_TEMT;
}

static void eud_start_tx(struct uart_port *port)
{
        struct eud_port *up = to_eud_port(port);

        schedule_work(&up->tx_work);
}

static void eud_stop_tx(struct uart_port *port)
{
}

static void eud_stop_rx(struct uart_port *port)
{
}

static void eud_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int eud_get_mctrl(struct uart_port *port)
{
        return TIOCM_CTS | TIOCM_DSR | TIOCM_CAR;
}

static int eud_startup(struct uart_port *port)
{
        return 0;
}

static void eud_shutdown(struct uart_port *port)
{
        struct eud_port *up = to_eud_port(port);
        struct tty_port *tport;
        u8 ch;

        cancel_work_sync(&up->tx_work);

        if (!port->state)
                return;
        tport = &port->state->port;
	mutex_lock(&up->tx_mutex);

        /*
         * Drain what the tty queued.  The pacing has to run in process
         * context, and a short lived writer (echo > /dev/ttyEUD0) closes the
         * port before the workqueue gets a chance to run - without this its
         * bytes are simply lost.
         */
        for (;;) {
                unsigned long flags;
                u8 buf[EUD_TX_CHUNK];
                unsigned int n = 0;

                uart_port_lock_irqsave(port, &flags);
                while (n < EUD_TX_CHUNK && kfifo_get(&tport->xmit_fifo, &ch)) {
                        buf[n++] = ch;
                        port->icount.tx++;
                }
                uart_port_unlock_irqrestore(port, flags);

                if (!n)
                        break;

                uart_port_lock_irqsave(port, &flags);
            eud_send_frame(port, buf, n, EUD_TX_SOURCE_TTY);
            uart_port_unlock_irqrestore(port, flags);
        }
	mutex_unlock(&up->tx_mutex);
}

static void eud_set_termios(struct uart_port *port, struct ktermios *new,
                            const struct ktermios *old)
{
        /* the FIFO has no line control at all */
}

static const char *eud_type(struct uart_port *port)
{
        return "EUD";
}

static void eud_config_port(struct uart_port *port, int flags)
{
        if (flags & UART_CONFIG_TYPE)
                port->type = EUD_PORT_TYPE;
}

static int eud_verify_port(struct uart_port *port, struct serial_struct *ser)
{
        if (ser->type != PORT_UNKNOWN && ser->type != EUD_PORT_TYPE)
                return -EINVAL;
        return 0;
}

static const struct uart_ops eud_ops = {
        .tx_empty       = eud_tx_empty,
        .set_mctrl      = eud_set_mctrl,
        .get_mctrl      = eud_get_mctrl,
        .stop_tx        = eud_stop_tx,
        .start_tx       = eud_start_tx,
        .stop_rx        = eud_stop_rx,
        .startup        = eud_startup,
        .shutdown       = eud_shutdown,
        .set_termios    = eud_set_termios,
        .type           = eud_type,
        .config_port    = eud_config_port,
        .verify_port    = eud_verify_port,
};

static struct uart_driver eud_uart_driver = {
        .owner       = THIS_MODULE,
        .driver_name = "eud_com",
        .dev_name    = "ttyEUD",
        .major       = 0,
        .minor       = 0,
        .nr          = 1,
};

/*
 * Console.  Registered only when "console=eud" is on the command line (see the
 * file comment).  .device makes /dev/console bind to ttyEUD once the port is up.
 */
static void eud_console_write_legacy(struct console *co, const char *s,
                              unsigned int count)
{
        struct uart_port *port;
        unsigned long flags;
        bool kick, collect;
        int ret;

        if (!eud)
                return;
        port = &eud->port;

        uart_port_lock_irqsave(port, &flags);
        /* Legacy printk also disables IRQs outside this callback. Service
         * RX at whole TX frame boundaries rather than waiting for the entire
         * record. The shared collector finishes DAT before any further TX;
         * receipt/tty/F1 stay in work context after releasing this lock.
         */
        ret = eud_console_rx_locked(eud);
        kick = ret != 0;
        collect = ret >= 0;
        while (count) {
                unsigned int n = min_t(unsigned int, count, EUD_TX_CHUNK);

                eud_send_frame(port, (const u8 *)s, n, EUD_TX_SOURCE_CONSOLE);
                s += n;
                count -= n;
                if (collect) {
                        ret = eud_console_rx_locked(eud);
                        kick |= ret != 0;
                        collect = ret >= 0;
                }
        }
        uart_port_unlock_irqrestore(port, flags);
        if (kick && !READ_ONCE(eud->rx_stopping))
                mod_delayed_work(system_percpu_wq, &eud->rx_work, 0);
}

/*
 * Normal printing is serialized with TTY TX by a mutex, while the port lock
 * protects just one complete FIFO frame and its RX collection. Keep the
 * measured pacing: speeding up MMIO loses whole frames on this board.
 *
 * The UART driver deliberately has no .cons association: the EUD console is
 * registered manually after probe. Port operations use their existing lock;
 * normal console/TTY writers also share tx_mutex. No non-printing UART path
 * tries to acquire a second nbcon NORMAL context while this writer owns it.
 */
static void eud_console_device_lock(struct console *co, unsigned long *flags)
{
	mutex_lock(&eud->tx_mutex);
	migrate_disable();
	*flags = 0;
}

static void eud_console_device_unlock(struct console *co, unsigned long flags)
{
	migrate_enable();
	mutex_unlock(&eud->tx_mutex);
}

static void eud_console_write_thread(struct console *co,
				     struct nbcon_write_context *wctxt)
{
	struct eud_port *up = eud;
	unsigned int offset = 0;
	bool kick = false, collect = true;

	while (offset < wctxt->len) {
		unsigned long flags;
		unsigned int n;
		bool owned;
		int ret;

		uart_port_lock_irqsave(&up->port, &flags);
		if (!nbcon_enter_unsafe(wctxt)) {
			uart_port_unlock_irqrestore(&up->port, flags);
			break;
		}

		if (!offset) {
			ret = eud_console_rx_locked(up);
			kick = ret != 0;
			collect = ret >= 0;
		}

		n = min_t(unsigned int, wctxt->len - offset, EUD_TX_CHUNK);
		eud_send_frame(&up->port, (const u8 *)wctxt->outbuf + offset,
			       n, EUD_TX_SOURCE_CONSOLE);
		offset += n;
		if (collect) {
			ret = eud_console_rx_locked(up);
			kick |= ret != 0;
			collect = ret >= 0;
		}

		owned = nbcon_exit_unsafe(wctxt);
		uart_port_unlock_irqrestore(&up->port, flags);
		if (!owned)
			break;
		cond_resched();
	}

	if (kick && !READ_ONCE(up->rx_stopping))
		mod_delayed_work(system_percpu_wq, &up->rx_work, 0);
}

/*
 * The old atomic writer is retained only for final panic flushing. It takes
 * the port lock and can hold IRQs off for an entire record, so it must never
 * be advertised as a safe atomic nbcon callback or used for normal printk.
 */
static void eud_console_write_atomic(struct console *co,
				     struct nbcon_write_context *wctxt)
{
	if (!nbcon_enter_unsafe(wctxt))
		return;
	eud_console_write_legacy(co, wctxt->outbuf, wctxt->len);
	nbcon_exit_unsafe(wctxt);
}

static struct console eud_console = {
        .name   = "eud",
	.write_thread = eud_console_write_thread,
	.write_atomic = eud_console_write_atomic,
	.device_lock = eud_console_device_lock,
	.device_unlock = eud_console_device_unlock,
        .device = uart_console_device,
	.flags = CON_PRINTBUFFER | CON_NBCON | CON_NBCON_ATOMIC_UNSAFE,
        .index  = -1,
        .data   = &eud_uart_driver,
};

/*
 * Command channel.  Only the header is reliable, so the command travels in the
 * length byte: [0x81][cmd].  Payload bytes, when they do show up, are passed to
 * the tty instead.
 */
static void eud_command(struct eud_port *up, const u8 *data, unsigned int n)
{
        void __iomem *base = up->port.membase;
        unsigned int i;
        u8 cmd = n ? data[0] : 0;

        switch (cmd) {
        case 1:
                pr_info("eud: pong\n");
                break;
        case 2:
                pr_info("eud: regs");
                for (i = 0; i <= 0x60; i += 4)
                        pr_cont(" %02x=%08x", i, readl_relaxed(base + i));
                pr_cont("\n");
                break;
        case 3:
                pr_info("eud: polls=%u reports=%u last_status=%08x\n",
                        up->rx_polls, up->rx_reports, up->rx_last_status);
                break;
        default:
                pr_info("eud: unknown command %u\n", cmd);
                break;
        }
}

/* Length 2 reboots to bootloader. Disable EUD first so
 * the USB PHY goes back to the gadget/ABL, else the host never sees fastboot. */
static bool eud_reboot_pending;

static void eud_restore_ahb2phy(struct eud_port *up)
{
        writel(up->original_ahb2phy_cfg, up->ahb2phy_cfg);
        /* Complete restoration before handing the PHY back to ABL. */
        readl(up->ahb2phy_cfg);
}

/* Called with the shared UART lock. GICv3 disable does not wait for this
 * handler; synchronization/freeing happens in process context on removal.
 */
static void eud_rx_irq_stop_locked(struct eud_port *up, u32 fault)
{
        up->rx_irq_waiting = false;
        up->rx_console_irq_credit = false;
        if (up->rx_irq_enabled) {
                up->rx_irq_enabled = false;
                disable_irq_nosync(up->rx_irq);
        }
        if (up->rx_mask_changed) {
                writel(up->original_int1_mask,
                       up->port.membase + EUD_REG_INT1_EN_MASK);
                readl(up->port.membase + EUD_REG_INT1_EN_MASK);
                up->rx_mask_changed = false;
        }
        if (fault && !up->rx_irq_fault)
                up->rx_irq_fault = fault;
}

static void eud_reboot_cmd(struct eud_port *up, unsigned int len)
{
	unsigned long flags;
	const char *what;

	if (eud_reboot_pending)
		return;
	if (len == 2)
		what = "bootloader";
	else
		return;
	eud_reboot_pending = true;

	uart_port_lock_irqsave(&up->port, &flags);
	up->rx_stopping = true;
	eud_rx_irq_stop_locked(up, 0);
	uart_port_unlock_irqrestore(&up->port, flags);
	if (up->rx_irq_requested)
		synchronize_irq(up->rx_irq);

	pr_info("eud: reboot2 %s requested\n", what);
	/*
	 * The F1 notice is queued by RX work. With nbcon, printk returning
	 * does not mean its frames have reached the FIFO. Drain the queued
	 * records before retiring the printer and giving the PHY back to ABL.
	 * Do not hold tx_mutex while waiting for the printer that needs it.
	 */
	pr_flush(1000, true);
	if (up->console_registered) {
		unregister_console(&eud_console);
		up->console_registered = false;
	}

	/* Serialize the last TTY writer with hardware shutdown as well. */
	mutex_lock(&up->tx_mutex);
	uart_port_lock_irqsave(&up->port, &flags);
	up->tx_stopping = true;
	uart_port_unlock_irqrestore(&up->port, flags);
	eud_restore_ahb2phy(up);
	writel(0, up->port.membase + EUD_REG_CSR_EUD_EN);
	readl(up->port.membase + EUD_REG_CSR_EUD_EN);
	mutex_unlock(&up->tx_mutex);
	kernel_restart((char *)what);
}

static void eud_rx_dispatch(struct eud_port *up)
{
        struct uart_port *port = &up->port;
        unsigned int i;
        unsigned long flags;

        up->rx_delivered = 0;
        if (up->rx_id == EUD_RX_UART_ID && up->rx_len == 2) {
                eud_reboot_cmd(up, up->rx_len);
        } else if ((up->rx_id == EUD_RX_CHAR_ID || up->rx_id == EUD_RX_CHAR_ID2 ||
             up->rx_id == EUD_RX_UART_ID) &&
            up->rx_have) {
                if (port->state) {
                        uart_port_lock_irqsave(port, &flags);
                        up->rx_delivered = tty_insert_flip_string(
                                &port->state->port, up->rx_buf, up->rx_have);
                        port->icount.rx += up->rx_delivered;
                        port->icount.buf_overrun += up->rx_have - up->rx_delivered;
                        up->rx_tty_bytes += up->rx_delivered;
                        uart_port_unlock_irqrestore(port, flags);
                        tty_flip_buffer_push(&port->state->port);
                } else {
                        uart_port_lock_irqsave(port, &flags);
                        up->rx_no_tty++;
                        uart_port_unlock_irqrestore(port, flags);
                }
        } else if (up->rx_id == EUD_RX_CMD_ID) {
                eud_command(up, up->rx_buf, up->rx_have);
        } else if (up->rx_reports < EUD_RX_REPORTS) {
                pr_info("eud: rx id=%02x len=%02x data", up->rx_id, up->rx_have);
                for (i = 0; i < up->rx_have; i++)
                        pr_cont(" %02x", up->rx_buf[i]);
                pr_cont("\n");
                up->rx_reports++;
        }
}

/* Shared collector: called only under the UART lock with an already-read
 * STATUS1. IRQ and fallback polling cannot consume the FIFO concurrently.
 * No printk, tty operation or reboot is performed in hard IRQ context.
 */
static int eud_rx_collect_locked(struct eud_port *up, u32 s1, u8 source)
{
        void __iomem *base = up->port.membase;
        struct eud_rx_frame frame = {};
        u32 id, len;
        unsigned int i;

        up->rx_last_status = s1;
        if (!(s1 & EUD_INT_RX_PENDING))
                return 0;

        up->rx_pending++;
        id = readl(base + EUD_REG_COM_RX_ID) & 0xff;
        len = readl(base + EUD_REG_COM_RX_LEN) & 0xff;
        up->rx_last_id = id;
        up->rx_last_len = len;
        if (id == EUD_RX_UART_ID && len == 2) {
                up->rx_f1++;
                up->rx_bootloader_pending = true;
                up->rx_bootloader_source = source;
                if (source == EUD_RX_SOURCE_CONSOLE)
                        up->rx_console_f1++;
                eud_rx_irq_stop_locked(up, 0);
                return 2;
        }
        if ((id != EUD_RX_UART_ID && id != EUD_RX_CHAR_ID &&
             id != EUD_RX_CHAR_ID2 && id != EUD_RX_CMD_ID) ||
            len < 1 || len > EUD_RX_MAX_FRAME) {
                up->rx_bad_headers++;
                up->rx_bad_id = id;
                up->rx_bad_len = len;
                return -EINVAL;
        }

        frame.id = id;
        frame.len = len;
        frame.source = source;
        for (i = 0; i < len; i++)
                frame.data[i] = readl(base + EUD_REG_COM_RX_DAT) & 0xff;
        frame.after = readl(base + EUD_REG_INT_STATUS_1);
        up->rx_frames++;
        up->rx_bytes += len;
        up->rx_last_after = frame.after;
        if (source == EUD_RX_SOURCE_IRQ)
                up->rx_irq_frames++;
        else if (source == EUD_RX_SOURCE_CONSOLE)
                up->rx_console_frames++;
        else
                up->rx_poll_frames++;
        if (up->rx_queue_count == EUD_RX_QUEUE_DEPTH) {
                up->rx_queue_drops++;
                return -ENOSPC;
        }
        up->rx_queue[up->rx_queue_head] = frame;
        up->rx_queue_head = (up->rx_queue_head + 1) % EUD_RX_QUEUE_DEPTH;
        up->rx_queue_count++;
        return 1;
}

/* No MMIO until normal RX startup; never recurse into printk/tty here.
 * One frame per boundary bounds work even if a host ignores receipt pacing.
 * A negative result stops console collection for this record and lets the
 * existing work-context fallback handle the fault. TOP_CFG and TX pacing stay
 * unchanged. The lock serializes console, IRQ and polling FIFO access.
 */
static int eud_console_rx_locked(struct eud_port *up)
{
        u32 s1;
        int ret;

        if (!up->rx_irq_started || up->rx_stopping || up->rx_bootloader_pending)
                return 0;
        up->rx_console_checks++;
        s1 = readl(up->port.membase + EUD_REG_INT_STATUS_1);
        ret = eud_rx_collect_locked(up, s1, EUD_RX_SOURCE_CONSOLE);
        if (ret > 0) {
                /* The level may already have reached the GIC before DAT was
                 * consumed. Credit at most ONE subsequent empty IRQ, not an
                 * unlimited storm. Any IRQ observation consumes this credit.
                 */
                if (up->rx_irq_enabled)
                        up->rx_console_irq_credit = true;
                up->rx_irq_empty_streak = 0;
                if (up->rx_irq_waiting) {
                        up->rx_irq_wait_peak_ms = max_t(u32,
                                up->rx_irq_wait_peak_ms,
                                jiffies_to_msecs(jiffies - up->rx_irq_wait_start));
                        up->rx_irq_wait_cleared++;
                        up->rx_irq_waiting = false;
                }
        } else if (ret < 0) {
                eud_rx_irq_stop_locked(up,
                        ret == -ENOSPC ? EUD_IRQ_FAULT_QUEUE : EUD_IRQ_FAULT_HEADER);
        }
        return ret;
}

/* Read-only GICv3 snapshot for the first/expired watchdog observation.
 * The board-specific direct SPI has no sleeping irq_chip bus lock. Caller
 * holds the UART lock with preemption/IRQs disabled. Never set chip state.
 * Bits: valid=0x80, pending=1, active=2, masked=4. Error leaves valid clear.
 */
static u8 eud_rx_gic_snapshot_locked(struct eud_port *up)
{
        static const enum irqchip_irq_state which[] = {
                IRQCHIP_STATE_PENDING, IRQCHIP_STATE_ACTIVE,
                IRQCHIP_STATE_MASKED,
        };
        bool value;
        unsigned int i;
        u8 result = 0;
        int ret;

        for (i = 0; i < ARRAY_SIZE(which); i++) {
                ret = irq_get_irqchip_state(up->rx_irq, which[i], &value);
                if (ret) {
                        up->rx_gic_error = ret;
                        return result;
                }
                if (value)
                        result |= BIT(i);
        }
        return result | 0x80;
}

static irqreturn_t eud_rx_irq(int irq, void *data)
{
        struct eud_port *up = data;
        unsigned long flags;
        bool kick = false;
        bool console_credit;
        u32 s1;
        int ret;

        uart_port_lock_irqsave(&up->port, &flags);
        up->rx_irq_events++;
        console_credit = up->rx_console_irq_credit;
        up->rx_console_irq_credit = false;
        if (!up->rx_stopping && up->rx_irq_enabled) {
                s1 = readl(up->port.membase + EUD_REG_INT_STATUS_1);
                ret = eud_rx_collect_locked(up, s1, EUD_RX_SOURCE_IRQ);
                if (!ret) {
                        up->rx_irq_empty++;
                        if (console_credit) {
                                up->rx_console_empty_irqs++;
                        } else if (++up->rx_irq_empty_streak >= EUD_RX_IRQ_EMPTY_MAX) {
                                eud_rx_irq_stop_locked(up, EUD_IRQ_FAULT_EMPTY);
                                kick = true;
                        }
                } else {
                        if (ret > 0 && up->rx_irq_waiting) {
                                up->rx_irq_wait_peak_ms = max_t(u32,
                                        up->rx_irq_wait_peak_ms,
                                        jiffies_to_msecs(jiffies - up->rx_irq_wait_start));
                                up->rx_irq_recovered++;
                                up->rx_irq_waiting = false;
                        }
                        up->rx_irq_empty_streak = 0;
                        kick = true;
                        if (ret < 0)
                                eud_rx_irq_stop_locked(up,
                                        ret == -ENOSPC ? EUD_IRQ_FAULT_QUEUE :
                                                        EUD_IRQ_FAULT_HEADER);
                }
        }
        uart_port_unlock_irqrestore(&up->port, flags);
        if (kick && !READ_ONCE(up->rx_stopping))
                mod_delayed_work(system_percpu_wq, &up->rx_work, 0);
        return IRQ_HANDLED;
}

static void eud_rx_snapshot_locked(struct eud_port *up,
                                   struct eud_rx_snapshot *s)
{
        s->polls = up->rx_polls;
        s->pending = up->rx_pending;
        s->bad = up->rx_bad_headers;
        s->frames = up->rx_frames;
        s->bytes = up->rx_bytes;
        s->tty_before = up->rx_tty_bytes;
        s->no_tty = up->rx_no_tty;
        s->overrun = up->port.icount.buf_overrun;
        s->bad_id = up->rx_bad_id;
        s->bad_len = up->rx_bad_len;
        s->irqs = up->rx_irq_events;
        s->irq_frames = up->rx_irq_frames;
        s->poll_frames = up->rx_poll_frames;
        s->empty = up->rx_irq_empty;
        s->watchdog = up->rx_watchdog_misses;
        s->drops = up->rx_queue_drops;
        s->fault = up->rx_irq_fault;
        s->active = up->rx_irq_enabled;
}

static void eud_rx_work(struct work_struct *work)
{
        struct eud_port *up = container_of(to_delayed_work(work),
                                           struct eud_port, rx_work);
        struct eud_rx_frame frame;
        struct eud_rx_snapshot stats;
        unsigned long flags;
        bool arm_notice = false, fault_notice = false;
        bool have = false, f1, queued;
        u8 f1_source;
        u32 s1;

        uart_port_lock_irqsave(&up->port, &flags);
        if (up->rx_stopping) {
                uart_port_unlock_irqrestore(&up->port, flags);
                return;
        }
        if (!up->rx_irq_started) {
                up->rx_irq_started = true;
                if (up->rx_irq_requested) {
                        up->original_int1_mask = readl(up->port.membase +
                                                      EUD_REG_INT1_EN_MASK) & 0xff;
                        up->rx_mask_changed = true;
                        /* Only RX may assert this handler. Do not enable
                         * stuck TX/VBUS/charger levels during this diagnostic.
                         */
                        writel((u32)EUD_INT_RX,
                               up->port.membase + EUD_REG_INT1_EN_MASK);
                        if ((readl(up->port.membase + EUD_REG_INT1_EN_MASK) & 0xff) !=
                            EUD_INT_RX) {
                                eud_rx_irq_stop_locked(up, EUD_IRQ_FAULT_MASK);
                        } else {
                                up->rx_irq_enabled = true;
                                /* Direct SM8150 GICv3 IRQ: no sleeping bus_lock.
                                 * The UART lock prevents remove/start races.
                                 */
                                enable_irq(up->rx_irq);
                        }
                        arm_notice = true;
                }
        }
        /* A pending status can race an IRQ waiting for the UART lock. Give
         * the IRQ unlocked execution opportunities before declaring failure.
         * Keep the original 20 ms observations; only persistently unserviced
         * data reaches the bounded watchdog. Never collect DAT while the IRQ
         * is enabled, or count fallback data as IRQ success.
         */
        if (!up->rx_queue_count && !up->rx_bootloader_pending) {
                up->rx_polls++;
                s1 = readl(up->port.membase + EUD_REG_INT_STATUS_1);
                if ((s1 & EUD_INT_RX_PENDING) && up->rx_irq_enabled) {
                        if (!up->rx_irq_waiting) {
                                up->rx_irq_waiting = true;
                                up->rx_irq_wait_start = jiffies;
                                up->rx_irq_waits++;
                                up->rx_gic_first = eud_rx_gic_snapshot_locked(up);
                        } else if (time_after_eq(jiffies,
                                   up->rx_irq_wait_start +
                                   msecs_to_jiffies(EUD_RX_IRQ_GRACE_MS))) {
                                up->rx_gic_last = eud_rx_gic_snapshot_locked(up);
                                up->rx_irq_wait_peak_ms = max_t(u32,
                                        up->rx_irq_wait_peak_ms,
                                        jiffies_to_msecs(jiffies - up->rx_irq_wait_start));
                                up->rx_watchdog_misses++;
                                eud_rx_irq_stop_locked(up, EUD_IRQ_FAULT_WATCHDOG);
                        }
                } else if (up->rx_irq_waiting) {
                        up->rx_irq_wait_cleared++;
                        up->rx_irq_waiting = false;
                }
                if (!up->rx_irq_enabled)
                        eud_rx_collect_locked(up, s1, EUD_RX_SOURCE_POLL);
        }
        f1 = up->rx_bootloader_pending;
        f1_source = up->rx_bootloader_source;
        if (!f1 && up->rx_queue_count) {
                frame = up->rx_queue[up->rx_queue_tail];
                up->rx_queue_tail = (up->rx_queue_tail + 1) % EUD_RX_QUEUE_DEPTH;
                up->rx_queue_count--;
                /* Only this work item owns the existing dispatch buffer. */
                up->rx_id = frame.id;
                up->rx_len = frame.len;
                up->rx_have = frame.len;
                memcpy(up->rx_buf, frame.data, frame.len);
                have = true;
        }
        queued = up->rx_queue_count != 0;
        eud_rx_snapshot_locked(up, &stats);
        if (up->rx_irq_fault && !up->rx_fault_reported) {
                up->rx_fault_reported = true;
                fault_notice = true;
        }
        uart_port_unlock_irqrestore(&up->port, flags);

        if (arm_notice)
                pr_info("eud: RX46 IRQ armed virq=%d active=%u mask=01 original=%02x grace_ms=%u\n",
                        up->rx_irq, stats.active, up->original_int1_mask,
                        EUD_RX_IRQ_GRACE_MS);
        if (fault_notice)
                pr_warn("eud: RX46 IRQ fallback fault=%u irqs=%u irq_frames=%u watchdog=%u empty=%u drops=%u\n",
                        stats.fault, stats.irqs, stats.irq_frames,
                        stats.watchdog, stats.empty, stats.drops);
        if (f1) {
                pr_info("eud: RX46 F1 via=%s irqs=%u fault=%u\n",
                        eud_rx_source_name(f1_source), stats.irqs, stats.fault);
                eud_reboot_cmd(up, 2);
                return;
        }
        if (!have)
                goto out;
        /* Finish the receipt before tty echo/command output can be queued. */
        if (frame.len == 1) {
                if (up->rx_buf[0] == 0x15)
                        pr_info("eud: tty byte=15 polls=%u pending=%u bad=%u frames=%u bytes=%u tty_before=%u no_tty=%u overrun=%u bad_id=%02x bad_len=%u irqs=%u irq_frames=%u poll_frames=%u empty=%u watchdog=%u drops=%u fault=%u active=%u\n",
                                stats.polls, stats.pending, stats.bad, stats.frames,
                                stats.bytes, stats.tty_before, stats.no_tty,
                                stats.overrun, stats.bad_id, stats.bad_len,
                                stats.irqs, stats.irq_frames, stats.poll_frames,
                                stats.empty, stats.watchdog, stats.drops,
                                stats.fault, stats.active);
                else
                        pr_info("eud: tty byte=%02x\n", up->rx_buf[0]);
        } else {
                pr_info("eud: rx frame len=%u data=%*ph s1_after=%08x via=%s\n",
                        frame.len, frame.len, up->rx_buf, frame.after,
                        eud_rx_source_name(frame.source));
        }
        eud_rx_dispatch(up);
        if (frame.id == EUD_RX_UART_ID && up->rx_delivered != frame.len)
                pr_warn("eud: tty accepted %u/%u bytes\n", up->rx_delivered, frame.len);
out:
        if (!READ_ONCE(up->rx_stopping))
                schedule_delayed_work(&up->rx_work,
                                      queued ? 0 : msecs_to_jiffies(EUD_RX_POLL_MS));
}

/* Read a coherent software snapshot, never touch the hardware for diagnostics.
 * The counter updates reuse the existing RX/TX lock and MMIO reads. There is
 * no hardware access in this sysfs read. RX46 also records IRQ versus fallback
 * frames and bounded-fault counters; Ctrl-U snapshots include these fields.
 */
static ssize_t rx_stats_show(struct device *dev, struct device_attribute *attr,
                             char *buf)
{
        struct uart_port *port = dev_get_drvdata(dev);
        struct eud_port *up = to_eud_port(port);
        unsigned long flags;
        ssize_t n;

        uart_port_lock_irqsave(port, &flags);
        n = sysfs_emit(buf,
                "polls=%u pending=%u bad=%u frames=%u f1=%u bytes=%u tty=%u no_tty=%u overrun=%u status=%08x id=%02x len=%u after=%08x bad_id=%02x bad_len=%u irqs=%u irq_frames=%u poll_frames=%u empty=%u watchdog=%u drops=%u fault=%u active=%u queued=%u console_frames=%u\n",
                up->rx_polls, up->rx_pending, up->rx_bad_headers,
                up->rx_frames, up->rx_f1, up->rx_bytes, up->rx_tty_bytes,
                up->rx_no_tty, port->icount.buf_overrun, up->rx_last_status,
                up->rx_last_id, up->rx_last_len, up->rx_last_after,
                up->rx_bad_id, up->rx_bad_len, up->rx_irq_events,
                up->rx_irq_frames, up->rx_poll_frames, up->rx_irq_empty,
                up->rx_watchdog_misses, up->rx_queue_drops, up->rx_irq_fault,
                up->rx_irq_enabled, up->rx_queue_count, up->rx_console_frames);
        uart_port_unlock_irqrestore(port, flags);
        return n;
}
static DEVICE_ATTR_RO(rx_stats);

/* Compact software-only diagnostics avoid expanding the legacy Ctrl-U log.
 * first/last are non-atomic snapshots of three chip reads, not wire events.
 */
static ssize_t irq_watch_show(struct device *dev, struct device_attribute *attr,
                              char *buf)
{
        struct eud_port *up = to_eud_port(dev_get_drvdata(dev));
        unsigned long flags;
        ssize_t n;

        uart_port_lock_irqsave(&up->port, &flags);
        n = sysfs_emit(buf,
                "waits=%u recovered=%u cleared=%u waiting=%u max_ms=%u first=%02x last=%02x err=%d\n",
                up->rx_irq_waits, up->rx_irq_recovered, up->rx_irq_wait_cleared,
                up->rx_irq_waiting, up->rx_irq_wait_peak_ms,
                up->rx_gic_first, up->rx_gic_last, up->rx_gic_error);
        uart_port_unlock_irqrestore(&up->port, flags);
        return n;
}
static DEVICE_ATTR_RO(irq_watch);

/* Software-only provenance for collection while the console owns the port.
 * Credited empty IRQs remain in the ordinary empty/irqs totals.
 */
static ssize_t console_rx_show(struct device *dev, struct device_attribute *attr,
                               char *buf)
{
        struct eud_port *up = to_eud_port(dev_get_drvdata(dev));
        unsigned long flags;
        ssize_t n;

        uart_port_lock_irqsave(&up->port, &flags);
        n = sysfs_emit(buf,
                "checks=%u frames=%u f1=%u empty_irqs=%u credit=%u\n",
                up->rx_console_checks, up->rx_console_frames,
                up->rx_console_f1, up->rx_console_empty_irqs,
                up->rx_console_irq_credit);
        uart_port_unlock_irqrestore(&up->port, flags);
        return n;
}
static DEVICE_ATTR_RO(console_rx);

/* Explicit live, read-only chip snapshot; irq_watch itself stays software-only. */
static ssize_t irq_state_show(struct device *dev, struct device_attribute *attr,
                              char *buf)
{
        struct eud_port *up = to_eud_port(dev_get_drvdata(dev));
        unsigned long flags;
        u8 state = 0;
        int error = -ENXIO;

        uart_port_lock_irqsave(&up->port, &flags);
        if (up->rx_irq_requested) {
                state = eud_rx_gic_snapshot_locked(up);
                error = state & 0x80 ? 0 : up->rx_gic_error;
        }
        uart_port_unlock_irqrestore(&up->port, flags);
        return sysfs_emit(buf, "gic=%02x err=%d\n", state, error);
}
static DEVICE_ATTR_RO(irq_state);

/* One atomic read into a buffer large enough for the entire snapshot.
 * Save it to tmpfs before transporting it through EUD: CRC detects loss in
 * the subsequent base64 output. No register reads/writes occur here.
 */
static ssize_t tx_journal_read(struct file *file, struct kobject *kobj,
                               const struct bin_attribute *attr, char *buf,
                               loff_t off, size_t count)
{
        struct eud_port *up = to_eud_port(dev_get_drvdata(kobj_to_dev(kobj)));
        struct eud_tx_journal_header header = {};
        unsigned long flags;
        unsigned int i, start, size;

        if (off)
                return 0;
        uart_port_lock_irqsave(&up->port, &flags);
        size = sizeof(header) + up->tx_journal_count * sizeof(struct eud_tx_record);
        if (count < size) {
                uart_port_unlock_irqrestore(&up->port, flags);
                return -EINVAL;
        }
        memcpy(header.magic, "EUDTXJ48", sizeof(header.magic));
        header.version = cpu_to_le32(1);
        header.size = cpu_to_le32(size);
        header.count = cpu_to_le32(up->tx_journal_count);
        header.record_size = cpu_to_le32(sizeof(struct eud_tx_record));
        header.first_seq = cpu_to_le64(up->tx_journal_count ?
                                up->tx_journal_total - up->tx_journal_count + 1 : 0);
        header.last_seq = cpu_to_le64(up->tx_journal_total);
        memcpy(buf, &header, sizeof(header));
        start = (up->tx_journal_head + EUD_TX_JOURNAL_DEPTH - up->tx_journal_count) %
                EUD_TX_JOURNAL_DEPTH;
        for (i = 0; i < up->tx_journal_count; i++)
                memcpy(buf + sizeof(header) + i * sizeof(struct eud_tx_record),
                       &up->tx_journal[(start + i) % EUD_TX_JOURNAL_DEPTH],
                       sizeof(struct eud_tx_record));
        header.crc = cpu_to_le32(crc32_le(~0, buf, size) ^ ~0);
        memcpy(buf, &header, sizeof(header));
        uart_port_unlock_irqrestore(&up->port, flags);
        return size;
}
static BIN_ATTR_RO(tx_journal, sizeof(struct eud_tx_journal_header) +
                             EUD_TX_JOURNAL_DEPTH * sizeof(struct eud_tx_record));

static struct attribute *eud_attrs[] = {
        &dev_attr_rx_stats.attr,
        &dev_attr_irq_watch.attr,
        &dev_attr_irq_state.attr,
        &dev_attr_console_rx.attr,
        NULL,
};
static const struct bin_attribute *const eud_bin_attrs[] = {
        &bin_attr_tx_journal,
        NULL,
};
static const struct attribute_group eud_group = {
        .attrs = eud_attrs,
        .bin_attrs = eud_bin_attrs,
};
__ATTRIBUTE_GROUPS(eud);

/* Logdump-only diagnostic for the current firmware DT, which omits this IRQ.
 * The matching RMX1931 vendor DT specifies SPI 492 (GIC hardware ID 524),
 * level high. Restrict synthesis to this board/address and direct GICv3.
 * A future permanent driver should consume a proper DT IRQ resource.
 */
static int eud_rx_request_irq(struct platform_device *pdev, struct eud_port *up)
{
        struct device_node *parent;
        struct irq_domain *domain;
        struct irq_data *irq_data;
        struct irq_fwspec spec = {};
        u32 cells;
        unsigned int previous;
        bool legacy;
        int ret;

        parent = of_irq_find_parent(pdev->dev.of_node);
        if (!parent)
                return -ENXIO;
        if (!of_device_is_compatible(parent, "arm,gic-v3") ||
            of_property_read_u32(parent, "#interrupt-cells", &cells) || cells != 3) {
                ret = -EINVAL;
                goto put_parent;
        }
        legacy = !of_property_present(pdev->dev.of_node, "interrupts") &&
                 !of_property_present(pdev->dev.of_node, "interrupts-extended");
        if (legacy) {
                if (!of_machine_is_compatible("realme,samurai") ||
                    up->port.mapbase != 0x088e0000) {
                        ret = -ENODEV;
                        goto put_parent;
                }
                domain = irq_find_host(parent);
                if (!domain) {
                        ret = -EPROBE_DEFER;
                        goto put_parent;
                }
                previous = irq_find_mapping(domain, EUD_RX_GIC_SPI + 32);
                spec.fwnode = of_fwnode_handle(parent);
                spec.param_count = 3;
                spec.param[0] = GIC_SPI;
                spec.param[1] = EUD_RX_GIC_SPI;
                spec.param[2] = IRQ_TYPE_LEVEL_HIGH;
                up->rx_irq = irq_create_fwspec_mapping(&spec);
                up->rx_irq_mapping_owned = up->rx_irq > 0 && !previous;
        } else {
                up->rx_irq = platform_get_irq(pdev, 0);
        }
        of_node_put(parent);
        if (up->rx_irq <= 0)
                return up->rx_irq ? up->rx_irq : -ENXIO;
        irq_data = irq_get_irq_data(up->rx_irq);
        if (!irq_data || irq_data->hwirq != EUD_RX_GIC_SPI + 32) {
                ret = -EINVAL;
                goto dispose;
        }
        ret = request_irq(up->rx_irq, eud_rx_irq, IRQF_NO_AUTOEN,
                          "eud-com-rx46", up);
        if (ret)
                goto dispose;
        up->rx_irq_requested = true;
        dev_info(&pdev->dev, "RX46 IRQ requested virq=%d hwirq=%lu route=%s, deferred enable\n",
                 up->rx_irq, irq_data->hwirq, legacy ? "legacy-SPI492" : "DT");
        return 0;

dispose:
        if (up->rx_irq_mapping_owned) {
                irq_dispose_mapping(up->rx_irq);
                up->rx_irq_mapping_owned = false;
        }
        up->rx_irq = 0;
        return ret;
put_parent:
        of_node_put(parent);
        return ret;
}

static int eud_probe(struct platform_device *pdev)
{
        struct eud_port *up;
        struct uart_port *port;
        struct resource *res;
        int ret;

        if (eud)
                return -EBUSY;

        up = devm_kzalloc(&pdev->dev, sizeof(*up), GFP_KERNEL);
        if (!up)
                return -ENOMEM;

        port = &up->port;
        port->membase = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
        if (IS_ERR(port->membase))
                return PTR_ERR(port->membase);
        port->mapbase = res->start;

        /* This driver is bound only to qcom,sm8150-eud-com. The stock
         * dwc3-msm driver uses 0x11 for one read/write wait state; RX41
         * verifies it enables native FIFO advancement on this board.
         */
        up->ahb2phy_cfg = devm_ioremap(&pdev->dev, EUD_AHB2PHY_TOP_CFG,
                                     sizeof(u32));
        if (!up->ahb2phy_cfg)
                return -ENOMEM;
        up->original_ahb2phy_cfg = readl(up->ahb2phy_cfg);
        writel(EUD_AHB2PHY_ONE_WAIT, up->ahb2phy_cfg);
        if (readl(up->ahb2phy_cfg) != EUD_AHB2PHY_ONE_WAIT) {
                eud_restore_ahb2phy(up);
                return dev_err_probe(&pdev->dev, -EIO,
                                     "AHB2PHY one-wait readback failed\n");
        }
        dev_info(&pdev->dev, "AHB2PHY TOP_CFG=%08x original=%08x\n",
                 EUD_AHB2PHY_ONE_WAIT, up->original_ahb2phy_cfg);

        port->dev      = &pdev->dev;
        port->iotype   = UPIO_MEM;
        port->flags    = UPF_BOOT_AUTOCONF | UPF_SKIP_TEST;
        port->ops      = &eud_ops;
        port->fifosize = 7;
        port->type     = EUD_PORT_TYPE;
        port->line     = 0;
        port->uartclk  = 115200;
        spin_lock_init(&port->lock);
	mutex_init(&up->tx_mutex);

        /* the EDK2 boot chain leaves EUD enabled; enable it if it did not */
        writel_relaxed(1, port->membase + EUD_REG_CSR_EUD_EN);

        INIT_WORK(&up->tx_work, eud_tx_work);
        INIT_DELAYED_WORK(&up->rx_work, eud_rx_work);

        ret = uart_add_one_port(&eud_uart_driver, port);
        if (ret) {
                eud_restore_ahb2phy(up);
                return ret;
        }

        platform_set_drvdata(pdev, port);
        eud = up;

        ret = eud_rx_request_irq(pdev, up);
        if (ret) {
                up->rx_irq_fault = EUD_IRQ_FAULT_SETUP;
                dev_warn(&pdev->dev, "RX46 IRQ unavailable: %d; polling retained\n", ret);
        }

        if (strstr(saved_command_line, "console=eud")) {
                register_console(&eud_console);
                up->console_registered = true;
        }

        /* let the boot log burst drain before the RX work starts talking */
        schedule_delayed_work(&up->rx_work, msecs_to_jiffies(20000));

        dev_info(&pdev->dev,
                 "EUD COM ready: ttyEUD0, [0x%02x][1,3..14]=tty [2]=bootloader\n",
                 EUD_RX_UART_ID);
        return 0;
}

static void eud_remove(struct platform_device *pdev)
{
        struct uart_port *port = platform_get_drvdata(pdev);
        struct eud_port *up = to_eud_port(port);
        unsigned long flags;

        uart_port_lock_irqsave(port, &flags);
        up->rx_stopping = true;
        eud_rx_irq_stop_locked(up, 0);
        uart_port_unlock_irqrestore(port, flags);
        if (up->rx_irq_requested) {
                free_irq(up->rx_irq, up);
                up->rx_irq_requested = false;
        }
        cancel_delayed_work_sync(&up->rx_work);
        cancel_work_sync(&up->tx_work);
        if (up->console_registered)
                unregister_console(&eud_console);
        uart_remove_one_port(&eud_uart_driver, port);
        eud_restore_ahb2phy(up);
        if (up->rx_irq_mapping_owned)
                irq_dispose_mapping(up->rx_irq);
        eud = NULL;
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

static int __init eud_init(void)
{
        int ret;

        ret = uart_register_driver(&eud_uart_driver);
        if (ret)
                return ret;

        ret = platform_driver_register(&eud_com_driver);
        if (ret)
                uart_unregister_driver(&eud_uart_driver);

        return ret;
}
module_init(eud_init);

static void __exit eud_exit(void)
{
        platform_driver_unregister(&eud_com_driver);
        uart_unregister_driver(&eud_uart_driver);
}
module_exit(eud_exit);

MODULE_DESCRIPTION("Qualcomm EUD COM console, tty and command channel");
MODULE_LICENSE("GPL");
