/*
* $Id: mxs_ekuart.c,v 1.4 2015/02/05 15:56:44 gianluca Exp $
* Eurek SERIAL Support for RS485 mode
* hardware based transceiver and gpio
*
* Author: gianluca renzi <gianlucarenzi@eurek.it>
*
* based on:
* Freescale STMP37XX/STMP378X Application UART driver
*
* Copyright 2014/2015 Eurek Elettronica SRL Italy
*
* The code contained herein is licensed under the GNU General Public
* License. You may obtain a copy of the GNU General Public License
* Version 2 or later at the following locations:
*
* http://www.opensource.org/licenses/gpl-license.html
* http://www.gnu.org/copyleft/gpl.html
*/

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/of.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>
#include <linux/workqueue.h>
#include <linux/list.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include <dt-bindings/gpio/gpio.h>

#include <asm/cacheflush.h>

#define MXS_AUART_PORTS 5

#define AUART_CTRL0             0x00000000
#define AUART_CTRL0_SET         0x00000004
#define AUART_CTRL0_CLR         0x00000008
#define AUART_CTRL0_TOG         0x0000000c
#define AUART_CTRL1             0x00000010
#define AUART_CTRL1_SET         0x00000014
#define AUART_CTRL1_CLR         0x00000018
#define AUART_CTRL1_TOG         0x0000001c
#define AUART_CTRL2             0x00000020
#define AUART_CTRL2_SET         0x00000024
#define AUART_CTRL2_CLR         0x00000028
#define AUART_CTRL2_TOG         0x0000002c
#define AUART_LINECTRL          0x00000030
#define AUART_LINECTRL_SET      0x00000034
#define AUART_LINECTRL_CLR      0x00000038
#define AUART_LINECTRL_TOG      0x0000003c
#define AUART_LINECTRL2         0x00000040
#define AUART_LINECTRL2_SET     0x00000044
#define AUART_LINECTRL2_CLR     0x00000048
#define AUART_LINECTRL2_TOG     0x0000004c
#define AUART_INTR              0x00000050
#define AUART_INTR_SET          0x00000054
#define AUART_INTR_CLR          0x00000058
#define AUART_INTR_TOG          0x0000005c
#define AUART_DATA              0x00000060
#define AUART_STAT              0x00000070
#define AUART_DEBUG             0x00000080
#define AUART_VERSION           0x00000090
#define AUART_AUTOBAUD          0x000000a0

#define AUART_CTRL0_SFTRST          (1 << 31)
#define AUART_CTRL0_CLKGATE         (1 << 30)
#define AUART_CTRL0_RXTO_ENABLE     (1 << 27)
#define AUART_CTRL0_RXTIMEOUT(v)    (((v) & 0x7ff) << 16)
#define AUART_CTRL0_XFER_COUNT(v)   ((v) & 0xffff)

#define AUART_CTRL1_XFER_COUNT(v)   ((v) & 0xffff)

#define AUART_CTRL2_DMAONERR    (1 << 26)
#define AUART_CTRL2_TXDMAE      (1 << 25)
#define AUART_CTRL2_RXDMAE      (1 << 24)

#define AUART_CTRL2_TXIFSEL(v)    (((v) & 0x07) << 16)
#define AUART_CTRL2_CTSEN         (1 << 15)
#define AUART_CTRL2_RTSEN         (1 << 14)
#define AUART_CTRL2_RTS           (1 << 11)
#define AUART_CTRL2_RXE           (1 << 9)
#define AUART_CTRL2_TXE           (1 << 8)
#define AUART_CTRL2_UARTEN        (1 << 0)

#define AUART_LINECTRL_BAUD_DIVINT_SHIFT    16
#define AUART_LINECTRL_BAUD_DIVINT_MASK     0xffff0000
#define AUART_LINECTRL_BAUD_DIVINT(v)       (((v) & 0xffff) << 16)
#define AUART_LINECTRL_BAUD_DIVFRAC_SHIFT   8
#define AUART_LINECTRL_BAUD_DIVFRAC_MASK    0x00003f00
#define AUART_LINECTRL_BAUD_DIVFRAC(v)      (((v) & 0x3f) << 8)
#define AUART_LINECTRL_WLEN_MASK            0x00000060
#define AUART_LINECTRL_WLEN(v)              (((v) & 0x3) << 5)
#define AUART_LINECTRL_FEN                  (1 << 4)
#define AUART_LINECTRL_STP2                 (1 << 3)
#define AUART_LINECTRL_EPS                  (1 << 2)
#define AUART_LINECTRL_PEN                  (1 << 1)
#define AUART_LINECTRL_BRK                  (1 << 0)

#define AUART_INTR_RTIEN    (1 << 22)
#define AUART_INTR_TXIEN    (1 << 21)
#define AUART_INTR_RXIEN    (1 << 20)
#define AUART_INTR_CTSMIEN  (1 << 17)
#define AUART_INTR_RTIS     (1 << 6)
#define AUART_INTR_TXIS     (1 << 5)
#define AUART_INTR_RXIS     (1 << 4)
#define AUART_INTR_CTSMIS   (1 << 1)

#define AUART_STAT_BUSY         (1 << 29)
#define AUART_STAT_CTS          (1 << 28)
#define AUART_STAT_TXFE         (1 << 27)
#define AUART_STAT_TXFF         (1 << 25)
#define AUART_STAT_RXFE         (1 << 24)
#define AUART_STAT_OERR         (1 << 19)
#define AUART_STAT_BERR         (1 << 18)
#define AUART_STAT_PERR         (1 << 17)
#define AUART_STAT_FERR         (1 << 16)
#define AUART_STAT_RXCOUNT_MASK 0xffff

#define DRIVER_VERSION "$Id: mxs_ekuart.c,v 1.4 2015/02/05 15:56:44 gianluca Exp $"

#define DEBUG_LVL    KERN_ERR
#define ERROR        0
#define SILENT       1
#define INFO         2
#define VERBOSE      3
#define NOISY        4

#define MILLI 0
#define MICRO 1

/* Lower less verbose, higher more verbose */
static int debuglevel = ERROR;
static int hr_resolution = MILLI;

/*
* A seconda della granularita` richiesta si puo` utilizzare:
* 1 - Kernel timers (con latenza CONFIG_HZ che puo` essere anche
*       elevata (da 2ms fino a 10ms))
* 2 - tasklet che viene invocato da uno scheduler separato con
*       tempistiche piu` strette
* 3 - Kernel High Resolution Timers (con latenza di qualche decina di
*       microsecondi)
* 
* Come default utilizziamo il sistema High Resolution Timer.
* E` modificabile tramite la variabile 'use_method=' passata al driver
* oppure specificando in kernel modprobe cmdline:
* 'mxs_ekuart.use_method='
* 
* Allo stesso modo nel caso di HRTimer si puo` specificare se il tempo
* deve essere espresso in millisecondi (default) oppure in microsecondi
* se hr_resolution=1. Abbiamo visto che tempi inferiori a 30 usec
* vengono di fatto ignorati (probabilmente dovuti alla gestione
* intrinseca dei timers hr nel kernel per i.MX28)
* 
*/
#define USE_TASKLET     1
#define USE_TIMER       2
#define USE_HRTIMER     3

static int use_method = USE_HRTIMER;

#define pdebug(lvl, fmt, args...) \
    do {\
        if (debuglevel >= lvl) {\
            printk(DEBUG_LVL "[%9lu] %s: " fmt, \
                    jiffies, __FUNCTION__, ##args);\
        }\
    }\
    while (0);

static struct uart_driver auart_driver;

enum mxs_auart_type {
	IMX28_AUART,
};

struct mxs_auart_port {
	struct uart_port port;

	#define MXS_AUART_DMA_ENABLED   0x2
	#define MXS_AUART_DMA_TX_SYNC   2  /* bit 2 */
	#define MXS_AUART_DMA_RX_READY  3  /* bit 3 */
	#define MXS_AUART_RTSCTS        4  /* bit 4 */
	unsigned long flags;
	unsigned int ctrl;
	enum mxs_auart_type devtype;

	unsigned int irq;

	struct clk *clk;
	struct device *dev;

	/* for DMA */
	struct scatterlist tx_sgl;
	struct dma_chan	*tx_dma_chan;
	void *tx_dma_buf;

	struct scatterlist rx_sgl;
	struct dma_chan	*rx_dma_chan;
	void *rx_dma_buf;

	/* Added RS485 EIA485 support in software */
	struct serial_rs485	rs485;
	int  tx_gpio;
	int  rx_gpio;
	bool has_rx_gpio;
	int  write_op;
	struct timer_list timer;
	struct tasklet_struct tasklet;
	struct hrtimer hr_timer_pre;
	struct hrtimer hr_timer_post;
	int baudrate;
	int datasize;
};

typedef struct {
	struct uart_port *port;
	unsigned long expire;
} timer_data;

static struct platform_device_id mxs_auart_devtype[] = {
	{ .name = "mxs-ekuart-imx28", .driver_data = IMX28_AUART },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, mxs_auart_devtype);

static struct of_device_id mxs_auart_dt_ids[] = {
	{
		.compatible = "fsl,imx28-ekuart",
		.data = &mxs_auart_devtype[IMX28_AUART]
	},
	{
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(of, mxs_auart_dt_ids);

#define to_auart_port(u) container_of(u, struct mxs_auart_port, port)

static void mxs_auart_tx_chars(struct mxs_auart_port *s);
static void mxs_auart_stop_tx(struct uart_port *u);
static void mxs_auart_stop_rx(struct uart_port *u);
static unsigned int mxs_auart_tx_empty(struct uart_port *u);
static struct mxs_auart_port *auart_port[MXS_AUART_PORTS];

static inline int is_imx28_auart(struct mxs_auart_port *s)
{
	return s->devtype == IMX28_AUART;
}

static inline bool auart_dma_enabled(struct mxs_auart_port *s)
{
	return s->flags & MXS_AUART_DMA_ENABLED;
}

static void mxs_rs485_start_write(struct uart_port *port)
{
	struct mxs_auart_port *up = to_auart_port(port);
	int res = 0; /* Avoid gcc warning */

	pdebug(NOISY, "Called\n");
	up->write_op = 1; /* Starting writing... */

	/* handle rs485 mode */
	if (up->rs485.flags & SER_RS485_ENABLED) {
		res = (up->rs485.flags & SER_RS485_RTS_ON_SEND) ? 1 : 0;
		pdebug(VERBOSE, "res must be: %d\n", res);
		/* Now set the transceiver in WRITE Mode */
		gpio_set_value(up->tx_gpio, res);
		if (up->has_rx_gpio) {
			pdebug(VERBOSE, "has rx gpio\n");
			// LS: EK330 has SN65HVD485ED with inverted logic
			gpio_set_value(up->rx_gpio, res);
		} else {
			pdebug(INFO, "has NO rx gpio\n");
		}
		if (up->rs485.delay_rts_before_send > 0)
			mdelay(up->rs485.delay_rts_before_send);
	}

	pdebug(NOISY, "Exit\n");
}

static void mxs_rs485_stop_write(struct uart_port *port)
{
	struct mxs_auart_port *up = to_auart_port(port);
	int res = 0; /* Avoid gcc warning */

	pdebug(NOISY, "Called\n");

	if ((up->rs485.flags & SER_RS485_ENABLED)) {
		if (use_method != USE_HRTIMER)
		{
			/* Nel metodo originale (timer e tasklet)
			 * verifico se per caso la coda non sia veramente
			 * vuota ed eventualmente se anche lo shift register
			 * non lo sia...
			 */
			while (mxs_auart_tx_empty(port) != TIOCSER_TEMT)
				udelay(1);
		}
		/* if rts not already disabled */
		res = (up->rs485.flags & SER_RS485_RTS_AFTER_SEND) ? 1 : 0;
		pdebug(VERBOSE, "res must be: %d\n", res);
		/* Nel metodo nuovo il tempo impostato per la commutazione
		 * e` gia` stato implementato nella callback hrtimer in quanto
		 * verificato nella routine uart_tx_chars(). Qui non ci sono
		 * attese.
		 */
		if (use_method != USE_HRTIMER) {
			if (up->rs485.delay_rts_after_send > 0)
				mdelay(up->rs485.delay_rts_after_send);
		}
		/* Let's set up the transceiver for reading... */
		gpio_set_value(up->tx_gpio, res);
		if (up->has_rx_gpio) {
			pdebug(VERBOSE, "has rx gpio\n");
			// LS: EK330 has SN65HVD485ED with inverted logic
			gpio_set_value(up->rx_gpio, res);
		} else {
			pdebug(INFO, "has NO rx gpio\n");
		}
	}

	up->write_op = 0; /* Stop writing */
	pdebug(NOISY, "Exit\n");
}

/* TASKLET */
static void end_of_tx_tasklet(unsigned long data)
{
	struct mxs_auart_port *up = (struct mxs_auart_port *) data;
	u32 stat = readl(up->port.membase + AUART_STAT);
	int somedata;

	pdebug(NOISY, "Called\n");
	somedata = !(uart_circ_empty(&(up->port.state->xmit))) |
               !(stat & AUART_STAT_RXFE);

	if (!somedata) {
		mxs_auart_stop_tx(&up->port);
	} else {
		tasklet_schedule(&up->tasklet);
	}
	pdebug(NOISY, "Exit\n");
}

/* TIMER */
static void end_of_tx_timer(unsigned long data)
{
	struct mxs_auart_port *up = (struct mxs_auart_port *) data;
	u32 stat = readl(up->port.membase + AUART_STAT);
	int somedata;

	pdebug(NOISY, "Called\n");
	somedata = !(uart_circ_empty(&(up->port.state->xmit))) |
               !(stat & AUART_STAT_RXFE);

	if (!somedata) {
		mxs_auart_stop_tx(&up->port);
	} else {
		/* Reload timer for next jiffy because there are some data
		 * into circular buffer */
		mod_timer(&up->timer, jiffies);
	}
	pdebug(NOISY, "Exit\n");
}

/* HRTIMER */
enum hrtimer_restart end_of_tx_hrtimer( struct hrtimer *timer )
{
	uint32_t stat;
	struct mxs_auart_port *up =
			container_of(timer, struct mxs_auart_port, hr_timer_post);

	pdebug(NOISY, "Enter\n");

	stat = readl(up->port.membase + AUART_STAT);

	if (stat & AUART_STAT_BUSY) {
		int us_to_wait;
		ktime_t ktime;

		/* Errore! Non possiamo muovere il pin di direzione per fine
		 * trasmissione! C'e` ancora qualcosa in transito sullo shift
		 * register. Ci riscateniamo tra un bitclock oppure tra 50
		 * microsecondi per sicurezza... */
		pdebug(ERROR, "Time too short for HRTIMER (%p)\n", up);

		us_to_wait = (1000000L / up->baudrate);

		if (us_to_wait < 50)
			us_to_wait = 50;

		ktime = ktime_set( 0, us_to_wait * 1000L );
		hrtimer_start( &up->hr_timer_post, ktime, HRTIMER_MODE_REL);

	} else {
		pdebug(VERBOSE, "End of tx for device (%p)\n", up);
		/* Invochiamo lo stop della trasmissione */
		mxs_auart_stop_tx(&up->port);
	}

	pdebug(NOISY, "Exit\n");
	return HRTIMER_NORESTART;
}

/* HRTIMER */
enum hrtimer_restart start_of_tx_hrtimer( struct hrtimer *timer )
{
	struct mxs_auart_port *up =
			container_of(timer, struct mxs_auart_port, hr_timer_pre);

	pdebug(NOISY,"Called for device (%p)\n", up);

	/* enable transmitter */
	writel(AUART_CTRL2_TXE, up->port.membase + AUART_CTRL2_SET);
	mxs_auart_tx_chars(up);

	pdebug(NOISY, "Exit\n");
	return HRTIMER_NORESTART;
}

static void mxs_auart_tx_chars(struct mxs_auart_port *s)
{
	struct circ_buf *xmit = &s->port.state->xmit;
	int txcount;
	uint32_t stat = readl(s->port.membase + AUART_STAT);

	pdebug(NOISY, "Called\n");

	txcount = 0;
	/* If it is not a FIFO FULL Flag, we can transmit */
	while (!(readl(s->port.membase + AUART_STAT) & AUART_STAT_TXFF)) {
		if (s->port.x_char) {
			s->port.icount.tx++;
			txcount++;
			writel(s->port.x_char, s->port.membase + AUART_DATA);
			s->port.x_char = 0;
			pdebug(VERBOSE, "[C] = %c = 0x%02x\n",
                   s->port.x_char, s->port.x_char);
			/* Last character! */
			continue;
		}
		/* If the circular buffer on xmit is not empty _AND_
		 * the uart is not stopped, get the character from tail of
		 * circular buffer and transmit it */
		if (!uart_circ_empty(xmit) && !uart_tx_stopped(&s->port)) {
			s->port.icount.tx++;
			txcount++;
			writel(xmit->buf[xmit->tail],
					 s->port.membase + AUART_DATA);
			pdebug(VERBOSE, "[CP] = %c = 0x%02x\n",
				 xmit->buf[xmit->tail],  xmit->buf[xmit->tail]);
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		} else
			break;
	}

	/* Se siamo configurati come una rs485 (cosa che normalmente e`)
	 * allora verifichiamo se dobbiamo fermare il timer che scatena
	 * il cambio di stato del bit di direzione sul transceiver
	 */
	if (s->rs485.flags & SER_RS485_ENABLED ) {
		/* Controlliamo se il timer fosse pendente e nel
		 * caso lo interrompiamo perche` abbiamo ripreso a
		 * scrivere
		 */
		if (txcount > 0) {
			if (use_method == USE_HRTIMER) {
				int rval;
				rval = hrtimer_try_to_cancel(&s->hr_timer_post);
				pdebug(VERBOSE, "HRTimer try_to_cancel() (%p) :%d\n",
					s, rval);
			}
		}
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(&s->port);

	if (uart_circ_empty(&(s->port.state->xmit))) {
		/* if there are no more character to send,
		 * disable interrupt on TX */ 
		writel(AUART_INTR_TXIEN, s->port.membase + AUART_INTR_CLR);
		if (s->rs485.flags & SER_RS485_ENABLED ) {
			/*
			 * Questi calcoli non servono nel driver precedente, ma
			 * solo in quello che utilizza l'HRTimer, ma li facciamo
			 * ugualmente...
			 */
			unsigned long us_to_wait;
			ktime_t ktime;

			/*
			 * Siamo sotto il WaterLevel mark dell FIFO (avendolo 
			 * opportunamente programmato nella routine di starup),
			 * quindi sappiamo solamente se non ce ne` nessuno,
			 * ma non sappiamo se ce ne sono 1 oppure 2.
			 * Prendiamo il caso con peggiore con 3 poiche` puo`
			 * essercene ancora una in transito sullo shift register.
			 */
			if ((stat & AUART_STAT_TXFE) == 0)
				txcount += 3;

			/*
			 * Dobbiamo muovere il pin di direzione solamente quando
			 * la FIFO e` relamente vuota ed i caratteri
			 * sono usciti dallo shift register considerando anche 
			 * il tempo programmato dal delay_rts_after_send.
			 */
			us_to_wait = ((1000000L*s->datasize +
				(s->baudrate - 1) /* arrotondiamo per eccesso */ ) /
				s->baudrate) * txcount;
			pdebug(VERBOSE, "Bitrate: %d -- Datasize: %d -- TXCOUNT: %d"
				" - RS485_DELAY_RTS_AFTER_SEND: %d\n"
				"** us_to_wait = %lu TXFE: %d\n",
				s->baudrate, s->datasize, txcount,
				s->rs485.delay_rts_after_send, us_to_wait,
				(stat & AUART_STAT_TXFE) ? 1 : 0);

			switch (use_method) {
				case USE_HRTIMER:
					us_to_wait +=
						s->rs485.delay_rts_after_send *
							(hr_resolution == MILLI ? 1000L : 1L);
					ktime = ktime_set( 0, us_to_wait * 1000L );
					hrtimer_start(
						&s->hr_timer_post, ktime, HRTIMER_MODE_REL);
					break;
				case USE_TIMER:
					mod_timer(&s->timer, jiffies);
					break;
				case USE_TASKLET:
					tasklet_schedule(&s->tasklet);
					break;
			}
		}
	} else {
		/* There are more characters to send... */
		writel(AUART_INTR_TXIEN, s->port.membase + AUART_INTR_SET);
	}

	if (uart_tx_stopped(&s->port))
		mxs_auart_stop_tx(&s->port);

	pdebug(NOISY, "Exit\n");
}

static void mxs_auart_rx_char(struct mxs_auart_port *s)
{
	int flag;
	u32 stat;
	u8 c;
	pdebug(NOISY, "Called\n");

	c = readl(s->port.membase + AUART_DATA);
	stat = readl(s->port.membase + AUART_STAT);

	flag = TTY_NORMAL;
	s->port.icount.rx++;

	if (stat & AUART_STAT_BERR) {
		s->port.icount.brk++;
		if (uart_handle_break(&s->port))
			goto out;
	} else if (stat & AUART_STAT_PERR) {
		s->port.icount.parity++;
	} else if (stat & AUART_STAT_FERR) {
		s->port.icount.frame++;
	}

	/*
	 * Mask off conditions which should be ingored.
	 */
	stat &= s->port.read_status_mask;

	if (stat & AUART_STAT_BERR) {
		flag = TTY_BREAK;
	} else if (stat & AUART_STAT_PERR)
		flag = TTY_PARITY;
	else if (stat & AUART_STAT_FERR)
		flag = TTY_FRAME;

	if (stat & AUART_STAT_OERR)
		s->port.icount.overrun++;

	if (uart_handle_sysrq_char(&s->port, c))
		goto out;

	uart_insert_char(&s->port, stat, AUART_STAT_OERR, c, flag);
out:
	writel(stat, s->port.membase + AUART_STAT);

	pdebug(NOISY, "Exit\n");
}

static void mxs_auart_rx_chars(struct mxs_auart_port *s)
{
	u32 stat = 0;
	pdebug(NOISY, "Called\n");

	for (;;) {
		stat = readl(s->port.membase + AUART_STAT);
		if (stat & AUART_STAT_RXFE)
			break;
		mxs_auart_rx_char(s);
	}

	writel(stat, s->port.membase + AUART_STAT);
	tty_flip_buffer_push(&s->port.state->port);
	pdebug(NOISY, "Exit\n");
}

static int mxs_auart_request_port(struct uart_port *u)
{
	int rval = 0;
	pdebug(NOISY, "Called\n");
	pdebug(NOISY, "Exit rval: %d\n", rval);
	return rval;
}

static int mxs_auart_verify_port(struct uart_port *u,
                struct serial_struct *ser)
{
	int rval = 0;
	pdebug(NOISY, "Called\n");
	if (u->type != PORT_UNKNOWN && u->type != PORT_IMX)
		rval = -EINVAL;
	pdebug(NOISY, "Exit: rval: %d\n", rval);
	return rval;
}

static void mxs_auart_config_port(struct uart_port *u, int flags)
{
	pdebug(NOISY, "Called\n");
	pdebug(NOISY, "Exit\n");
}

static const char *mxs_auart_type(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);
	char * type;
	pdebug(NOISY, "Called\n");
	type = (char *) dev_name(s->dev);
	pdebug(NOISY, "Exit %s\n", type);
	return (const char *) type;
}

static void mxs_auart_release_port(struct uart_port *u)
{
	pdebug(NOISY, "Called\n");
	pdebug(NOISY, "Exit\n");
}

static void mxs_auart_set_mctrl(struct uart_port *u, unsigned mctrl)
{
	struct mxs_auart_port *s = to_auart_port(u);

	u32 ctrl = readl(u->membase + AUART_CTRL2);

	pdebug(NOISY, "Called\n");

	ctrl &= ~(AUART_CTRL2_RTSEN | AUART_CTRL2_RTS);
	if (mctrl & TIOCM_RTS) {
		if (tty_port_cts_enabled(&u->state->port))
			ctrl |= AUART_CTRL2_RTSEN;
		else
			ctrl |= AUART_CTRL2_RTS;
	}

	s->ctrl = mctrl;

	writel(ctrl, u->membase + AUART_CTRL2);
	pdebug(NOISY, "Exit\n");
}

static u32 mxs_auart_get_mctrl(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);
	u32 stat = readl(u->membase + AUART_STAT);
	int ctrl2 = readl(u->membase + AUART_CTRL2);
	u32 mctrl = s->ctrl;
	pdebug(NOISY, "Called\n");

	mctrl &= ~TIOCM_CTS;
	if (stat & AUART_STAT_CTS)
		mctrl |= TIOCM_CTS;

	if (ctrl2 & AUART_CTRL2_RTS)
		mctrl |= TIOCM_RTS;

	pdebug(NOISY, "Exit 0x%08x\n", mctrl);
	return mctrl;
}

static void mxs_auart_settermios(struct uart_port *u,
             struct ktermios *termios,
             struct ktermios *old)
{
	u32 bm, ctrl, ctrl2, div;
	unsigned int cflag, baud;
	struct mxs_auart_port *up = to_auart_port(u);

	pdebug(NOISY, "Called\n");

	cflag = termios->c_cflag;

	ctrl = AUART_LINECTRL_FEN;
	ctrl2 = readl(u->membase + AUART_CTRL2);

	/* byte size */
	switch (cflag & CSIZE) {
	case CS5:
		bm = 0;
		break;
	case CS6:
		bm = 1;
		break;
	case CS7:
		bm = 2;
		break;
	case CS8:
		bm = 3;
		break;
	default:
		pdebug(ERROR, "Wrong Byte Size\n");
		return;
	}

	ctrl |= AUART_LINECTRL_WLEN(bm);

	/* parity */
	if (cflag & PARENB) {
		ctrl |= AUART_LINECTRL_PEN;
		if ((cflag & PARODD) == 0)
			ctrl |= AUART_LINECTRL_EPS;
	}

	u->read_status_mask = 0;

	if (termios->c_iflag & INPCK)
		u->read_status_mask |= AUART_STAT_PERR;
	if (termios->c_iflag & (BRKINT | PARMRK))
		u->read_status_mask |= AUART_STAT_BERR;

	/*
	 * Characters to ignore
	 */
	u->ignore_status_mask = 0;
	if (termios->c_iflag & IGNPAR)
		u->ignore_status_mask |= AUART_STAT_PERR;
	if (termios->c_iflag & IGNBRK) {
		u->ignore_status_mask |= AUART_STAT_BERR;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if (termios->c_iflag & IGNPAR)
			u->ignore_status_mask |= AUART_STAT_OERR;
	}

	/*
	 * ignore all characters if CREAD is not set
	 */
	if (cflag & CREAD)
		ctrl2 |= AUART_CTRL2_RXE;
	else
		ctrl2 &= ~AUART_CTRL2_RXE;

	/* figure out the stop bits requested */
	if (cflag & CSTOPB)
		ctrl |= AUART_LINECTRL_STP2;

	/* figure out the hardware flow control settings */
	if (cflag & CRTSCTS) {
		pdebug(VERBOSE, "RTSCTS not allowed\n");
	} else {
		pdebug(VERBOSE, "RTSCTS disabled\n");
		ctrl2 &= ~(AUART_CTRL2_CTSEN | AUART_CTRL2_RTSEN);
	}

	/* set baud rate */
	baud = uart_get_baud_rate(u, termios, old, 0, u->uartclk);

	/* Aggiorniamo i campi della struttura per poter utilizzarli
	 * nella sezione del calcolo tempistiche per hr_timer */
	up->baudrate = baud;
	up->datasize = 1;                         /* START BIT     */
	up->datasize += (bm + 5);                 /* DATA WORD LEN */
	up->datasize += (cflag & CSTOPB) ? 2 : 1; /* STOP BIT      */

	div = u->uartclk * 32 / baud;
	ctrl |= AUART_LINECTRL_BAUD_DIVFRAC(div & 0x3F);
	ctrl |= AUART_LINECTRL_BAUD_DIVINT(div >> 6);

	writel(ctrl, u->membase + AUART_LINECTRL);
	writel(ctrl2, u->membase + AUART_CTRL2);

	uart_update_timeout(u, termios->c_cflag, baud);

	pdebug(NOISY, "Exit\n");
}

static irqreturn_t mxs_auart_irq_handle(int irq, void *context)
{
	u32 istat;
	struct mxs_auart_port *s = context;
	u32 stat = readl(s->port.membase + AUART_STAT);

	istat = readl(s->port.membase + AUART_INTR);

	/* ack irq */
	writel(istat & ( AUART_INTR_RTIS |
        AUART_INTR_TXIS |
        AUART_INTR_RXIS |
        AUART_INTR_CTSMIS ),
        s->port.membase + AUART_INTR_CLR);

	if (istat & AUART_INTR_CTSMIS) {
		uart_handle_cts_change(&s->port, stat & AUART_STAT_CTS);
		writel(AUART_INTR_CTSMIS, s->port.membase + AUART_INTR_CLR);
		istat &= ~AUART_INTR_CTSMIS;
	}

	if (istat & (AUART_INTR_RTIS | AUART_INTR_RXIS)) {
		if (!auart_dma_enabled(s))
			mxs_auart_rx_chars(s);
		istat &= ~(AUART_INTR_RTIS | AUART_INTR_RXIS);
	}

	if (istat & AUART_INTR_TXIS) {
		mxs_auart_tx_chars(s);
		istat &= ~AUART_INTR_TXIS;
	}

	return IRQ_HANDLED;
}

static void mxs_auart_reset(struct uart_port *u)
{
	int i;
	unsigned int reg;

	pdebug(NOISY, "Called\n");

	writel(AUART_CTRL0_SFTRST, u->membase + AUART_CTRL0_CLR);
	for (i = 0; i < 10000; i++) {
		reg = readl(u->membase + AUART_CTRL0);
		if (!(reg & AUART_CTRL0_SFTRST))
			break;
		udelay(3);
	}
	writel(AUART_CTRL0_CLKGATE, u->membase + AUART_CTRL0_CLR);

	pdebug(NOISY, "Exit\n");
}

static int mxs_auart_startup(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);
	uint32_t ctrl2;
	int rval = 0;

	pdebug(NOISY, "Called\n");

	/* GPIOs remains allocated until driver's removal */

	clk_prepare_enable(s->clk);

	writel(AUART_CTRL0_CLKGATE, u->membase + AUART_CTRL0_CLR);

	writel(AUART_CTRL2_UARTEN, u->membase + AUART_CTRL2_SET);

	writel(AUART_INTR_RXIEN | AUART_INTR_RTIEN | AUART_INTR_CTSMIEN,
			u->membase + AUART_INTR);

	/*
	 * Enable fifo so all four bytes of a DMA word are written to
	 * output (otherwise, only the LSB is written, ie. 1 in 4 bytes)
	 */
	writel(AUART_LINECTRL_FEN, u->membase + AUART_LINECTRL_SET);

	ctrl2 = readl(u->membase + AUART_CTRL2);
	/* Azzeriamo la maschera dei bit per la selezione TXFIFOSEL */
	ctrl2 &= ~(0x07 << 16);

	if (use_method == USE_HRTIMER) {
		/* Nello specifico uso con HRTimer il watermark della FIFOTX
		 * viene configurato come (ref. UserManual)
		 * HW_UARTAPP_CTRL2
		 * Bit 18–16 Transmit Interrupt FIFO Level Select.
		 * The trigger points for the transmit interrupt are as follows:
		 * TXIFLSEL 0x0 EMPTY —
		 * Trigger on FIFO less than or equal to 2 of 16 entries.
		 */
		ctrl2 |= AUART_CTRL2_TXIFSEL(0x0);
		pdebug(VERBOSE, "TXFIFO Watermark EMPTY\n");
	} else {
		/* Il default watermark level TXFIFO se non modificato da
		 * altri e` impostato a META` FIFO:
		 * 0x2 ONE_HALF —
		 * Trigger on FIFO less than or equal to 8 of 16 entries.
		 */
		ctrl2 |= AUART_CTRL2_TXIFSEL(0x2);
		pdebug(VERBOSE, "TXFIFO Watermark ONE_HALF (default)\n");
	}

	writel(ctrl2, u->membase + AUART_CTRL2);

	pdebug(NOISY, "Exit rval: %d\n", rval);
	return rval;
}

static void mxs_auart_shutdown(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);
	pdebug(NOISY, "Called\n");

	writel(AUART_CTRL2_UARTEN, u->membase + AUART_CTRL2_CLR);

	writel(AUART_INTR_RXIEN | AUART_INTR_RTIEN | AUART_INTR_CTSMIEN,
			u->membase + AUART_INTR_CLR);

	writel(AUART_CTRL0_CLKGATE, u->membase + AUART_CTRL0_SET);

	clk_disable_unprepare(s->clk);

	/* GPIOs remains allocated until driver's removal */

	pdebug(NOISY, "Exit\n");
}

static unsigned int mxs_auart_tx_empty(struct uart_port *u)
{
	uint32_t auart_stat = readl(u->membase + AUART_STAT);
	unsigned int txfe, busy;
	unsigned int rval = 0;

	pdebug(NOISY, "Called\n");

	/* 
	 * To be sure everything is flushed outside the uart shift register
	 * and no more data is in the Transmitting FIFO I need to check
	 * both states (TXFE & !BUSY)...
	 */
	busy = auart_stat & AUART_STAT_BUSY;
	txfe = auart_stat & AUART_STAT_TXFE;

	if (txfe && !busy)
		rval = TIOCSER_TEMT;

	pdebug(VERBOSE, "returns: %s\n", rval == 0 ? "NOT EMPTY" : "EMPTY");

	pdebug(NOISY, "Exit rval: %d\n", rval);
	return rval;
}

static void mxs_auart_start_tx(struct uart_port *u)
{
	struct mxs_auart_port *s = to_auart_port(u);
	int res = 0;
	pdebug(NOISY, "Called\n");

	/* handle rs485 mode */
	if (s->rs485.flags & SER_RS485_ENABLED) {
		res = (s->rs485.flags & SER_RS485_RTS_ON_SEND) ? 1 : 0;
		pdebug(VERBOSE, "res must be: %d\n", res);
		/* Now set the transceiver in WRITE Mode */
		gpio_set_value(s->tx_gpio, res);
		if (s->has_rx_gpio) {
			pdebug(VERBOSE, "has rx gpio\n");
			gpio_set_value(s->rx_gpio, res);     // LS: EK330 has SN65HVD485ED with inverted logic
		} else {
			pdebug(VERBOSE, "has NO rx gpio\n");
		}
		if (s->rs485.delay_rts_before_send > 0) {
			if (use_method != USE_HRTIMER) {
				mxs_rs485_start_write(u);
			} else {
				ktime_t ktime;
				unsigned long usdelay =
					s->rs485.delay_rts_before_send *
					(hr_resolution == MILLI ? 1000 : 1L);
				ktime = ktime_set(0, usdelay * 1000L);
				hrtimer_start( &s->hr_timer_pre, ktime, HRTIMER_MODE_REL );
				pdebug(VERBOSE, "(HRTimer) : usec %lu -- Exit\n", usdelay);
				return;
			}
		}
	}

	/* enable transmitter */
	writel(AUART_CTRL2_TXE, u->membase + AUART_CTRL2_SET);
	mxs_auart_tx_chars(s);

	pdebug(NOISY, "Exit\n");
}

static void mxs_auart_stop_tx(struct uart_port *u)
{
	pdebug(NOISY, "Called\n");

	/* handle rs485 mode after transmitting */
	mxs_rs485_stop_write(u);
	writel(AUART_CTRL2_TXE, u->membase + AUART_CTRL2_CLR);

	pdebug(NOISY, "Exit\n");
}

static void mxs_auart_stop_rx(struct uart_port *u)
{
	pdebug(NOISY, "Called\n");

	writel(AUART_CTRL2_RXE, u->membase + AUART_CTRL2_CLR);

	pdebug(NOISY, "Exit\n");
}

static void mxs_auart_break_ctl(struct uart_port *u, int ctl)
{
	struct mxs_auart_port *up = to_auart_port(u);
	int res;

	pdebug(NOISY, "Called\n");

	if (ctl) {
		if (up->rs485.flags & SER_RS485_ENABLED) {
			res = (up->rs485.flags & SER_RS485_RTS_ON_SEND) ? 1 : 0;
			/* Set RS485 direction to output *BEFORE*
			 * writing BREAK SIGNAL */
			gpio_set_value(up->tx_gpio, res);
			if (up->has_rx_gpio) {
				// LS: EK330 has SN65HVD485ED with inverted logic
				gpio_set_value(up->rx_gpio, res);
			}
		}
		/* Effective BREAK SET */
		writel(AUART_LINECTRL_BRK, u->membase + AUART_LINECTRL_SET);
	} else {
		/* Effective BREAK CLR */
		writel(AUART_LINECTRL_BRK, u->membase + AUART_LINECTRL_CLR);
	}

	if (!ctl) {
		if (up->rs485.flags & SER_RS485_ENABLED) {
			res = (up->rs485.flags & SER_RS485_RTS_AFTER_SEND) ? 1 : 0;
			/* Set RS485 direction to input *AFTER*
			 * deasserting BREAK SIGNAL */
			gpio_set_value(up->tx_gpio, res);
			if (up->has_rx_gpio) {
				// LS: EK330 has SN65HVD485ED with inverted logic
				gpio_set_value(up->rx_gpio, res);
			}
		}
	}

	pdebug(NOISY, "Exit\n");
}

static void mxs_auart_enable_ms(struct uart_port *port)
{
	pdebug(NOISY, "Called\n");
	pdebug(NOISY, "Exit\n");
	/* just empty */
}

/* Enable or disable the rs485 support */
static void mxs_auart_config_rs485(
	struct uart_port *port, struct serial_rs485 *rs485conf)
{
	struct mxs_auart_port *up = to_auart_port(port);
	unsigned long flags;
	int val;
	pdebug(NOISY, "Called\n");

	pm_runtime_get_sync(up->dev);
	spin_lock_irqsave(&up->port.lock, flags);

	/* store new config */
	up->rs485 = *rs485conf;

	/*
	 * Just as a precaution, only allow rs485
	 * to be enabled if the gpio pin is valid
	 */
	if (gpio_is_valid(up->tx_gpio)) {
		/* enable / disable rts */
		val = (up->rs485.flags & SER_RS485_ENABLED) ?
			SER_RS485_RTS_AFTER_SEND : SER_RS485_RTS_ON_SEND;
		val = (up->rs485.flags & val) ? 1 : 0;
		gpio_set_value(up->tx_gpio, val);
		if (up->has_rx_gpio) {
			if (gpio_is_valid(up->rx_gpio)) {
				// LS: EK330 has SN65HVD485ED with inverted logic
				gpio_set_value(up->rx_gpio, val);
			}
		}
	} else {
		/* The TX_GPIO pin is not available. We can use this
		 * serial as RS485...
		 */
		up->rs485.flags &= ~SER_RS485_ENABLED;
	}

	/* This is not full duplex uart mode */
	if (up->rs485.flags & SER_RS485_RX_DURING_TX) {
		up->rs485.flags &= ~SER_RS485_RX_DURING_TX;
	}

	spin_unlock_irqrestore(&up->port.lock, flags);
	pm_runtime_mark_last_busy(up->dev);
	pm_runtime_put_autosuspend(up->dev);

	pdebug(NOISY, "Exit\n");
}

static int mxs_auart_ioctl(
	struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct serial_rs485 rs485conf;
	int rval = 0;

	pdebug(NOISY, "Called\n");

	switch (cmd) {
	case TIOCSRS485:
		if (copy_from_user(&rs485conf,
					(struct serial_rs485 *) arg,
					sizeof(rs485conf)))
			rval = -EFAULT;
		else
			mxs_auart_config_rs485(port, &rs485conf);
		break;

	case TIOCGRS485:
		if (copy_to_user((struct serial_rs485 *) arg,
					&(to_auart_port(port)->rs485),
					sizeof(rs485conf)))
			rval = -EFAULT;
		break;

	default:
		rval = -ENOIOCTLCMD;
	}

	pdebug(NOISY, "Exit %d\n", rval);
	return rval;
}

static struct uart_ops mxs_auart_ops = {
	.tx_empty       = mxs_auart_tx_empty,
	.start_tx       = mxs_auart_start_tx,
	.stop_tx        = mxs_auart_stop_tx,
	.stop_rx        = mxs_auart_stop_rx,
	.enable_ms      = mxs_auart_enable_ms,
	.break_ctl      = mxs_auart_break_ctl,
	.set_mctrl      = mxs_auart_set_mctrl,
	.get_mctrl      = mxs_auart_get_mctrl,
	.startup        = mxs_auart_startup,
	.shutdown       = mxs_auart_shutdown,
	.set_termios    = mxs_auart_settermios,
	.type           = mxs_auart_type,
	.release_port   = mxs_auart_release_port,
	.request_port   = mxs_auart_request_port,
	.config_port    = mxs_auart_config_port,
	.verify_port    = mxs_auart_verify_port,
	.ioctl          = mxs_auart_ioctl,
};

static struct uart_driver auart_driver = {
	.owner       = THIS_MODULE,
	.driver_name = "ttyEK",
	.dev_name    = "ttyEK",
	.major       = 0,
	.minor       = 0,
	.nr          = MXS_AUART_PORTS,
};

/*
* This function returns 1 if pdev isn't a device instatiated by dt, 0 if it
* could successfully get all information from dt or a negative errno.
*/
static int serial_mxs_probe_dt(struct mxs_auart_port *s,
	struct platform_device *pdev)
{
	struct serial_rs485 *rs485conf = &s->rs485;
	u32 rs485_delay[2];
	enum of_gpio_flags flags;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	pdebug(NOISY, "Called\n");

	if (!np)
		/* no device tree device */
		return 1;

	ret = of_alias_get_id(np, "serial");
	if (ret < 0) {
		pdebug(ERROR, "DT failed to get alias id: %d - Exit\n", ret);
		return ret;
	}
	s->port.line = ret;

	if (of_get_property(np, "fsl,uart-has-rtscts", NULL)) {
		pdebug(INFO, "DT has rtscts flag. Ignored\n");
	}

	if (of_property_read_bool(np, "rs485-rts-active-high")) {
		pdebug(INFO, "Has rs485-rts-active-high\n");
		rs485conf->flags |= SER_RS485_RTS_ON_SEND;
	} else {
		pdebug(INFO, "Has rs485-rts-active-low\n");
		rs485conf->flags |= SER_RS485_RTS_AFTER_SEND;
	}

	/* check for tx enable gpio */
	s->tx_gpio = of_get_named_gpio_flags(np, "rts-gpio", 0, &flags);
	if (gpio_is_valid(s->tx_gpio)) {
		ret = gpio_request(s->tx_gpio, np->name);
		if (ret < 0) {
			pdebug(ERROR, "Can't request tx_gpio: %d - Exit\n", ret);
			return ret;
		}
		ret = gpio_direction_output(s->tx_gpio,
									flags & SER_RS485_RTS_AFTER_SEND);
		if (ret < 0) {
			pdebug(ERROR, "Cannot set direction for output"
				" in TX GPIO - Exit\n");
			/* Need to free the gpio_request() */
			gpio_free(s->tx_gpio);
			return ret;
		}
	} else {
		pdebug(ERROR, "Error on tx_gpio is_valid() - Exit\n");
		return -EINVAL;
	}

	/* check for rx enable gpio */
	s->rx_gpio = of_get_named_gpio_flags(np, "cts-gpio", 0, &flags);
	if (gpio_is_valid(s->rx_gpio)) {
		ret = gpio_request(s->rx_gpio, np->name);
		if (ret < 0) {
			pdebug(ERROR, "Can't request rx_gpio %d - Exit\n", ret);
			/* Free tx_gpio because it is allocated! */
			gpio_free(s->tx_gpio);
			return ret;
		}
		ret = gpio_direction_output(s->rx_gpio,
						!(flags & SER_RS485_RTS_AFTER_SEND));
		if (ret < 0) {
			pdebug(ERROR, "Cannot set direction for output "
					"in RX GPIO - Exit\n");
			/* Free tx_gpio & rx_gpio because they are allocated! */
			gpio_free(s->tx_gpio);
			gpio_free(s->rx_gpio);
			return ret;
		}
		s->has_rx_gpio = true;
	} else {
		pdebug(INFO, "rx_gpio is not used\n");
		s->rx_gpio = -EINVAL;
		s->has_rx_gpio = false;
	}

	if (of_property_read_u32_array(np, "rs485-rts-delay",
					rs485_delay, 2) == 0) {
		rs485conf->delay_rts_before_send = rs485_delay[0];
		rs485conf->delay_rts_after_send = rs485_delay[1];
	}

	if (of_property_read_bool(np, "rs485-rx-during-tx")) {
		pdebug(ERROR, "Cannot use full-duplex! no RX during TX\n");
		rs485conf->flags &= ~SER_RS485_RX_DURING_TX;
	}

	if (of_property_read_bool(np, "linux,rs485-enabled-at-boot-time")) {
		pdebug(INFO, "Uart RS485 Enabled by default"
			" at boot-time\n");
		rs485conf->flags |= SER_RS485_ENABLED;
	}

	pdebug(NOISY, "Exiting\n");
	return 0;
}

static int mxs_auart_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
					of_match_device(mxs_auart_dt_ids, &pdev->dev);
	struct mxs_auart_port *s;
	u32 version;
	int ret = 0;
	struct resource *r;

	pdebug(NOISY, "Called\n");

	s = kzalloc(sizeof(struct mxs_auart_port), GFP_KERNEL);
	if (!s) {
		ret = -ENOMEM;
		goto out;
	}

	ret = serial_mxs_probe_dt(s, pdev);
	if (ret > 0)
		s->port.line = pdev->id < 0 ? 0 : pdev->id;
	else if (ret < 0)
		goto out_free;

	if (of_id) {
		pdev->id_entry = of_id->data;
		s->devtype = pdev->id_entry->driver_data;
	}

	s->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(s->clk)) {
		ret = PTR_ERR(s->clk);
		goto out_free;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		ret = -ENXIO;
		goto out_free_clk;
	}

	s->port.mapbase = r->start;
	s->port.membase = ioremap(r->start, resource_size(r));
	s->port.ops = &mxs_auart_ops;
	s->port.iotype = UPIO_MEM;
	s->port.fifosize = 16;
	s->port.uartclk = clk_get_rate(s->clk);
	s->port.type = PORT_IMX;
	s->port.dev = s->dev = &pdev->dev;

	s->ctrl = 0;

	s->irq = platform_get_irq(pdev, 0);
	s->port.irq = s->irq;
	ret = request_irq(s->irq, mxs_auart_irq_handle, 0, dev_name(&pdev->dev), s);
	if (ret)
		goto out_free_clk;

	platform_set_drvdata(pdev, s);

	auart_port[s->port.line] = s;

	mxs_auart_reset(&s->port);

	/* HRTIMERS SETUP */
	hrtimer_init( &s->hr_timer_pre, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	hrtimer_init( &s->hr_timer_post, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	/* HRTimers' callbacks */
	s->hr_timer_post.function = &end_of_tx_hrtimer;
	s->hr_timer_pre.function =  &start_of_tx_hrtimer;

	/* TASKLET SETUP */
	tasklet_init(&s->tasklet, end_of_tx_tasklet, (unsigned long) s);

	/* TIMER SETUP */
	init_timer(&s->timer);
	s->timer.data = (unsigned long) s;
	/* Timer's callback */
	s->timer.function = end_of_tx_timer;

	ret = uart_add_one_port(&auart_driver, &s->port);
	if (ret)
		goto out_free_irq;

	version = readl(s->port.membase + AUART_VERSION);
	dev_info(&pdev->dev, "Found EKUART %d.%d.%d (sw %s)\n"
					"\tusing method: %d, hr_resolution: %d\n",
					(version >> 24) & 0xff,
					(version >> 16) & 0xff, version & 0xffff,
					DRIVER_VERSION, use_method, hr_resolution);

	pdebug(NOISY, "Exit ret: %d\n", ret);
	return ret;

out_free_irq:
	/* Removal of kernel timer, tasklet and hrtimers */
	del_timer(&s->timer);
	tasklet_kill(&s->tasklet);
	hrtimer_cancel(&s->hr_timer_pre);
	hrtimer_cancel(&s->hr_timer_post);
	auart_port[pdev->id] = NULL;
	free_irq(s->irq, s);
out_free_clk:
	clk_put(s->clk);
out_free:
	gpio_free(s->tx_gpio);
	if (s->has_rx_gpio)
		gpio_free(s->tx_gpio);
	kfree(s);
out:
	pdebug(NOISY, "Exit Err %d\n", ret);
	return ret;
}

static int mxs_auart_remove(struct platform_device *pdev)
{
	struct mxs_auart_port *s = platform_get_drvdata(pdev);

	pdebug(NOISY, "Called\n");

	uart_remove_one_port(&auart_driver, &s->port);
	auart_port[pdev->id] = NULL;

	/* Removal of kernel timer, tasklet and hrtimers becasuse they
	 * are ALWAYS present if a driver is created. */
	tasklet_kill(&s->tasklet);
	del_timer(&s->timer);
	hrtimer_cancel(&s->hr_timer_pre);
	hrtimer_cancel(&s->hr_timer_post);

	/* Free IRQ */
	free_irq(s->irq, s);

	/* Stop Clock Tree for UART */
	clk_put(s->clk);

	/* Free GPIOs used for TX/RX directions */
	gpio_free(s->tx_gpio);
	if (s->has_rx_gpio)
		gpio_free(s->tx_gpio);

	kfree(s);

	dev_info(&pdev->dev, "Driver removed\n");

	pdebug(NOISY, "Exit\n");
	return 0;
}

static struct platform_driver mxs_auart_driver = {
	.probe = mxs_auart_probe,
	.remove = mxs_auart_remove,
	.driver = {
		.name = "mxs-ekuart",
		.owner = THIS_MODULE,
		.of_match_table = mxs_auart_dt_ids,
	},
};

static int __init mxs_auart_init(void)
{
	int r;

	pdebug(NOISY, "Called\n");

	r = uart_register_driver(&auart_driver);
	if (r)
		goto out;

	r = platform_driver_register(&mxs_auart_driver);
	if (r)
		goto out_err;

	pdebug(NOISY, "Exit\n");

	return 0;

out_err:
	uart_unregister_driver(&auart_driver);

out:
	pdebug(NOISY, "Exit Err\n");
	return r;
}

static void __exit mxs_auart_exit(void)
{
	pdebug(NOISY, "Called\n");

	platform_driver_unregister(&mxs_auart_driver);
	uart_unregister_driver(&auart_driver);

	pdebug(NOISY, "Exit\n");
}

module_init(mxs_auart_init);
module_exit(mxs_auart_exit);

module_param(debuglevel, uint, 0644);
module_param(use_method, uint, 0644);
module_param(hr_resolution, uint, 0644);

MODULE_PARM_DESC(debuglevel, "debugging level (0-4)");
MODULE_PARM_DESC(use_method, "scheduling method "
	"(1- Tasklet / 2- Timer / 3 - HRTimer)");
MODULE_PARM_DESC(hr_resolution, "Resolution HR Timer "
	"(0 - MilliSecs / 1- MicroSecs )");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gianluca Renzi <gianlucarenzi@eurek.it>"
				" & Claudio Lanconelli <claudiolanconelli@eurek.it>");
MODULE_DESCRIPTION("Freescale MXS EUREK uart driver");
MODULE_ALIAS("platform:mxs-ekuart");
