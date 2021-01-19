/*
 * Apple SoC UART driver
 *
 * Copyright (C) 2020 Corellium LLC
 *
 * Based on: milbeaut_usio.c
 * Copyright (C) 2018 Socionext Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */


#include <linux/clk.h>
#include <linux/console.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>

#define REG_ULCON                       0x000
#define   REG_ULCON_IRDA                (1 << 6)
#define   REG_ULCON_PAR_EN              (1 << 5)
#define   REG_ULCON_PAR_MASK            (3 << 3)
#define     REG_ULCON_PAR_ODD           (0 << 3)
#define     REG_ULCON_PAR_EVEN          (1 << 3)
#define     REG_ULCON_PAR_ONE           (2 << 3)
#define     REG_ULCON_PAR_ZERO          (3 << 3)
#define   REG_ULCON_NSTOP               (1 << 2)
#define   REG_ULCON_WORD_MASK           (3 << 0)
#define     REG_ULCON_WORD_5            (0 << 0)
#define     REG_ULCON_WORD_6            (1 << 0)
#define     REG_ULCON_WORD_7            (2 << 0)
#define     REG_ULCON_WORD_8            (3 << 0)
#define REG_UCON                        0x004
#define   REG_UCON_UNKNOWN_20           (1 << 20)
#define   REG_UCON_AUTOBAUD_IRQ_EN      (1 << 16)
#define   REG_UCON_RX_ERR_IRQ_EN        (1 << 14)
#define   REG_UCON_TX_IRQ_EN            (1 << 13)
#define   REG_UCON_RX_IRQ_EN            (1 << 12)
#define   REG_UCON_RX_TO_DMA_SUSP       (1 << 10)
#define   REG_UCON_RX_TO_IRQ_EN         (1 << 9)
#define   REG_UCON_RX_TO_EN             (1 << 7)
#define   REG_UCON_LOOPBACK             (1 << 5)
#define   REG_UCON_BREAK                (1 << 4)
#define   REG_UCON_TX_MODE_MASK         (3 << 2)
#define     REG_UCON_TX_MODE_OFF        (0 << 2)
#define     REG_UCON_TX_MODE_PIO        (1 << 2)
#define     REG_UCON_TX_MODE_DMA        (2 << 2)
#define     REG_UCON_TX_MODE_SIO        (3 << 2)
#define   REG_UCON_RX_MODE_MASK         (3 << 0)
#define     REG_UCON_RX_MODE_OFF        (0 << 0)
#define     REG_UCON_RX_MODE_PIO        (1 << 0)
#define     REG_UCON_RX_MODE_DMA        (2 << 0)
#define     REG_UCON_RX_MODE_SIO        (3 << 0)
#define REG_UFCON                       0x008
#define   REG_UFCON_TXF_TRIG_MASK       (7 << 8)
#define     REG_UFCON_TXF_TRIG_SHIFT    (8)
#define   REG_UFCON_RXF_TRIG_MASK       (7 << 4)
#define     REG_UFCON_RXF_TRIG_SHIFT    (4)
#define   REG_UFCON_TXF_RESET           (1 << 2)
#define   REG_UFCON_RXF_RESET           (1 << 1)
#define   REG_UFCON_FIFO_EN             (1 << 0)
#define REG_UMCON                       0x00c
#define   REG_UMCON_RTS_TRIG_MASK       (7 << 5)
#define     REG_UMCON_RTS_TRIG_SHIFT    (5)
#define   REG_UMCON_AUTOFC_EN           (1 << 4)
#define   REG_UMCON_MODEM_IRQ_EN        (1 << 3)
#define   REG_UMCON_RTS                 (1 << 0)
#define REG_UTRSTAT                     0x010
#define   REG_UTRSTAT_TOFCOUNT_MASK     (255 << 16)
#define     REG_UTRSTAT_TOFCOUNT_SHIFT  (16)
#define   REG_UTRSTAT_RX_TO             (1 << 9)
#define   REG_UTRSTAT_AUTOBAUD          (1 << 8)
#define   REG_UTRSTAT_RX_ERR            (1 << 6)
#define   REG_UTRSTAT_TX                (1 << 5)
#define   REG_UTRSTAT_RX                (1 << 4)
#define   REG_UTRSTAT_RX_TO_LOW         (1 << 3)
#define   REG_UTRSTAT_TXE               (1 << 2)
#define   REG_UTRSTAT_TBE               (1 << 1)
#define   REG_UTRSTAT_RBR               (1 << 0)
#define REG_UERSTAT                     0x014
#define   REG_UERSTAT_BREAK             (1 << 3)
#define   REG_UERSTAT_FRAMERR           (1 << 2)
#define   REG_UERSTAT_PARERR            (1 << 1)
#define   REG_UERSTAT_OVRERR            (1 << 0)
#define REG_UFSTAT                      0x018
#define   REG_UFSTAT_TXFF               (1 << 9)
#define   REG_UFSTAT_RXFF               (1 << 8)
#define   REG_UFSTAT_TXF_MASK           (15 << 4)
#define     REG_UFSTAT_TXF_SHIFT        (4)
#define   REG_UFSTAT_RXF_MASK           (15 << 0)
#define     REG_UFSTAT_RXF_SHIFT        (0)
#define REG_UMSTAT                      0x01c
#define   REG_UMSTAT_DCTS               (1 << 4)
#define   REG_UMSTAT_CTS                (1 << 0)
#define REG_UTXH                        0x020
#define REG_URXH                        0x024
#define REG_UBRDIV                      0x028
#define REG_UFRACVAL                    0x02c
#define REG_UVER                        0x03c

#define APPLE_UART_NAME            "apple-uart"
#define APPLE_UART_UART_DEV_NAME   "ttyAPL"

static struct uart_port apple_uart_ports[CONFIG_SERIAL_APPLE_UART_PORTS];

static int apple_uart_irq_num[CONFIG_SERIAL_APPLE_UART_PORTS];

static inline void rmwl(volatile void __iomem *addr, u32 clr, u32 set)
{
    writel((readl(addr) & ~clr) | set, addr);
}

static void apple_uart_stop_tx(struct uart_port *port)
{
    rmwl(port->membase + REG_UCON, REG_UCON_TX_IRQ_EN, 0);
}

static void apple_uart_tx_chars(struct uart_port *port)
{
    struct circ_buf *xmit = &port->state->xmit;
    int count;
    uint32_t ufstat;

    rmwl(port->membase + REG_UCON, REG_UCON_TX_IRQ_EN, 0);

    if((uart_circ_empty(xmit) || uart_tx_stopped(port)) && !port->x_char)
        return;

    ufstat = readl(port->membase + REG_UFSTAT);
    count = port->fifosize - ((ufstat & REG_UFSTAT_TXF_MASK) >> REG_UFSTAT_TXF_SHIFT);
    if(ufstat & REG_UFSTAT_TXFF)
        count = 0;

    while(count) {
        if(port->x_char) {
            writel(port->x_char, port->membase + REG_UTXH);
            port->x_char = 0;
        } else {
            if(uart_circ_empty(xmit) || uart_tx_stopped(port))
                break;
            writel(xmit->buf[xmit->tail], port->membase + REG_UTXH);
            xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        }
        port->icount.tx++;
        count --;
    }

    if(!uart_circ_empty(xmit) || port->x_char)
        rmwl(port->membase + REG_UCON, 0, REG_UCON_TX_IRQ_EN);

    if(uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);
}

static void apple_uart_start_tx(struct uart_port *port)
{
    rmwl(port->membase + REG_UCON, 0, REG_UCON_TX_IRQ_EN);

    if(!(readl(port->membase + REG_UFSTAT) & REG_UFSTAT_TXFF))
        apple_uart_tx_chars(port);
}

static void apple_uart_stop_rx(struct uart_port *port)
{
    rmwl(port->membase + REG_UCON, REG_UCON_RX_ERR_IRQ_EN | REG_UCON_RX_IRQ_EN | REG_UCON_RX_TO_IRQ_EN | REG_UCON_RX_TO_EN, 0);
}

static void apple_uart_enable_ms(struct uart_port *port)
{
    rmwl(port->membase + REG_UCON, 0, REG_UCON_RX_ERR_IRQ_EN | REG_UCON_RX_IRQ_EN | REG_UCON_RX_TO_IRQ_EN | REG_UCON_RX_TO_EN);
}

static void apple_uart_rx_chars(struct uart_port *port)
{
    struct tty_port *ttyport = &port->state->port;
    unsigned long flag = 0;
    char ch = 0;
    uint32_t ufstat, uerstat;

    while(1) {
        ufstat = readl(port->membase + REG_UFSTAT);

        if(!(ufstat & (REG_UFSTAT_RXFF | REG_UFSTAT_RXF_MASK)))
            break;

        uerstat = readl(port->membase + REG_UERSTAT);
        ch = readl(port->membase + REG_URXH);
        flag = TTY_NORMAL;
        port->icount.rx++;

        if(uart_handle_sysrq_char(port, ch))
            continue;
        uart_insert_char(port, uerstat, REG_UERSTAT_OVRERR, ch, flag);
    }

    tty_flip_buffer_push(ttyport);
}

static void apple_uart_console_putchar(struct uart_port *port, int c);

static irqreturn_t apple_uart_irq(int irq, void *dev_id)
{
    struct uart_port *port = dev_id;
    uint32_t utrstat, utrmask = 0, ucon;

    spin_lock(&port->lock);

    utrstat = readl(port->membase + REG_UTRSTAT);
    ucon = readl(port->membase + REG_UCON);

    if(ucon & REG_UCON_RX_IRQ_EN)
        utrmask |= REG_UTRSTAT_RX;
    if(ucon & REG_UCON_RX_TO_IRQ_EN)
        utrmask |= REG_UTRSTAT_RX_TO | REG_UTRSTAT_RX_TO_LOW;
    if(ucon & REG_UCON_TX_IRQ_EN)
        utrmask |= REG_UTRSTAT_TX;
    if(ucon & REG_UCON_RX_ERR_IRQ_EN)
        utrmask |= REG_UTRSTAT_RX_ERR;
    if(ucon & REG_UCON_AUTOBAUD_IRQ_EN)
        utrmask |= REG_UTRSTAT_AUTOBAUD;
    utrstat &= utrmask;
    writel(utrstat, port->membase + REG_UTRSTAT);

    if(utrstat & REG_UTRSTAT_TX)
        apple_uart_tx_chars(port);

    if(utrstat & (REG_UTRSTAT_RX | REG_UTRSTAT_RX_TO | REG_UTRSTAT_RX_ERR))
        apple_uart_rx_chars(port);

    spin_unlock(&port->lock);

    return IRQ_HANDLED;
}

static unsigned int apple_uart_tx_empty(struct uart_port *port)
{
    if(readl(port->membase + REG_UFSTAT) & (REG_UFSTAT_TXFF | REG_UFSTAT_TXF_MASK))
        return 0;
    return 1;
}

static void apple_uart_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static unsigned int apple_uart_get_mctrl(struct uart_port *port)
{
    return TIOCM_CAR | TIOCM_DSR | TIOCM_CTS;
}

static void apple_uart_break_ctl(struct uart_port *port, int break_state)
{
    unsigned long flags;

    spin_lock_irqsave(&port->lock, flags);
    rmwl(port->membase + REG_UCON, REG_UCON_BREAK, break_state ? REG_UCON_BREAK : 0);
    spin_unlock_irqrestore(&port->lock, flags);
}

static int apple_uart_startup(struct uart_port *port)
{
    const char *portname = to_platform_device(port->dev)->name;
    unsigned long flags;
    int ret, index = port->line;

    ret = request_irq(apple_uart_irq_num[index], apple_uart_irq, 0, portname, port);
    if (ret)
            return ret;

    spin_lock_irqsave(&port->lock, flags);

    rmwl(port->membase + REG_UCON, REG_UCON_RX_MODE_MASK | REG_UCON_TX_MODE_MASK, REG_UCON_RX_MODE_PIO | REG_UCON_TX_MODE_PIO | REG_UCON_RX_TO_IRQ_EN | REG_UCON_RX_TO_EN | REG_UCON_TX_IRQ_EN | REG_UCON_RX_IRQ_EN | REG_UCON_RX_ERR_IRQ_EN);

    writel(REG_UFCON_FIFO_EN | REG_UFCON_TXF_RESET | REG_UFCON_RXF_RESET | (1 << REG_UFCON_TXF_TRIG_SHIFT) | (1 << REG_UFCON_RXF_TRIG_SHIFT), port->membase + REG_UFCON);

    spin_unlock_irqrestore(&port->lock, flags);

    return 0;
}

static void apple_uart_shutdown(struct uart_port *port)
{
    int index = port->line;

    rmwl(port->membase + REG_UCON, REG_UCON_RX_MODE_MASK | REG_UCON_TX_MODE_MASK | REG_UCON_RX_TO_IRQ_EN | REG_UCON_TX_IRQ_EN | REG_UCON_RX_IRQ_EN | REG_UCON_RX_ERR_IRQ_EN, REG_UCON_RX_MODE_OFF | REG_UCON_TX_MODE_OFF);

    free_irq(apple_uart_irq_num[index], port);
}

static void apple_uart_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
{
    unsigned long flags, baud, quot;
    uint32_t ulcon = 0;

    switch(termios->c_cflag & CSIZE) {
    case CS5:
        ulcon = REG_ULCON_WORD_5;
        break;
    case CS6:
        ulcon = REG_ULCON_WORD_6;
        break;
    case CS7:
        ulcon = REG_ULCON_WORD_7;
        break;
    case CS8:
    default:
        ulcon = REG_ULCON_WORD_8;
        break;
    }

    if(termios->c_cflag & CSTOPB)
        ulcon |= REG_ULCON_NSTOP;

    if(termios->c_cflag & PARENB) {
        ulcon |= REG_ULCON_PAR_EN;
        if(termios->c_cflag & PARODD)
            ulcon |= REG_ULCON_PAR_ODD;
        else
            ulcon |= REG_ULCON_PAR_EVEN;
    }

    baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk);
    if(baud > 1)
        quot = port->uartclk / baud;
    else
        quot = 0;

    spin_lock_irqsave(&port->lock, flags);
    uart_update_timeout(port, termios->c_cflag, baud);
    port->read_status_mask = REG_UERSTAT_OVRERR | REG_UERSTAT_FRAMERR;
    if(termios->c_iflag & INPCK)
        port->read_status_mask |= REG_UERSTAT_PARERR;

    port->ignore_status_mask = 0;
    if(termios->c_iflag & IGNPAR)
        port->ignore_status_mask |= REG_UERSTAT_PARERR;
    if(termios->c_iflag & IGNBRK)
        port->ignore_status_mask |= REG_UERSTAT_BREAK;

    writel(ulcon, port->membase + REG_ULCON);
    writel(REG_UMCON_RTS, port->membase + REG_UMCON);
    writel((quot >> 4) - 1, port->membase + REG_UBRDIV);
    writel(quot & 15, port->membase + REG_UFRACVAL);

    rmwl(port->membase + REG_UCON, REG_UCON_RX_MODE_MASK | REG_UCON_TX_MODE_MASK, REG_UCON_RX_MODE_PIO | REG_UCON_TX_MODE_PIO | REG_UCON_RX_TO_EN | REG_UCON_RX_TO_IRQ_EN | REG_UCON_TX_IRQ_EN | REG_UCON_RX_IRQ_EN | REG_UCON_RX_ERR_IRQ_EN);

    spin_unlock_irqrestore(&port->lock, flags);
}

static const char *apple_uart_type(struct uart_port *port)
{
    return ((port->type == PORT_APPLE_UART) ? APPLE_UART_NAME : NULL);
}

static void apple_uart_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE)
        port->type = PORT_APPLE_UART;
}

static const struct uart_ops apple_uart_ops = {
    .tx_empty       = apple_uart_tx_empty,
    .set_mctrl      = apple_uart_set_mctrl,
    .get_mctrl      = apple_uart_get_mctrl,
    .stop_tx        = apple_uart_stop_tx,
    .start_tx       = apple_uart_start_tx,
    .stop_rx        = apple_uart_stop_rx,
    .enable_ms      = apple_uart_enable_ms,
    .break_ctl      = apple_uart_break_ctl,
    .startup        = apple_uart_startup,
    .shutdown       = apple_uart_shutdown,
    .set_termios    = apple_uart_set_termios,
    .type           = apple_uart_type,
    .config_port    = apple_uart_config_port,
};

static void apple_uart_console_putchar(struct uart_port *port, int c)
{
    while(readl(port->membase + REG_UFSTAT) & REG_UFSTAT_TXFF)
        cpu_relax();

    writel(c, port->membase + REG_UTXH);
}

static void apple_uart_console_write(struct console *co, const char *s, unsigned int count)
{
    struct uart_port *port = &apple_uart_ports[co->index];

    uart_console_write(port, s, count, apple_uart_console_putchar);
}

static int __init apple_uart_console_setup(struct console *co, char *options)
{
    struct uart_port *port;
    int baud = 115200;
    int parity = 'n';
    int flow = 'n';
    int bits = 8;

    if (co->index >= CONFIG_SERIAL_APPLE_UART_PORTS)
        return -ENODEV;

    port = &apple_uart_ports[co->index];
    if (!port->membase)
        return -ENODEV;

    writel(REG_UFCON_FIFO_EN | REG_UFCON_TXF_RESET | REG_UFCON_RXF_RESET | (1 << REG_UFCON_TXF_TRIG_SHIFT) | (1 << REG_UFCON_RXF_TRIG_SHIFT), port->membase + REG_UFCON);

    if (options)
        uart_parse_options(options, &baud, &parity, &bits, &flow);

    return uart_set_options(port, co, baud, parity, bits, flow);
}

static struct uart_driver apple_uart_uart_driver;
static struct console apple_uart_console = {
    .name   = APPLE_UART_UART_DEV_NAME,
    .write  = apple_uart_console_write,
    .device = uart_console_device,
    .setup  = apple_uart_console_setup,
    .flags  = CON_PRINTBUFFER,
    .index  = -1,
    .data   = &apple_uart_uart_driver,
};

static int __init apple_uart_console_init(void)
{
    register_console(&apple_uart_console);
    return 0;
}
console_initcall(apple_uart_console_init);


static void apple_uart_early_console_write(struct console *co, const char *s, u_int count)
{
    struct earlycon_device *dev = co->data;

    uart_console_write(&dev->port, s, count, apple_uart_console_putchar);
}

static int __init apple_uart_early_console_setup(struct earlycon_device *device, const char *opt)
{
    if (!device->port.membase)
        return -ENODEV;
    device->con->write = apple_uart_early_console_write;

    writel(REG_UFCON_FIFO_EN | REG_UFCON_RXF_RESET, device->port.membase + REG_UFCON);

    return 0;
}

OF_EARLYCON_DECLARE(apple_uart, "apple,uart", apple_uart_early_console_setup);

static struct  uart_driver apple_uart_uart_driver = {
    .owner       = THIS_MODULE,
    .driver_name = APPLE_UART_NAME,
    .dev_name    = APPLE_UART_UART_DEV_NAME,
    .cons        = &apple_uart_console,
    .nr          = CONFIG_SERIAL_APPLE_UART_PORTS,
};

static int apple_uart_probe(struct platform_device *pdev)
{
    struct clk *clk = devm_clk_get(&pdev->dev, NULL);
    struct uart_port *port;
    struct resource *res;
    int index = 0;
    int ret;

    if (IS_ERR(clk)) {
        dev_err(&pdev->dev, "Missing clock\n");
        return PTR_ERR(clk);
    }
    ret = clk_prepare_enable(clk);
    if (ret) {
        dev_err(&pdev->dev, "Clock enable failed: %d\n", ret);
        return ret;
    }
    of_property_read_u32(pdev->dev.of_node, "index", &index);
    port = &apple_uart_ports[index];

    port->private_data = (void *)clk;
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (res == NULL) {
        dev_err(&pdev->dev, "Missing regs\n");
        ret = -ENODEV;
        goto failed;
    }
    port->membase = devm_ioremap(&pdev->dev, res->start,
                            resource_size(res));

    ret = platform_get_irq(pdev, 0);
    apple_uart_irq_num[index] = ret;

    port->irq = apple_uart_irq_num[index];
    port->uartclk = clk_get_rate(clk);
    port->fifosize = 16;
    port->iotype = UPIO_MEM32;
    port->flags = UPF_BOOT_AUTOCONF | UPF_SPD_VHI;
    port->line = index;
    port->ops = &apple_uart_ops;
    port->dev = &pdev->dev;

    ret = uart_add_one_port(&apple_uart_uart_driver, port);
    if (ret) {
        dev_err(&pdev->dev, "Adding port %d failed: %d\n", index, ret);
        goto failed;
    }

    return 0;

failed:
    clk_disable_unprepare(clk);

    return ret;
}

static int apple_uart_remove(struct platform_device *pdev)
{
    struct uart_port *port = &apple_uart_ports[pdev->id];
    struct clk *clk = port->private_data;

    uart_remove_one_port(&apple_uart_uart_driver, port);
    clk_disable_unprepare(clk);

    return 0;
}

static const struct of_device_id apple_uart_dt_ids[] = {
    { .compatible = "apple,uart" },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, apple_uart_dt_ids);

static struct platform_driver apple_uart_driver = {
    .probe          = apple_uart_probe,
    .remove         = apple_uart_remove,
    .driver         = {
        .name   = APPLE_UART_NAME,
        .of_match_table = apple_uart_dt_ids,
    },
};

static int __init apple_uart_init(void)
{
    int ret = uart_register_driver(&apple_uart_uart_driver);

    if (ret) {
        pr_err("%s: uart registration failed: %d\n", __func__, ret);
        return ret;
    }
    ret = platform_driver_register(&apple_uart_driver);
    if (ret) {
        uart_unregister_driver(&apple_uart_uart_driver);
        pr_err("%s: drv registration failed: %d\n", __func__, ret);
        return ret;
    }

    return 0;
}

static void __exit apple_uart_exit(void)
{
    platform_driver_unregister(&apple_uart_driver);
    uart_unregister_driver(&apple_uart_uart_driver);
}

module_init(apple_uart_init);
module_exit(apple_uart_exit);

MODULE_AUTHOR("Corellium LLC");
MODULE_DESCRIPTION("Apple UART Driver");
MODULE_LICENSE("GPL");
