// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2020-2024 Loongson Technology Corporation Limited
 */

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/reset.h>

#include "8250.h"

/* Flags */
#define LOONGSON_UART_FRAC         BIT(0)

/* Quirks */
#define LOONGSON_UART_QUIRK_MCR    BIT(0)
#define LOONGSON_UART_QUIRK_MSR    BIT(1)

struct loongson_uart_config {
	unsigned int flags;
	unsigned int quirks;
};

struct loongson_uart_data {
	struct reset_control *rst;
	int line;
	int mcr_invert;
	int msr_invert;
	const struct loongson_uart_config *config;
};

static const struct loongson_uart_config ls3a5000_uart_config = {
	.flags = 0,
	.quirks = LOONGSON_UART_QUIRK_MCR
};

static const struct loongson_uart_config ls7a_uart_config = {
	.flags = 0,
	.quirks = LOONGSON_UART_QUIRK_MCR | LOONGSON_UART_QUIRK_MSR
};

static const struct loongson_uart_config ls2k2000_uart_config = {
	.flags = LOONGSON_UART_FRAC,
	.quirks = LOONGSON_UART_QUIRK_MCR
};

static unsigned int serial_fixup(struct uart_port *p, unsigned int offset, unsigned int val)
{
	struct loongson_uart_data *data = p->private_data;

	if (offset == UART_MCR)
		val ^= data->mcr_invert;
	if (offset == UART_MSR)
		val ^= data->msr_invert;

	return val;
}

static unsigned int loongson_serial_in(struct uart_port *p, int offset)
{
	unsigned int val, offset0 = offset;

	offset = offset << p->regshift;
	val = readb(p->membase + offset);

	return serial_fixup(p, offset0, val);
}

static void loongson_serial_out(struct uart_port *p, int offset, int value)
{
	offset = offset << p->regshift;
	writeb(serial_fixup(p, offset, value), p->membase + offset);
}

static unsigned int loongson_frac_get_divisor(struct uart_port *port,
		unsigned int baud,
		unsigned int *frac)
{
	unsigned int quot;

	quot = DIV_ROUND_CLOSEST((port->uartclk << 4), baud);
	*frac = quot & 0xff;

	return quot >> 8;
}

static void loongson_frac_set_divisor(struct uart_port *port, unsigned int baud,
		unsigned int quot, unsigned int quot_frac)
{
	struct uart_8250_port *up = up_to_u8250p(port);

	serial_port_out(port, UART_LCR, up->lcr | UART_LCR_DLAB);

	serial_dl_write(up, quot);

	serial_port_out(port, 0x2, quot_frac);
}

static int loongson_uart_probe(struct platform_device *pdev)
{
	struct uart_8250_port uart = {};
	struct loongson_uart_data *data;
	struct uart_port *port;
	struct resource *res;
	int ret;

	port = &uart.port;
	spin_lock_init(&port->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	port->flags = UPF_SHARE_IRQ | UPF_FIXED_PORT | UPF_FIXED_TYPE | UPF_IOREMAP;
	port->iotype = UPIO_MEM;
	port->regshift = 0;
	port->dev = &pdev->dev;
	port->type = PORT_LOONGSON;
	port->mapbase = res->start;
	port->mapsize = resource_size(res);
	port->serial_in = loongson_serial_in;
	port->serial_out = loongson_serial_out;

	port->irq = platform_get_irq(pdev, 0);
	if (port->irq < 0)
		return -EINVAL;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->config = device_get_match_data(&pdev->dev);

	port->private_data = data;

	if (data->config->flags & LOONGSON_UART_FRAC) {
		port->get_divisor = loongson_frac_get_divisor;
		port->set_divisor = loongson_frac_set_divisor;
	}

	if (data->config->quirks & LOONGSON_UART_QUIRK_MCR)
		data->mcr_invert |= (UART_MCR_RTS | UART_MCR_DTR);

	if (data->config->quirks & LOONGSON_UART_QUIRK_MSR)
		data->msr_invert |= (UART_MSR_CTS | UART_MSR_DSR);

	data->rst = devm_reset_control_get_optional_shared(&pdev->dev, NULL);
	if (IS_ERR(data->rst))
		return PTR_ERR(data->rst);

	device_property_read_u32(&pdev->dev, "clock-frequency", &port->uartclk);

	ret = reset_control_deassert(data->rst);
	if (ret)
		goto err_unprepare;

	ret = serial8250_register_8250_port(&uart);
	if (ret < 0)
		goto err_unprepare;

	platform_set_drvdata(pdev, data);
	data->line = ret;

	return 0;

err_unprepare:
	reset_control_assert(data->rst);

	return ret;
}

static void loongson_uart_remove(struct platform_device *pdev)
{
	struct loongson_uart_data *data = platform_get_drvdata(pdev);

	serial8250_unregister_port(data->line);
	reset_control_assert(data->rst);
}

#ifdef CONFIG_PM_SLEEP
static int loongson_uart_suspend(struct device *dev)
{
	struct loongson_uart_data *data = dev_get_drvdata(dev);

	serial8250_suspend_port(data->line);

	return 0;
}

static int loongson_uart_resume(struct device *dev)
{
	struct loongson_uart_data *data = dev_get_drvdata(dev);

	serial8250_resume_port(data->line);

	return 0;
}
#endif

static const struct dev_pm_ops loongson_uart_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(loongson_uart_suspend, loongson_uart_resume)
};

static const struct of_device_id of_platform_serial_table[] = {
	{.compatible = "loongson,ls7a-uart", .data = &ls7a_uart_config},
	{.compatible = "loongson,ls3a5000-uart", .data = &ls3a5000_uart_config},
	{.compatible = "loongson,ls2k2000-uart", .data = &ls2k2000_uart_config},
	{},
};
MODULE_DEVICE_TABLE(of, of_platform_serial_table);

static struct platform_driver loongson_uart_driver = {
	.probe = loongson_uart_probe,
	.remove = loongson_uart_remove,
	.driver = {
		.name = "loongson-uart",
		.pm = &loongson_uart_pm_ops,
		.of_match_table = of_platform_serial_table,
	},
};

module_platform_driver(loongson_uart_driver);

MODULE_DESCRIPTION("LOONGSON 8250 Driver");
MODULE_AUTHOR("Haowei Zheng <zhenghaowei@loongson.cn>");
MODULE_LICENSE("GPL");
