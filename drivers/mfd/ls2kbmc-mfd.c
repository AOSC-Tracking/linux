// SPDX-License-Identifier: GPL-2.0-only
/*
 * Loongson-2K Board Management Controller (BMC) Core Driver.
 *
 * Copyright (C) 2024-2025 Loongson Technology Corporation Limited.
 *
 * Authors:
 *	Chong Qiao <qiaochong@loongson.cn>
 *	Binbin Zhou <zhoubinbin@loongson.cn>
 */

#include <linux/aperture.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/iopoll.h>
#include <linux/kbd_kern.h>
#include <linux/kernel.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/platform_data/simplefb.h>
#include <linux/platform_device.h>
#include <linux/stop_machine.h>
#include <linux/vt_kern.h>

/* LS2K BMC resources */
#define LS2K_DISPLAY_RES_START		(SZ_16M + SZ_2M)
#define LS2K_IPMI_RES_SIZE		0x1C
#define LS2K_IPMI0_RES_START		(SZ_16M + 0xF00000)
#define LS2K_IPMI1_RES_START		(LS2K_IPMI0_RES_START + LS2K_IPMI_RES_SIZE)
#define LS2K_IPMI2_RES_START		(LS2K_IPMI1_RES_START + LS2K_IPMI_RES_SIZE)
#define LS2K_IPMI3_RES_START		(LS2K_IPMI2_RES_START + LS2K_IPMI_RES_SIZE)
#define LS2K_IPMI4_RES_START		(LS2K_IPMI3_RES_START + LS2K_IPMI_RES_SIZE)

#define LS7A_PCI_CFG_SIZE		0x100

/* LS7A bridge registers */
#define LS7A_PCIE_PORT_CTL0		0x0
#define LS7A_PCIE_PORT_STS1		0xC
#define LS7A_GEN2_CTL			0x80C
#define LS7A_SYMBOL_TIMER		0x71C

/* Bits of LS7A_PCIE_PORT_CTL0 */
#define LS2K_BMC_PCIE_LTSSM_ENABLE	BIT(3)

/* Bits of LS7A_PCIE_PORT_STS1 */
#define LS2K_BMC_PCIE_LTSSM_STS		GENMASK(5, 0)

/* Bits of LS7A_GEN2_CTL */
#define LS7A_GEN2_SPEED_CHANG		BIT(17)
#define LS7A_CONF_PHY_TX		BIT(18)

/* Bits of LS7A_SYMBOL_TIMER */
#define LS7A_MASK_LEN_MATCH		BIT(26)

/* Interval between interruptions */
#define LS2K_BMC_INT_INTERVAL		(60 * HZ)

/* Maximum time to wait for U-Boot and DDR config to be ready with ms. */
#define LS2K_BMC_RESET_WAIT_TIME	10000

/* It's an experience value */
#define LS7A_BAR0_CHECK_MAX_TIMES	2000

#define LS2K_BMC_RESET_GPIO		14
#define LOONGSON_GPIO_REG_BASE		0x1fe00500
#define LOONGSON_GPIO_REG_SIZE		0x18
#define LOONGSON_GPIO_OEN		0x0
#define LOONGSON_GPIO_FUNC		0x4
#define LOONGSON_GPIO_INTPOL		0x10
#define LOONGSON_GPIO_INTEN		0x14

static struct resource ls2k_display_resources[] = {
	DEFINE_RES_MEM_NAMED(LS2K_DISPLAY_RES_START, SZ_4M, "simpledrm-res"),
};

static struct resource ls2k_ipmi0_resources[] = {
	DEFINE_RES_MEM_NAMED(LS2K_IPMI0_RES_START, LS2K_IPMI_RES_SIZE, "ipmi0-res"),
};

static struct resource ls2k_ipmi1_resources[] = {
	DEFINE_RES_MEM_NAMED(LS2K_IPMI1_RES_START, LS2K_IPMI_RES_SIZE, "ipmi1-res"),
};

static struct resource ls2k_ipmi2_resources[] = {
	DEFINE_RES_MEM_NAMED(LS2K_IPMI2_RES_START, LS2K_IPMI_RES_SIZE, "ipmi2-res"),
};

static struct resource ls2k_ipmi3_resources[] = {
	DEFINE_RES_MEM_NAMED(LS2K_IPMI3_RES_START, LS2K_IPMI_RES_SIZE, "ipmi3-res"),
};

static struct resource ls2k_ipmi4_resources[] = {
	DEFINE_RES_MEM_NAMED(LS2K_IPMI4_RES_START, LS2K_IPMI_RES_SIZE, "ipmi4-res"),
};

static struct mfd_cell ls2k_bmc_cells[] = {
	MFD_CELL_RES("simple-framebuffer", ls2k_display_resources),
	MFD_CELL_RES("ls2k-ipmi-si", ls2k_ipmi0_resources),
	MFD_CELL_RES("ls2k-ipmi-si", ls2k_ipmi1_resources),
	MFD_CELL_RES("ls2k-ipmi-si", ls2k_ipmi2_resources),
	MFD_CELL_RES("ls2k-ipmi-si", ls2k_ipmi3_resources),
	MFD_CELL_RES("ls2k-ipmi-si", ls2k_ipmi4_resources),
};

/* Index of the bridge PCI configuration space to be restored at BMC reset. */
struct ls2k_bmc_bridge_pci_data {
	u32 pci_command;
	u32 base_address[6];
	u32 rom_addreess;
	u32 interrupt_line;
	u32 msi_hi;
	u32 msi_lo;
	u32 devctl;
	u32 linkcap;
	u32 linkctl_sts;
	u32 symbol_timer;
	u32 gen2_ctrl;
};

/* Index of the BMC PCI configuration space to be restored at BMC reset. */
struct ls2k_bmc_pci_data {
	u32 pci_command;
	u32 base_address0;
	u32 interrupt_line;
};

struct ls2k_bmc_pdata {
	struct device *dev;
	struct work_struct bmc_reset_work;
	struct ls2k_bmc_bridge_pci_data bridge_pci_data;
	struct ls2k_bmc_pci_data bmc_pci_data;
};

static bool ls2k_bmc_bar0_addr_is_set(struct pci_dev *ppdev)
{
	u32 addr;

	pci_read_config_dword(ppdev, PCI_BASE_ADDRESS_0, &addr);

	return addr & PCI_BASE_ADDRESS_MEM_MASK ? true : false;
}

static bool ls2k_bmc_check_pcie_connected(struct pci_dev *parent,
					  struct ls2k_bmc_pdata *priv)
{
	void __iomem *base;
	int sts, ret;

	base = pci_iomap(parent, 0, LS7A_PCI_CFG_SIZE);
	if (!base)
		return false;

	writel(readl(base + LS7A_PCIE_PORT_CTL0) | LS2K_BMC_PCIE_LTSSM_ENABLE,
	       base + LS7A_PCIE_PORT_CTL0);

	ret = readl_poll_timeout_atomic(base + LS7A_PCIE_PORT_STS1, sts,
					(sts & LS2K_BMC_PCIE_LTSSM_STS) == 0x11,
					1000, 1000000);
	if (ret) {
		pci_iounmap(parent, base);
		dev_err(priv->dev, "PCIE train failed status=0x%x\n", sts);
		return false;
	}

	pci_iounmap(parent, base);
	return true;
}

static void ls2k_bmc_restore_bridge_pci_data(struct pci_dev *parent, struct ls2k_bmc_pdata *priv)
{
	int base, i = 0;

	pci_write_config_dword(parent, PCI_COMMAND, priv->bridge_pci_data.pci_command);

	for (base = PCI_BASE_ADDRESS_0; base <= PCI_BASE_ADDRESS_5; base += 4, i++)
		pci_write_config_dword(parent, base, priv->bridge_pci_data.base_address[i]);

	pci_write_config_dword(parent, PCI_ROM_ADDRESS, priv->bridge_pci_data.rom_addreess);
	pci_write_config_dword(parent, PCI_INTERRUPT_LINE, priv->bridge_pci_data.interrupt_line);

	pci_write_config_dword(parent, parent->msi_cap + PCI_MSI_ADDRESS_LO,
			       priv->bridge_pci_data.msi_lo);
	pci_write_config_dword(parent, parent->msi_cap + PCI_MSI_ADDRESS_HI,
			       priv->bridge_pci_data.msi_hi);
	pci_write_config_dword(parent, parent->pcie_cap + PCI_EXP_DEVCTL,
			       priv->bridge_pci_data.devctl);
	pci_write_config_dword(parent, parent->pcie_cap + PCI_EXP_LNKCAP,
			       priv->bridge_pci_data.linkcap);
	pci_write_config_dword(parent, parent->pcie_cap + PCI_EXP_LNKCTL,
			       priv->bridge_pci_data.linkctl_sts);

	pci_write_config_dword(parent, LS7A_GEN2_CTL, priv->bridge_pci_data.gen2_ctrl);
	pci_write_config_dword(parent, LS7A_SYMBOL_TIMER, priv->bridge_pci_data.symbol_timer);
}

static int ls2k_bmc_recover_pci_data(void *data)
{
	struct ls2k_bmc_pdata *priv = data;
	struct pci_dev *pdev = to_pci_dev(priv->dev);
	struct pci_dev *parent = pdev->bus->self;
	u32 i;

	/*
	 * Clear the bus, io and mem resources of the PCI-E bridge to zero, so that
	 * the processor can not access the LS2K PCI-E port, to avoid crashing due to
	 * the lack of return signal from accessing the LS2K PCI-E port.
	 */
	pci_write_config_dword(parent, PCI_BASE_ADDRESS_2, 0);
	pci_write_config_dword(parent, PCI_BASE_ADDRESS_3, 0);
	pci_write_config_dword(parent, PCI_BASE_ADDRESS_4, 0);

	/*
	 * When the LS2K BMC is reset, the LS7A PCI-E port is also reset, and its PCI
	 * BAR0 register is cleared. Due to the time gap between the GPIO interrupt
	 * generation and the LS2K BMC reset, the LS7A PCI BAR0 register is read to
	 * determine whether the reset has begun.
	 */
	for (i = LS7A_BAR0_CHECK_MAX_TIMES; i > 0 ; i--) {
		if (!ls2k_bmc_bar0_addr_is_set(parent))
			break;
		mdelay(1);
	};

	if (i == 0)
		return false;

	ls2k_bmc_restore_bridge_pci_data(parent, priv);

	/* Check if PCI-E is connected */
	if (!ls2k_bmc_check_pcie_connected(parent, priv))
		return false;

	/* Waiting for U-Boot and DDR config ready */
	mdelay(LS2K_BMC_RESET_WAIT_TIME);
	if (!ls2k_bmc_bar0_addr_is_set(parent))
		return false;

	/* Restore LS2K BMC pci config data */
	pci_write_config_dword(pdev, PCI_COMMAND, priv->bmc_pci_data.pci_command);
	pci_write_config_dword(pdev, PCI_BASE_ADDRESS_0, priv->bmc_pci_data.base_address0);
	pci_write_config_dword(pdev, PCI_INTERRUPT_LINE, priv->bmc_pci_data.interrupt_line);

	return 0;
}

static void ls2k_bmc_events_fn(struct work_struct *work)
{
	struct ls2k_bmc_pdata *priv = container_of(work, struct ls2k_bmc_pdata, bmc_reset_work);

	/*
	 * The pcie is lost when the BMC resets, at which point access to the pcie
	 * from other CPUs is suspended to prevent a crash.
	 */
	stop_machine(ls2k_bmc_recover_pci_data, priv, NULL);

#ifdef CONFIG_VT
	/* Re-push the display due to previous pcie loss. */
	set_console(vt_move_to_console(MAX_NR_CONSOLES - 1, 1));
#endif
}

static irqreturn_t ls2k_bmc_interrupt(int irq, void *arg)
{
	struct ls2k_bmc_pdata *priv = arg;
	static unsigned long last_jiffies;

	if (system_state != SYSTEM_RUNNING)
		return IRQ_HANDLED;

	/* Skip interrupt in LS2K_BMC_INT_INTERVAL */
	if (time_after(jiffies, last_jiffies + LS2K_BMC_INT_INTERVAL)) {
		schedule_work(&priv->bmc_reset_work);
		last_jiffies = jiffies;
	}

	return IRQ_HANDLED;
}

/*
 * Saves the BMC parent device (LS7A) and its own PCI configuration space registers
 * that need to be restored after BMC reset.
 */
static void ls2k_bmc_save_pci_data(struct pci_dev *pdev, struct ls2k_bmc_pdata *priv)
{
	struct pci_dev *parent = pdev->bus->self;
	int base, i = 0;

	pci_read_config_dword(parent, PCI_COMMAND, &priv->bridge_pci_data.pci_command);

	for (base = PCI_BASE_ADDRESS_0; base <= PCI_BASE_ADDRESS_5; base += 4, i++)
		pci_read_config_dword(parent, base, &priv->bridge_pci_data.base_address[i]);

	pci_read_config_dword(parent, PCI_ROM_ADDRESS, &priv->bridge_pci_data.rom_addreess);
	pci_read_config_dword(parent, PCI_INTERRUPT_LINE, &priv->bridge_pci_data.interrupt_line);

	pci_read_config_dword(parent, parent->msi_cap + PCI_MSI_ADDRESS_LO,
			      &priv->bridge_pci_data.msi_lo);
	pci_read_config_dword(parent, parent->msi_cap + PCI_MSI_ADDRESS_HI,
			      &priv->bridge_pci_data.msi_hi);

	pci_read_config_dword(parent, parent->pcie_cap + PCI_EXP_DEVCTL,
			      &priv->bridge_pci_data.devctl);
	pci_read_config_dword(parent, parent->pcie_cap + PCI_EXP_LNKCAP,
			      &priv->bridge_pci_data.linkcap);
	pci_read_config_dword(parent, parent->pcie_cap + PCI_EXP_LNKCTL,
			      &priv->bridge_pci_data.linkctl_sts);

	pci_read_config_dword(parent, LS7A_GEN2_CTL, &priv->bridge_pci_data.gen2_ctrl);
	priv->bridge_pci_data.gen2_ctrl |= FIELD_PREP(LS7A_GEN2_SPEED_CHANG, 0x1)
					 | FIELD_PREP(LS7A_CONF_PHY_TX, 0x0);

	pci_read_config_dword(parent, LS7A_SYMBOL_TIMER, &priv->bridge_pci_data.symbol_timer);
	priv->bridge_pci_data.symbol_timer |= LS7A_MASK_LEN_MATCH;

	pci_read_config_dword(pdev, PCI_COMMAND, &priv->bmc_pci_data.pci_command);
	pci_read_config_dword(pdev, PCI_BASE_ADDRESS_0, &priv->bmc_pci_data.base_address0);
	pci_read_config_dword(pdev, PCI_INTERRUPT_LINE, &priv->bmc_pci_data.interrupt_line);
}

static int ls2k_bmc_pdata_initial(struct pci_dev *pdev, struct ls2k_bmc_pdata *priv)
{
	int gsi = 16 + (LS2K_BMC_RESET_GPIO & 7);
	void __iomem *gpio_base;
	int irq, ret;

	ls2k_bmc_save_pci_data(pdev, priv);

	INIT_WORK(&priv->bmc_reset_work, ls2k_bmc_events_fn);

	ret = devm_request_irq(&pdev->dev, pdev->irq, ls2k_bmc_interrupt,
			       IRQF_SHARED | IRQF_TRIGGER_FALLING, "ls2kbmc pcie", priv);
	if (ret) {
		dev_err(priv->dev, "LS2KBMC PCI-E request_irq(%d) failed\n", pdev->irq);
		return ret;
	}

	/*
	 * Since Loongson-3 hardware does not support GPIO interrupt cascade,
	 * chip->gpio_to_irq() cannot be implemented, here acpi_register_gsi() is used
	 * to get gpio irq.
	 * The gpio interrupt is a watchdog interrupt that is triggered when the BMC resets.
	 */
	irq = acpi_register_gsi(NULL, gsi, ACPI_EDGE_SENSITIVE, ACPI_ACTIVE_LOW);
	if (irq < 0)
		return irq;

	gpio_base = ioremap(LOONGSON_GPIO_REG_BASE, LOONGSON_GPIO_REG_SIZE);
	if (!gpio_base) {
		ret = PTR_ERR(gpio_base);
		goto acpi_failed;
	}

	writel(readl(gpio_base + LOONGSON_GPIO_OEN) | BIT(LS2K_BMC_RESET_GPIO),
	       gpio_base + LOONGSON_GPIO_OEN);
	writel(readl(gpio_base + LOONGSON_GPIO_FUNC) & ~BIT(LS2K_BMC_RESET_GPIO),
	       gpio_base + LOONGSON_GPIO_FUNC);
	writel(readl(gpio_base + LOONGSON_GPIO_INTPOL) & ~BIT(LS2K_BMC_RESET_GPIO),
	       gpio_base + LOONGSON_GPIO_INTPOL);
	writel(readl(gpio_base + LOONGSON_GPIO_INTEN) | BIT(LS2K_BMC_RESET_GPIO),
	       gpio_base + LOONGSON_GPIO_INTEN);

	ret = devm_request_irq(priv->dev, irq, ls2k_bmc_interrupt,
			       IRQF_SHARED | IRQF_TRIGGER_FALLING, "ls2kbmc gpio", priv);
	if (ret)
		dev_err(priv->dev, "LS2KBMC GPIO request_irq(%d) failed\n", irq);

	iounmap(gpio_base);

acpi_failed:
	acpi_unregister_gsi(gsi);
	return ret;
}

/*
 * Currently the Loongson-2K BMC hardware does not have an I2C interface to adapt to the
 * resolution. We set the resolution by presetting "video=1280x1024-16@2M" to the BMC memory.
 */
static int ls2k_bmc_parse_mode(struct pci_dev *pdev, struct simplefb_platform_data *pd)
{
	char *mode;
	int depth, ret;

	/* The last 16M of PCI BAR0 is used to store the resolution string. */
	mode = devm_ioremap(&pdev->dev, pci_resource_start(pdev, 0) + SZ_16M, SZ_16M);
	if (!mode)
		return -ENOMEM;

	/* The resolution field starts with the flag "video=". */
	if (!strncmp(mode, "video=", 6))
		mode = mode + 6;

	ret = kstrtoint(strsep(&mode, "x"), 10, &pd->width);
	if (ret)
		return ret;

	ret = kstrtoint(strsep(&mode, "-"), 10, &pd->height);
	if (ret)
		return ret;

	ret = kstrtoint(strsep(&mode, "@"), 10, &depth);
	if (ret)
		return ret;

	pd->stride = pd->width * depth / 8;
	pd->format = depth == 32 ? "a8r8g8b8" : "r5g6b5";

	return 0;
}

static int ls2k_bmc_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	struct simplefb_platform_data pd;
	struct ls2k_bmc_pdata *priv;
	resource_size_t base;
	int ret;

	ret = pci_enable_device(dev);
	if (ret)
		return ret;

	priv = devm_kzalloc(&dev->dev, sizeof(*priv), GFP_KERNEL);
	if (IS_ERR(priv)) {
		ret = -ENOMEM;
		goto disable_pci;
	}

	priv->dev = &dev->dev;

	ret = ls2k_bmc_pdata_initial(dev, priv);
	if (ret)
		goto disable_pci;

	ret = ls2k_bmc_parse_mode(dev, &pd);
	if (ret)
		goto disable_pci;

	ls2k_bmc_cells[0].platform_data = &pd;
	ls2k_bmc_cells[0].pdata_size = sizeof(pd);
	base = dev->resource[0].start + LS2K_DISPLAY_RES_START;

	/* Remove conflicting efifb device */
	ret = aperture_remove_conflicting_devices(base, SZ_4M, "simple-framebuffer");
	if (ret) {
		dev_err(&dev->dev, "Failed to removed firmware framebuffers: %d\n", ret);
		goto disable_pci;
	}

	return devm_mfd_add_devices(&dev->dev, PLATFORM_DEVID_AUTO,
				    ls2k_bmc_cells, ARRAY_SIZE(ls2k_bmc_cells),
				    &dev->resource[0], 0, NULL);

disable_pci:
	pci_disable_device(dev);
	return ret;
}

static void ls2k_bmc_remove(struct pci_dev *dev)
{
	pci_disable_device(dev);
}

static struct pci_device_id ls2k_bmc_devices[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_LOONGSON, 0x1a05) },
	{ }
};
MODULE_DEVICE_TABLE(pci, ls2k_bmc_devices);

static struct pci_driver ls2k_bmc_driver = {
	.name = "ls2k-bmc",
	.id_table = ls2k_bmc_devices,
	.probe = ls2k_bmc_probe,
	.remove = ls2k_bmc_remove,
};
module_pci_driver(ls2k_bmc_driver);

MODULE_DESCRIPTION("Loongson-2K BMC driver");
MODULE_AUTHOR("Loongson Technology Corporation Limited");
MODULE_LICENSE("GPL");
