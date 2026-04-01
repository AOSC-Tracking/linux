// SPDX-License-Identifier: GPL-2.0
/*
 * ACPI scan handler to block PNP0D10 (generic xHCI) on CIX Sky1.
 *
 * The Sky1 DSDT defines both PNP0D10 (generic xHCI) and CIXH2030/CIXH2031
 * (vendor USB wrapper/controller) for each USB port, with overlapping MMIO
 * on the xHCI register block.  The vendor driver chain (cdnsp-sky1 for
 * CIXH2030 wrapper → cdnsp-plat for CIXH2031 controller) must own the
 * hardware for proper initialization including timing prescalers, APB
 * timeout chicken bits, and PHY lifecycle management.
 *
 * Block PNP0D10 platform device creation so xhci-plat doesn't race for
 * the same MMIO region.  Same pattern as pci-sky1-acpi.c blocking PNP0A08.
 */

#ifdef CONFIG_ACPI

#include <linux/acpi.h>
#include <linux/init.h>

static bool sky1_usb_acpi_detected;

static int sky1_usb_scan_attach(struct acpi_device *adev,
				const struct acpi_device_id *id)
{
	if (!sky1_usb_acpi_detected)
		return 0;

	dev_info(&adev->dev,
		 "Sky1: blocking PNP0D10, CIXH2030/CIXH2031 will handle USB\n");
	return 1;
}

static const struct acpi_device_id sky1_usb_scan_ids[] = {
	{ "PNP0D10", 0 },
	{ },
};

static struct acpi_scan_handler sky1_usb_scan_handler = {
	.ids = sky1_usb_scan_ids,
	.attach = sky1_usb_scan_attach,
};

static int __init sky1_usb_acpi_init(void)
{
	struct acpi_table_header *header;

	if (acpi_disabled)
		return 0;

	/* Use MCFG OEM ID to identify Sky1, same as PCIe scan handler */
	if (ACPI_FAILURE(acpi_get_table(ACPI_SIG_MCFG, 0, &header)))
		return 0;

	sky1_usb_acpi_detected = !memcmp(header->oem_id, "CIXTEK", 6) &&
				  !memcmp(header->oem_table_id, "SKY1EDK2", 8);
	acpi_put_table(header);

	if (!sky1_usb_acpi_detected)
		return 0;

	pr_info("Sky1 USB: registering PNP0D10 scan handler\n");
	return acpi_scan_add_handler(&sky1_usb_scan_handler);
}
/* Must run before subsys_initcall(acpi_init) which creates platform devices */
arch_initcall(sky1_usb_acpi_init);

#endif /* CONFIG_ACPI */
