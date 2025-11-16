// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Loongson Technology Corporation Limited
 */

#include <linux/pci.h>
#include <linux/platform_device.h>

#include <video/nomodeset.h>

#include "loongson_module.h"

static int loongson_modeset = -1;
MODULE_PARM_DESC(modeset, "Disable/Enable modesetting");
module_param_named(modeset, loongson_modeset, int, 0400);

int loongson_vblank = 1;
MODULE_PARM_DESC(vblank, "Disable/Enable hw vblank support");
module_param_named(vblank, loongson_vblank, int, 0400);
extern struct platform_driver lsdc_platform_driver;

int loongson_ls7a1000_support = 0;
MODULE_PARM_DESC(ls7a1000_support, "7A1000 support (1 = enabled, 0 = disabled (default))");
module_param_named(ls7a1000_support, loongson_ls7a1000_support, int, 0400);

int loongson_ls7a2000_support = 0;
MODULE_PARM_DESC(ls7a2000_support, "7A2000 support (1 = enabled, 0 = disabled (default))");
module_param_named(ls7a2000_support, loongson_ls7a2000_support, int, 0400);

static int __init loongson_module_init(void)
{
	if (!loongson_modeset || video_firmware_drivers_only())
		return -ENODEV;

	platform_driver_register(&lsdc_platform_driver);
	return pci_register_driver(&lsdc_pci_driver);
}
module_init(loongson_module_init);

static void __exit loongson_module_exit(void)
{
	pci_unregister_driver(&lsdc_pci_driver);
}
module_exit(loongson_module_exit);
