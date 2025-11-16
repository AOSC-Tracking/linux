// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Loongson Technology Corporation Limited
 */

#include <linux/pci.h>
#include <linux/vgaarb.h>
#include <linux/of_address.h>

#include <drm/drm_aperture.h>
#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_ttm.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_modeset_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>
#include <drm/drm_of.h>
#include <drm/drm_bridge_connector.h>

#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>

#include "loongson_module.h"
#include "lsdc_drv.h"
#include "lsdc_gem.h"
#include "lsdc_ttm.h"

#define DRIVER_AUTHOR               "Sui Jingfeng <suijingfeng@loongson.cn>"
#define DRIVER_NAME                 "loongson"
#define DRIVER_DESC                 "drm driver for loongson graphics"
#define DRIVER_DATE                 "20220701"
#define DRIVER_MAJOR                1
#define DRIVER_MINOR                0
#define DRIVER_PATCHLEVEL           0

DEFINE_DRM_GEM_FOPS(lsdc_gem_fops);

static const struct drm_driver lsdc_drm_driver = {
	.driver_features = DRIVER_MODESET | DRIVER_RENDER | DRIVER_GEM | DRIVER_ATOMIC,
	.fops = &lsdc_gem_fops,

	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,

	.debugfs_init = lsdc_debugfs_init,
	.dumb_create = lsdc_dumb_create,
	.dumb_map_offset = lsdc_dumb_map_offset,
	.gem_prime_import_sg_table = lsdc_prime_import_sg_table,
};

static const struct drm_mode_config_funcs lsdc_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/* Display related */

static int lsdc_attach_output(struct lsdc_device *ldev, uint32_t num_crtc);
static int lsdc_common_probe(struct lsdc_device *ldev);

static int lsdc_modeset_init(struct lsdc_device *ldev,
			     unsigned int num_crtc,
			     const struct lsdc_kms_funcs *funcs,
			     bool has_vblank)
{
	struct drm_device *ddev = &ldev->base;
	struct lsdc_display_pipe *dispipe;
	unsigned int i;
	int ret;

	for (i = 0; i < num_crtc; i++) {
		dispipe = &ldev->dispipe[i];

		/* We need an index before crtc is initialized */
		dispipe->index = i;

		ret = funcs->create_i2c(ddev, dispipe, i);
		if (ret)
			return ret;
	}

	if (ldev->has_ports_node) {
		drm_info(ddev, "Has OF graph support\n");
		ret = lsdc_attach_output(ldev, num_crtc);
		if (ret)
			return ret;
	}

	/*
	 * 之前的处理是 OF扫描到 dc之后，就按照ACPI的做法处理
	 * 这样的话就不会扫到edid（2k300 没i2c去读）
	 */
	if (!ldev->num_output) {
		for (i = 0; i < num_crtc; i++) {
			struct i2c_adapter *ddc = NULL;

			dispipe = &ldev->dispipe[i];
			if (dispipe->li2c)
				ddc = &dispipe->li2c->adapter;

			ret = funcs->output_init(ddev, dispipe, ddc, i);
			if (ret)
				return ret;

			ldev->num_output++;
		}
	}

	for (i = 0; i < num_crtc; i++) {
		dispipe = &ldev->dispipe[i];

		ret = funcs->primary_plane_init(ddev, &dispipe->primary.base, i);
		if (ret)
			return ret;

		ret = funcs->cursor_plane_init(ddev, &dispipe->cursor.base, i);
		if (ret)
			return ret;

		ret = funcs->crtc_init(ddev, &dispipe->crtc.base,
				       &dispipe->primary.base,
				       &dispipe->cursor.base,
				       i, has_vblank);
		if (ret)
			return ret;
	}

	drm_info(ddev, "Total %u outputs\n", ldev->num_output);

	return 0;
}

static const struct drm_mode_config_helper_funcs lsdc_mode_config_helper_funcs = {
	.atomic_commit_tail = drm_atomic_helper_commit_tail,
};

static int lsdc_mode_config_init(struct drm_device *ddev,
				 const struct lsdc_desc *descp)
{
	int ret;

	ret = drmm_mode_config_init(ddev);
	if (ret)
		return ret;

	ddev->mode_config.funcs = &lsdc_mode_config_funcs;
	ddev->mode_config.min_width = 1;
	ddev->mode_config.min_height = 1;
	ddev->mode_config.max_width = descp->max_width * LSDC_NUM_CRTC;
	ddev->mode_config.max_height = descp->max_height * LSDC_NUM_CRTC;
	ddev->mode_config.preferred_depth = 24;
	ddev->mode_config.prefer_shadow = 1;

	ddev->mode_config.cursor_width = descp->hw_cursor_h;
	ddev->mode_config.cursor_height = descp->hw_cursor_h;

	ddev->mode_config.helper_private = &lsdc_mode_config_helper_funcs;

	if (descp->has_vblank_counter)
		ddev->max_vblank_count = 0xffffffff;

	return ret;
}

/*
 * The GPU and display controller in the LS7A1000/LS7A2000/LS2K2000 are
 * separated PCIE devices. They are two devices, not one. Bar 2 of the GPU
 * device contains the base address and size of the VRAM, both the GPU and
 * the DC could access the on-board VRAM.
 */
static int lsdc_get_dedicated_vram(struct lsdc_device *ldev,
				   struct pci_dev *pdev_dc)
{
	struct drm_device *ddev = &ldev->base;
	struct pci_dev *pdev_gpu;
	resource_size_t base, size;

	/*
	 * The GPU has 00:06.0 as its BDF, while the DC has 00:06.1
	 * This is true for the LS7A1000, LS7A2000 and LS2K2000.
	 */
	pdev_gpu = pci_get_domain_bus_and_slot(pci_domain_nr(pdev_dc->bus),
					       pdev_dc->bus->number,
					       PCI_DEVFN(6, 0));
	if (!pdev_gpu) {
		drm_err(ddev, "No GPU device, then no VRAM\n");
		return -ENODEV;
	}

	base = pci_resource_start(pdev_gpu, 2);
	size = pci_resource_len(pdev_gpu, 2);

	ldev->vram_base = base;
	ldev->vram_size = size;
	ldev->gpu = pdev_gpu;

	drm_info(ddev, "Dedicated vram start: 0x%llx, size: %uMiB\n",
		 (u64)base, (u32)(size >> 20));

	return (size > SZ_1M) ? 0 : -ENODEV;
}

static int lsdc_of_get_reserved_mem(struct lsdc_device *ldev)
{
	struct drm_device *ddev = &ldev->base;
	unsigned long size = 0;
	struct device_node *node;
	struct resource r;
	int ret;

	if (!ddev->dev->of_node)
		return -ENOENT;

	node = of_parse_phandle(ddev->dev->of_node, "memory-region", 0);
	if (!node) {
		drm_err(ddev, "No memory-region property\n");
		return -ENOENT;
	}

	ret = of_address_to_resource(node, 0, &r);
	of_node_put(node);
	if (ret)
		return ret;

	size = r.end - r.start + 1;

	ldev->vram_base = r.start;
	ldev->vram_size = size;

	drm_info(ddev, "Using of reserved mem: %lx@%pa\n", size, &r.start);

	return 0;
}

static struct lsdc_device *
lsdc_create_device(struct device *dev,
		   const struct lsdc_desc *descp,
		const struct drm_driver *driver,
		struct pci_dev *pdev,
	struct platform_device *pfdev)
{
	struct lsdc_device *ldev;
	struct drm_device *ddev;
	int ret;

	ldev = devm_drm_dev_alloc(dev, driver, struct lsdc_device, base);
	if (IS_ERR(ldev))
		return ldev;

	ldev->dc = pdev;
	ldev->descp = descp;

	// 先注册了 dev， lsdc_common_probe里面需要扫描 OF
	ldev->dev = dev;
	ddev = &ldev->base;

	loongson_gfxpll_create(ddev, &ldev->gfxpll);

	if (descp->has_dedicated_vram)
		ret = lsdc_get_dedicated_vram(ldev, pdev);
	else {
		ret = lsdc_of_get_reserved_mem(ldev);
		if (ret) {
			dma_addr_t  daddr = 0;
			void *vaddr;
			ldev->vram_size = 0x900000;
			vaddr = dma_alloc_wc(dev, ldev->vram_size, &daddr,
						GFP_KERNEL | __GFP_NOWARN);

			ret = !vaddr;
			ldev->vram_base = daddr;
		}
	}

	if (ret) {
		drm_err(ddev, "Init VRAM failed: %d\n", ret);
		return ERR_PTR(ret);
	}

	ret = drm_aperture_remove_conflicting_framebuffers(ldev->vram_base,
							   ldev->vram_size,
							   driver);
	if (ret) {
		drm_err(ddev, "Remove firmware framebuffers failed: %d\n", ret);
		return ERR_PTR(ret);
	}

	ret = lsdc_ttm_init(ldev);
	if (ret) {
		drm_err(ddev, "Memory manager init failed: %d\n", ret);
		return ERR_PTR(ret);
	}

	lsdc_gem_init(ddev);

	/* Bar 0 of the DC device contains the MMIO register's base address */
	if (pdev)
		ldev->reg_base = pcim_iomap(pdev, 0, 0);
	else {
		struct resource *res;
		res = platform_get_resource(pfdev, IORESOURCE_MEM, 0);
		ldev->reg_base = devm_ioremap_resource(dev, res);
	}
	if (!ldev->reg_base)
		return ERR_PTR(-ENODEV);

	spin_lock_init(&ldev->reglock);

	lsdc_common_probe(ldev);

	ret = lsdc_mode_config_init(ddev, descp);
	if (ret)
		return ERR_PTR(ret);

	ret = lsdc_modeset_init(ldev, descp->num_of_crtc, descp->funcs,
				loongson_vblank);
	if (ret)
		return ERR_PTR(ret);

	drm_mode_config_reset(ddev);

	return ldev;
}

/* For multiple GPU driver instance co-exixt in the system */

static unsigned int lsdc_vga_set_decode(struct pci_dev *pdev, bool state)
{
	return VGA_RSRC_NORMAL_IO | VGA_RSRC_NORMAL_MEM;
}

static int lsdc_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	const struct lsdc_desc *descp;
	struct drm_device *ddev;
	struct lsdc_device *ldev;
	int ret;

	descp = lsdc_device_probe(pdev, ent->driver_data);

	if (to_loongson_gfx(descp)->chip_id == CHIP_LS7A1000 &&
	    !loongson_ls7a1000_support) {
		dev_info(&pdev->dev,
			"LS7A1000 support disabled or not enabled via module parameter\n");
		return -ENODEV;
	}

	if (to_loongson_gfx(descp)->chip_id == CHIP_LS7A2000 &&
	    !loongson_ls7a2000_support) {
		dev_info(&pdev->dev,
			"LS7A2000 support disabled or not enabled via module parameter\n");
		return -ENODEV;
	}

	if (IS_ERR_OR_NULL(descp))
		return -ENODEV;

	pci_set_master(pdev);

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(40));
	if (ret)
		return ret;

	ret = pcim_enable_device(pdev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "Found %s, revision: %u",
		 to_loongson_gfx(descp)->model, pdev->revision);

	if (!descp->hw_vblank_enable)
		loongson_vblank = 0;

	// ldev = lsdc_create_device(pdev, descp, &lsdc_drm_driver);
	ldev = lsdc_create_device(&pdev->dev, descp, &lsdc_drm_driver, pdev, NULL);
	if (IS_ERR(ldev))
		return PTR_ERR(ldev);

	ddev = &ldev->base;

	pci_set_drvdata(pdev, ddev);

	vga_client_register(pdev, lsdc_vga_set_decode);

	drm_kms_helper_poll_init(ddev);

	if (loongson_vblank) {
		ret = drm_vblank_init(ddev, descp->num_of_crtc);
		if (ret)
			return ret;

		ret = devm_request_irq(&pdev->dev, pdev->irq,
				       descp->funcs->irq_handler,
				       IRQF_SHARED,
				       dev_name(&pdev->dev), ddev);
		if (ret) {
			drm_err(ddev, "Failed to register interrupt: %d\n", ret);
			return ret;
		}

		drm_info(ddev, "registered irq: %u\n", pdev->irq);
	}

	ret = drm_dev_register(ddev, 0);
	if (ret)
		return ret;

	drm_fbdev_ttm_setup(ddev, 32);

	return 0;
}

static void lsdc_pci_remove(struct pci_dev *pdev)
{
	struct drm_device *ddev = pci_get_drvdata(pdev);

	drm_dev_unregister(ddev);
	drm_atomic_helper_shutdown(ddev);
}

static void lsdc_pci_shutdown(struct pci_dev *pdev)
{
	drm_atomic_helper_shutdown(pci_get_drvdata(pdev));
}

static int lsdc_drm_freeze(struct drm_device *ddev)
{
	struct lsdc_device *ldev = to_lsdc(ddev);
	struct lsdc_bo *lbo;
	int ret;

	/* unpin all of buffers in the VRAM */
	mutex_lock(&ldev->gem.mutex);
	list_for_each_entry(lbo, &ldev->gem.objects, list) {
		struct ttm_buffer_object *tbo = &lbo->tbo;
		struct ttm_resource *resource = tbo->resource;
		unsigned int pin_count = tbo->pin_count;

		drm_dbg(ddev, "bo[%p], size: %zuKiB, type: %s, pin count: %u\n",
			lbo, lsdc_bo_size(lbo) >> 10,
			lsdc_mem_type_to_str(resource->mem_type), pin_count);

		if (!pin_count)
			continue;

		if (resource->mem_type == TTM_PL_VRAM) {
			ret = lsdc_bo_reserve(lbo);
			if (unlikely(ret)) {
				drm_err(ddev, "bo reserve failed: %d\n", ret);
				continue;
			}

			do {
				lsdc_bo_unpin(lbo);
				--pin_count;
			} while (pin_count);

			lsdc_bo_unreserve(lbo);
		}
	}
	mutex_unlock(&ldev->gem.mutex);

	lsdc_bo_evict_vram(ddev);

	ret = drm_mode_config_helper_suspend(ddev);
	if (unlikely(ret)) {
		drm_err(ddev, "Freeze error: %d", ret);
		return ret;
	}

	return 0;
}

static int lsdc_drm_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *ddev = pci_get_drvdata(pdev);

	return drm_mode_config_helper_resume(ddev);
}

static int lsdc_pm_freeze(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct drm_device *ddev = pci_get_drvdata(pdev);

	return lsdc_drm_freeze(ddev);
}

static int lsdc_pm_thaw(struct device *dev)
{
	return lsdc_drm_resume(dev);
}

static int lsdc_pm_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	int error;

	error = lsdc_pm_freeze(dev);
	if (error)
		return error;

	pci_save_state(pdev);
	/* Shut down the device */
	pci_disable_device(pdev);
	pci_set_power_state(pdev, PCI_D3hot);

	return 0;
}

static int lsdc_pm_resume(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);

	pci_set_power_state(pdev, PCI_D0);

	pci_restore_state(pdev);

	if (pcim_enable_device(pdev))
		return -EIO;

	return lsdc_pm_thaw(dev);
}

static const struct dev_pm_ops lsdc_pm_ops = {
	.suspend = lsdc_pm_suspend,
	.resume = lsdc_pm_resume,
	.freeze = lsdc_pm_freeze,
	.thaw = lsdc_pm_thaw,
	.poweroff = lsdc_pm_freeze,
	.restore = lsdc_pm_resume,
};

static const struct pci_device_id lsdc_pciid_list[] = {
	{PCI_VDEVICE(LOONGSON, 0x7a06), CHIP_LS7A1000},
	{PCI_VDEVICE(LOONGSON, 0x7a36), CHIP_LS7A2000},
	{ }
};

struct pci_driver lsdc_pci_driver = {
	.name = DRIVER_NAME,
	.id_table = lsdc_pciid_list,
	.probe = lsdc_pci_probe,
	.remove = lsdc_pci_remove,
	.shutdown = lsdc_pci_shutdown,
	.driver.pm = &lsdc_pm_ops,
};

MODULE_DEVICE_TABLE(pci, lsdc_pciid_list);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

static const struct of_device_id lsdc_dt_ids[] = {
	{ .compatible = "loongson,ls2k0300-dc", .data = (void *)CHIP_LS2K0300, },
	{}
};

static int lsdc_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct lsdc_desc *descp = NULL;
	const struct of_device_id *of_id;
	struct lsdc_device *ldev;
	struct drm_device *ddev;
	int ret;

	ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "Failed to set dma mask\n");
		return ret;
	}

	of_id = of_match_device(lsdc_dt_ids, dev);
	if (of_id && of_id->data)
		descp = lsdc_device_probe(NULL, (enum loongson_chip_id)of_id->data);

	if (!descp) {
		dev_err(dev, "unknown dc chip core\n");
		return -ENOENT;
	}

	if (!descp->hw_vblank_enable)
		loongson_vblank = 0;

	ldev = lsdc_create_device(&pdev->dev, descp, &lsdc_drm_driver, NULL, pdev);
	if (IS_ERR(ldev))
		return PTR_ERR(ldev);

	ddev = &ldev->base;

	platform_set_drvdata(pdev, ddev);
	drm_kms_helper_poll_init(ddev);

	if (loongson_vblank) {
		int irq;
		ret = drm_vblank_init(ddev, descp->num_of_crtc);
		if (ret)
			return ret;

		irq = platform_get_irq(pdev, 0);
		ret = devm_request_irq(&pdev->dev, irq,
				       descp->funcs->irq_handler,
				       IRQF_SHARED /* | IRQF_TRIGGER_RISING */,
				       dev_name(&pdev->dev), ddev);
		if (ret) {
			drm_err(ddev, "Failed to register interrupt: %d\n", ret);
			return ret;
		}

		drm_info(ddev, "registered irq: %u\n", irq);
	}

	ret = drm_dev_register(ddev, 0);
	if (ret)
		return ret;

	drm_fbdev_ttm_setup(ddev, 32);

	return 0;
}

static void lsdc_platform_remove(struct platform_device *pdev)
{
	return ;
}
struct platform_driver lsdc_platform_driver = {
	.probe = lsdc_platform_probe,
	.remove = lsdc_platform_remove,
	.driver = {
		.name = "lsdc",
		.pm = &lsdc_pm_ops,
		.of_match_table = of_match_ptr(lsdc_dt_ids),
	},
};

MODULE_DEVICE_TABLE(of, lsdc_dt_ids);

/*
 * 从 5.10 移植而来
 * 这一套就包括了对 encoder connector (bridge)初始化
 * 最终作用于 lsdc_modeset_init 函数对完整链路的扫描（替代了原来的类ACPI的处理）
 * 暂时应该不支持 i2c 对 encoder connector 的沟通
 * 以后遇到实际情况再修改
 */

static void lsdc_encoder_reset(struct drm_encoder *encoder)
{
	return;
}

static const struct drm_encoder_funcs lsdc_encoder_funcs = {
	.reset = lsdc_encoder_reset,
	.destroy = drm_encoder_cleanup,
};

static int lsdc_attach_bridges(struct lsdc_device *ldev,
				struct device_node *ports,
				unsigned int i)
{
	struct lsdc_display_pipe * const dispipe = &ldev->dispipe[i];
	struct drm_device *ddev = &ldev->base;
	struct drm_bridge *bridge = NULL;
	struct drm_panel *panel = NULL;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct lsdc_output *output;
	int ret;

	ret = drm_of_find_panel_or_bridge(ports, i, 0, &panel, &bridge);

	if (panel) {
		bridge = devm_drm_panel_bridge_add_typed(ddev->dev, panel, DRM_MODE_CONNECTOR_DPI);
		drm_info(ddev, "output-%u is a DPI panel\n", i);
	}

	if (!bridge)
		return ret;

	output = devm_kzalloc(ddev->dev, sizeof(*output), GFP_KERNEL);
	if (!output)
		return -ENOMEM;

	encoder = &output->encoder;

	ret = drm_encoder_init(ddev, encoder, &lsdc_encoder_funcs,
				DRM_MODE_ENCODER_DPI, "encoder-%u", i);

	if (ret) {
		drm_err(ddev, "Failed to init encoder: %d\n", ret);
		return ret;
	}

	encoder->possible_crtcs = BIT(i);

	ret = drm_bridge_attach(encoder, bridge, NULL, 0);
	//ret = drm_bridge_attach(encoder, bridge, NULL, DRM_BRIDGE_ATTACH_NO_CONNECTOR);

	if (ret)
	{
		connector = drm_bridge_connector_init(ddev, encoder);
		if (IS_ERR(connector)) {
			drm_err(ddev, "Unable to init connector\n");
			return PTR_ERR(connector);
		}

		drm_connector_attach_encoder(connector, encoder);
	}
	drm_info(ddev, "bridge-%u attached to %s\n", i, encoder->name);

	dispipe->output = output[0];

	return 0;
}

static int lsdc_attach_output(struct lsdc_device *ldev, uint32_t num_crtc)
{
	struct drm_device *ddev = &ldev->base;
	struct device_node *ports;
	struct lsdc_display_pipe *disp;
	unsigned int i;
	int ret;

	ldev->num_output = 0;

	ports = of_get_child_by_name(ldev->dev->of_node, "ports");

	for (i = 0; i < num_crtc; i++) {
		struct drm_bridge *b;
		struct drm_panel *p;

		disp = &ldev->dispipe[i];
		disp->available = false;

		ret = drm_of_find_panel_or_bridge(ports, i, 0, &p, &b);
		if (ret) {
			if (ret == -ENODEV) {
				drm_dbg(ddev, "No active panel or bridge for port%u\n", i);
				disp->available = false;
				continue;
			}

			if (ret == -EPROBE_DEFER)
				drm_dbg(ddev, "Bridge for port%d is defer probed\n", i);

			goto RET;
		}

		disp->available = true;
		ldev->num_output++;
	}

	if (ldev->num_output == 0) {
		drm_err(ddev, "No valid output, abort\n");
		ret = -ENODEV;
		goto RET;
	}

	for (i = 0; i < num_crtc; i++) {
		disp = &ldev->dispipe[i];
		if (disp->available) {
			ret = lsdc_attach_bridges(ldev, ports, i);
			if (ret)
				goto RET;
		} else {
			drm_info(ddev, "output-%u is not available\n", i);
		}
	}

	drm_info(ddev, "number of outputs: %u\n", ldev->num_output);
RET:
	of_node_put(ports);
	return ret;
}

static void lsdc_of_probe(struct lsdc_device *ldev, struct device_node *np)
{
	struct device_node *ports;

	if (!np) {
		ldev->has_ports_node = false;
		dev_info(ldev->dev, "don't has DT support\n");
		return;
	}

	ports = of_get_child_by_name(np, "ports");
	ldev->has_ports_node = ports ? true : false;
	of_node_put(ports);
}

static int lsdc_common_probe(struct lsdc_device *ldev)
{
	lsdc_of_probe(ldev, ldev->dev->of_node);
	return 0;
}
