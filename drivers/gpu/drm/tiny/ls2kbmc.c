// SPDX-License-Identifier: GPL-2.0-only
/*
 * DRM driver for Loongson-2K BMC display
 *
 * Copyright (C) 2024 Loongson Technology Corporation Limited.
 *
 * Based on simpledrm
 */

#include <linux/aperture.h>
#include <linux/delay.h>
#include <linux/minmax.h>
#include <linux/pci.h>
#include <linux/platform_data/simplefb.h>
#include <linux/platform_device.h>
#include <linux/stop_machine.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_state_helper.h>
#include <drm/clients/drm_client_setup.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_device.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_shmem.h>
#include <drm/drm_format_helper.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_panic.h>
#include <drm/drm_probe_helper.h>

#define BMC_RESET_DELAY	(60 * HZ)
#define BMC_RESET_WAIT	10000

static const u32 index[] = { 0x4, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24,
			     0x30, 0x3c, 0x54, 0x58, 0x78, 0x7c, 0x80 };
static const u32 cindex[] = { 0x4, 0x10, 0x3c };

struct ls2kbmc_pci_data {
	u32 d80c;
	u32 d71c;
	u32 data[14];
	u32 cdata[3];
};

struct ls2kbmc_pdata {
	struct pci_dev *pdev;
	struct drm_device *ddev;
	struct work_struct bmc_work;
	unsigned long reset_time;
	struct simplefb_platform_data pd;
	struct ls2kbmc_pci_data pci_data;
};

/*
 * Helpers for simplefb_platform_data
 */

static int
simplefb_get_validated_int(struct drm_device *dev, const char *name,
			   u32 value)
{
	if (value > INT_MAX) {
		drm_err(dev, "simplefb: invalid framebuffer %s of %u\n",
			name, value);
		return -EINVAL;
	}
	return (int)value;
}

static int
simplefb_get_validated_int0(struct drm_device *dev, const char *name,
			    u32 value)
{
	if (!value) {
		drm_err(dev, "simplefb: invalid framebuffer %s of %u\n",
			name, value);
		return -EINVAL;
	}
	return simplefb_get_validated_int(dev, name, value);
}

static const struct drm_format_info *
simplefb_get_validated_format(struct drm_device *dev, const char *format_name)
{
	static const struct simplefb_format formats[] = SIMPLEFB_FORMATS;
	const struct simplefb_format *fmt = formats;
	const struct simplefb_format *end = fmt + ARRAY_SIZE(formats);
	const struct drm_format_info *info;

	if (!format_name) {
		drm_err(dev, "simplefb: missing framebuffer format\n");
		return ERR_PTR(-EINVAL);
	}

	while (fmt < end) {
		if (!strcmp(format_name, fmt->name)) {
			info = drm_format_info(fmt->fourcc);
			if (!info)
				return ERR_PTR(-EINVAL);
			return info;
		}
		++fmt;
	}

	drm_err(dev, "simplefb: unknown framebuffer format %s\n",
		format_name);

	return ERR_PTR(-EINVAL);
}

static int
simplefb_get_width_pd(struct drm_device *dev,
		      const struct simplefb_platform_data *pd)
{
	return simplefb_get_validated_int0(dev, "width", pd->width);
}

static int
simplefb_get_height_pd(struct drm_device *dev,
		       const struct simplefb_platform_data *pd)
{
	return simplefb_get_validated_int0(dev, "height", pd->height);
}

static int
simplefb_get_stride_pd(struct drm_device *dev,
		       const struct simplefb_platform_data *pd)
{
	return simplefb_get_validated_int(dev, "stride", pd->stride);
}

static const struct drm_format_info *
simplefb_get_format_pd(struct drm_device *dev,
		       const struct simplefb_platform_data *pd)
{
	return simplefb_get_validated_format(dev, pd->format);
}

/*
 * ls2kbmc Framebuffer device
 */

struct ls2kbmc_device {
	struct drm_device dev;

	/* simplefb settings */
	struct drm_display_mode mode;
	const struct drm_format_info *format;
	unsigned int pitch;

	/* memory management */
	struct iosys_map screen_base;

	/* modesetting */
	u32 formats[8];
	struct drm_plane primary_plane;
	struct drm_crtc crtc;
	struct drm_encoder encoder;
	struct drm_connector connector;
};

static struct ls2kbmc_device *ls2kbmc_device_of_dev(struct drm_device *dev)
{
	return container_of(dev, struct ls2kbmc_device, dev);
}

/*
 * Modesetting
 */

static const u64 ls2kbmc_primary_plane_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static int ls2kbmc_primary_plane_helper_atomic_check(struct drm_plane *plane,
						     struct drm_atomic_state *state)
{
	struct drm_plane_state *new_plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_shadow_plane_state *new_shadow_plane_state =
		to_drm_shadow_plane_state(new_plane_state);
	struct drm_framebuffer *new_fb = new_plane_state->fb;
	struct drm_crtc *new_crtc = new_plane_state->crtc;
	struct drm_crtc_state *new_crtc_state = NULL;
	struct drm_device *dev = plane->dev;
	struct ls2kbmc_device *sdev = ls2kbmc_device_of_dev(dev);
	int ret;

	if (new_crtc)
		new_crtc_state = drm_atomic_get_new_crtc_state(state, new_crtc);

	ret = drm_atomic_helper_check_plane_state(new_plane_state, new_crtc_state,
						  DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING,
						  false, false);
	if (ret)
		return ret;
	else if (!new_plane_state->visible)
		return 0;

	if (new_fb->format != sdev->format) {
		void *buf;

		/* format conversion necessary; reserve buffer */
		buf = drm_format_conv_state_reserve(&new_shadow_plane_state->fmtcnv_state,
						    sdev->pitch, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;
	}

	return 0;
}

static void ls2kbmc_primary_plane_helper_atomic_update(struct drm_plane *plane,
						       struct drm_atomic_state *state)
{
	struct drm_plane_state *plane_state = drm_atomic_get_new_plane_state(state, plane);
	struct drm_plane_state *old_plane_state = drm_atomic_get_old_plane_state(state, plane);
	struct drm_shadow_plane_state *shadow_plane_state = to_drm_shadow_plane_state(plane_state);
	struct drm_framebuffer *fb = plane_state->fb;
	struct drm_device *dev = plane->dev;
	struct ls2kbmc_device *sdev = ls2kbmc_device_of_dev(dev);
	struct drm_atomic_helper_damage_iter iter;
	struct drm_rect damage;
	int ret, idx;

	ret = drm_gem_fb_begin_cpu_access(fb, DMA_FROM_DEVICE);
	if (ret)
		return;

	if (!drm_dev_enter(dev, &idx))
		goto out_drm_gem_fb_end_cpu_access;

	drm_atomic_helper_damage_iter_init(&iter, old_plane_state, plane_state);
	drm_atomic_for_each_plane_damage(&iter, &damage) {
		struct drm_rect dst_clip = plane_state->dst;
		struct iosys_map dst = sdev->screen_base;

		if (!drm_rect_intersect(&dst_clip, &damage))
			continue;

		iosys_map_incr(&dst, drm_fb_clip_offset(sdev->pitch, sdev->format, &dst_clip));
		drm_fb_blit(&dst, &sdev->pitch, sdev->format->format, shadow_plane_state->data,
			    fb, &damage, &shadow_plane_state->fmtcnv_state);
	}

	drm_dev_exit(idx);
out_drm_gem_fb_end_cpu_access:
	drm_gem_fb_end_cpu_access(fb, DMA_FROM_DEVICE);
}

static void ls2kbmc_primary_plane_helper_atomic_disable(struct drm_plane *plane,
							struct drm_atomic_state *state)
{
	struct drm_device *dev = plane->dev;
	struct ls2kbmc_device *sdev = ls2kbmc_device_of_dev(dev);
	int idx;

	if (!drm_dev_enter(dev, &idx))
		return;

	/* Clear screen to black if disabled */
	iosys_map_memset(&sdev->screen_base, 0, 0, sdev->pitch * sdev->mode.vdisplay);

	drm_dev_exit(idx);
}

static int ls2kbmc_primary_plane_helper_get_scanout_buffer(struct drm_plane *plane,
							   struct drm_scanout_buffer *sb)
{
	struct ls2kbmc_device *sdev = ls2kbmc_device_of_dev(plane->dev);

	sb->width = sdev->mode.hdisplay;
	sb->height = sdev->mode.vdisplay;
	sb->format = sdev->format;
	sb->pitch[0] = sdev->pitch;
	sb->map[0] = sdev->screen_base;

	return 0;
}

static const struct drm_plane_helper_funcs ls2kbmc_primary_plane_helper_funcs = {
	DRM_GEM_SHADOW_PLANE_HELPER_FUNCS,
	.atomic_check = ls2kbmc_primary_plane_helper_atomic_check,
	.atomic_update = ls2kbmc_primary_plane_helper_atomic_update,
	.atomic_disable = ls2kbmc_primary_plane_helper_atomic_disable,
	.get_scanout_buffer = ls2kbmc_primary_plane_helper_get_scanout_buffer,
};

static const struct drm_plane_funcs ls2kbmc_primary_plane_funcs = {
	.update_plane = drm_atomic_helper_update_plane,
	.disable_plane = drm_atomic_helper_disable_plane,
	.destroy = drm_plane_cleanup,
	DRM_GEM_SHADOW_PLANE_FUNCS,
};

static enum drm_mode_status ls2kbmc_crtc_helper_mode_valid(struct drm_crtc *crtc,
							   const struct drm_display_mode *mode)
{
	struct ls2kbmc_device *sdev = ls2kbmc_device_of_dev(crtc->dev);

	return drm_crtc_helper_mode_valid_fixed(crtc, mode, &sdev->mode);
}

/*
 * The CRTC is always enabled. Screen updates are performed by
 * the primary plane's atomic_update function. Disabling clears
 * the screen in the primary plane's atomic_disable function.
 */
static const struct drm_crtc_helper_funcs ls2kbmc_crtc_helper_funcs = {
	.mode_valid = ls2kbmc_crtc_helper_mode_valid,
	.atomic_check = drm_crtc_helper_atomic_check,
};

static const struct drm_crtc_funcs ls2kbmc_crtc_funcs = {
	.reset = drm_atomic_helper_crtc_reset,
	.destroy = drm_crtc_cleanup,
	.set_config = drm_atomic_helper_set_config,
	.page_flip = drm_atomic_helper_page_flip,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_crtc_destroy_state,
};

static const struct drm_encoder_funcs ls2kbmc_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static int ls2kbmc_connector_helper_get_modes(struct drm_connector *connector)
{
	struct ls2kbmc_device *sdev = ls2kbmc_device_of_dev(connector->dev);

	return drm_connector_helper_get_modes_fixed(connector, &sdev->mode);
}

static const struct drm_connector_helper_funcs ls2kbmc_connector_helper_funcs = {
	.get_modes = ls2kbmc_connector_helper_get_modes,
};

static const struct drm_connector_funcs ls2kbmc_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static const struct drm_mode_config_funcs ls2kbmc_mode_config_funcs = {
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

/*
 * Init / Cleanup
 */

static struct drm_display_mode ls2kbmc_mode(unsigned int width, unsigned int height,
					    unsigned int width_mm, unsigned int height_mm)
{
	const struct drm_display_mode mode = {
		DRM_MODE_INIT(60, width, height, width_mm, height_mm)
	};

	return mode;
}

/*
 * DRM driver
 */

DEFINE_DRM_GEM_FOPS(ls2kbmc_fops);

static struct drm_driver ls2kbmc_driver = {
	DRM_GEM_SHMEM_DRIVER_OPS,
	DRM_FBDEV_SHMEM_DRIVER_OPS,
	.name			= "simpledrm",
	.desc			= "DRM driver for Loongson-2K BMC",
	.date			= "20241211",
	.major			= 1,
	.minor			= 0,
	.driver_features	= DRIVER_ATOMIC | DRIVER_GEM | DRIVER_MODESET,
	.fops			= &ls2kbmc_fops,
};

/*
 * Currently the Loongson-2K0500 BMC hardware does not have an i2c interface to
 * adapt to the resolution.
 * We set the resolution by presetting "video=1280x1024-16@2M" to the bmc memory.
 */
static int ls2kbmc_get_video_mode(struct pci_dev *pdev, struct simplefb_platform_data *pd)
{
	char *mode;
	int depth, ret;

	/* The pci mem bar last 16M is used to store the string. */
	mode = devm_ioremap(&pdev->dev, pci_resource_start(pdev, 0) + SZ_16M, SZ_16M);
	if (!mode)
		return -ENOMEM;

	/*env at last 16M's beginning, first env is video */
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

static struct ls2kbmc_device *ls2kbmc_device_create(struct drm_driver *drv,
						    struct platform_device *pdev,
						    struct ls2kbmc_pdata *priv)
{
	struct pci_dev *ppdev = priv->pdev;
	struct simplefb_platform_data *pd = &priv->pd;
	struct ls2kbmc_device *sdev;
	struct drm_device *dev;
	int width, height, stride;
	int width_mm = 0, height_mm = 0;
	const struct drm_format_info *format;
	struct resource *res, *mem = NULL;
	struct drm_plane *primary_plane;
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	struct drm_connector *connector;
	unsigned long max_width, max_height;
	void __iomem *screen_base;
	size_t nformats;
	int ret;

	sdev = devm_drm_dev_alloc(&pdev->dev, drv, struct ls2kbmc_device, dev);
	if (IS_ERR(sdev))
		return ERR_CAST(sdev);
	dev = &sdev->dev;
	platform_set_drvdata(pdev, sdev);

	ret = ls2kbmc_get_video_mode(ppdev, pd);
	if (ret) {
		drm_err(dev, "no simplefb configuration found\n");
		return ERR_PTR(ret);
	}

	width = simplefb_get_width_pd(dev, pd);
	if (width < 0)
		return ERR_PTR(width);

	height = simplefb_get_height_pd(dev, pd);
	if (height < 0)
		return ERR_PTR(height);

	stride = simplefb_get_stride_pd(dev, pd);
	if (stride < 0)
		return ERR_PTR(stride);

	if (!stride) {
		stride = drm_format_info_min_pitch(format, 0, width);
		if (drm_WARN_ON(dev, !stride))
			return ERR_PTR(-EINVAL);
	}

	format = simplefb_get_format_pd(dev, pd);
	if (IS_ERR(format))
		return ERR_CAST(format);

	/*
	 * Assume a monitor resolution of 96 dpi if physical dimensions
	 * are not specified to get a somewhat reasonable screen size.
	 */
	if (!width_mm)
		width_mm = DRM_MODE_RES_MM(width, 96ul);
	if (!height_mm)
		height_mm = DRM_MODE_RES_MM(height, 96ul);

	sdev->mode = ls2kbmc_mode(width, height, width_mm, height_mm);
	sdev->format = format;
	sdev->pitch = stride;

	drm_dbg(dev, "display mode={" DRM_MODE_FMT "}\n", DRM_MODE_ARG(&sdev->mode));
	drm_dbg(dev, "framebuffer format=%p4cc, size=%dx%d, stride=%d byte\n",
		&format->format, width, height, stride);

	/*
	 * Memory management
	 */

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return ERR_PTR(-EINVAL);

	ret = aperture_remove_conflicting_pci_devices(ppdev, ls2kbmc_driver.name);
	if (ret) {
		drm_err(dev, "could not acquire memory range %pr: %d\n", res, ret);
		return ERR_PTR(ret);
	}

	drm_dbg(dev, "using I/O memory framebuffer at %pr\n", res);

	mem = devm_request_mem_region(&ppdev->dev, res->start, resource_size(res),
				      drv->name);
	if (!mem) {
		/*
		 * We cannot make this fatal. Sometimes this comes from magic
		 * spaces our resource handlers simply don't know about. Use
		 * the I/O-memory resource as-is and try to map that instead.
		 */
		drm_warn(dev, "could not acquire memory region %pr\n", res);
		mem = res;
	}

	screen_base = devm_ioremap_wc(&ppdev->dev, mem->start, resource_size(mem));
	if (!screen_base)
		return ERR_PTR(-ENOMEM);

	iosys_map_set_vaddr_iomem(&sdev->screen_base, screen_base);

	/*
	 * Modesetting
	 */

	ret = drmm_mode_config_init(dev);
	if (ret)
		return ERR_PTR(ret);

	max_width = max_t(unsigned long, width, DRM_SHADOW_PLANE_MAX_WIDTH);
	max_height = max_t(unsigned long, height, DRM_SHADOW_PLANE_MAX_HEIGHT);

	dev->mode_config.min_width = width;
	dev->mode_config.max_width = max_width;
	dev->mode_config.min_height = height;
	dev->mode_config.max_height = max_height;
	dev->mode_config.preferred_depth = format->depth;
	dev->mode_config.funcs = &ls2kbmc_mode_config_funcs;

	/* Primary plane */

	nformats = drm_fb_build_fourcc_list(dev, &format->format, 1,
					    sdev->formats, ARRAY_SIZE(sdev->formats));

	primary_plane = &sdev->primary_plane;
	ret = drm_universal_plane_init(dev, primary_plane, 0, &ls2kbmc_primary_plane_funcs,
				       sdev->formats, nformats,
				       ls2kbmc_primary_plane_format_modifiers,
				       DRM_PLANE_TYPE_PRIMARY, NULL);
	if (ret)
		return ERR_PTR(ret);
	drm_plane_helper_add(primary_plane, &ls2kbmc_primary_plane_helper_funcs);
	drm_plane_enable_fb_damage_clips(primary_plane);

	/* CRTC */

	crtc = &sdev->crtc;
	ret = drm_crtc_init_with_planes(dev, crtc, primary_plane, NULL,
					&ls2kbmc_crtc_funcs, NULL);
	if (ret)
		return ERR_PTR(ret);
	drm_crtc_helper_add(crtc, &ls2kbmc_crtc_helper_funcs);

	/* Encoder */

	encoder = &sdev->encoder;
	ret = drm_encoder_init(dev, encoder, &ls2kbmc_encoder_funcs,
			       DRM_MODE_ENCODER_NONE, NULL);
	if (ret)
		return ERR_PTR(ret);
	encoder->possible_crtcs = drm_crtc_mask(crtc);

	/* Connector */

	connector = &sdev->connector;
	ret = drm_connector_init(dev, connector, &ls2kbmc_connector_funcs,
				 DRM_MODE_CONNECTOR_Unknown);
	if (ret)
		return ERR_PTR(ret);
	drm_connector_helper_add(connector, &ls2kbmc_connector_helper_funcs);
	drm_connector_set_panel_orientation_with_quirk(connector,
						       DRM_MODE_PANEL_ORIENTATION_UNKNOWN,
						       width, height);

	ret = drm_connector_attach_encoder(connector, encoder);
	if (ret)
		return ERR_PTR(ret);

	drm_mode_config_reset(dev);

	return sdev;
}

static bool ls2kbmc_bar0_addr_is_set(struct pci_dev *ppdev)
{
	u32 addr;

	pci_read_config_dword(ppdev, PCI_BASE_ADDRESS_0, &addr);
	addr &= PCI_BASE_ADDRESS_MEM_MASK;

	return addr ? true : false;
}

static void ls2kbmc_save_pci_data(struct ls2kbmc_pdata *priv)
{
	struct pci_dev *pdev = priv->pdev;
	struct pci_dev *parent = pdev->bus->self;
	int i;

	for (i = 0; i < ARRAY_SIZE(index); i++)
		pci_read_config_dword(parent, index[i], &priv->pci_data.data[i]);

	for (i = 0; i < ARRAY_SIZE(cindex); i++)
		pci_read_config_dword(pdev, cindex[i], &priv->pci_data.cdata[i]);

	pci_read_config_dword(parent, 0x80c, &priv->pci_data.d80c);
	priv->pci_data.d80c = (priv->pci_data.d80c & ~(3 << 17)) | (1 << 17);

	pci_read_config_dword(parent, 0x71c, &priv->pci_data.d71c);
	priv->pci_data.d71c |= 1 << 26;
}

static bool ls2kbmc_check_pcie_connected(struct pci_dev *parent, struct drm_device *dev)
{
	void __iomem *mmio;
	int sts, timeout = 10000;

	mmio = pci_iomap(parent, 0, 0x100);
	if (!mmio)
		return false;

	writel(readl(mmio) | 0x8, mmio);
	while (timeout) {
		sts = readl(mmio + 0xc);
		if ((sts & 0x11) == 0x11)
			break;
		mdelay(1);
		timeout--;
	}

	pci_iounmap(parent, mmio);

	if (!timeout) {
		drm_err(dev, "pcie train failed status=0x%x\n", sts);
		return false;
	}

	return true;
}

static int ls2kbmc_recove_pci_data(void *data)
{
	struct ls2kbmc_pdata *priv = data;
	struct pci_dev *pdev = priv->pdev;
	struct drm_device *dev = priv->ddev;
	struct pci_dev *parent = pdev->bus->self;
	u32 i, timeout, retry = 0;
	bool ready;

	pci_write_config_dword(parent, PCI_BASE_ADDRESS_2, 0);
	pci_write_config_dword(parent, PCI_BASE_ADDRESS_3, 0);
	pci_write_config_dword(parent, PCI_BASE_ADDRESS_4, 0);

	timeout = 10000;
	while (timeout) {
		ready = ls2kbmc_bar0_addr_is_set(parent);
		if (!ready)
			break;
		mdelay(1);
		timeout--;
	};

	if (!timeout)
		drm_warn(dev, "bar not clear 0\n");

retrain:
	for (i = 0; i < ARRAY_SIZE(index); i++)
		pci_write_config_dword(parent, index[i], priv->pci_data.data[i]);

	pci_write_config_dword(parent, 0x80c, priv->pci_data.d80c);
	pci_write_config_dword(parent, 0x71c, priv->pci_data.d71c);

	/* Check if the pcie is connected */
	ready = ls2kbmc_check_pcie_connected(parent, dev);
	if (!ready)
		return ready;

	for (i = 0; i < ARRAY_SIZE(cindex); i++)
		pci_write_config_dword(pdev, cindex[i], priv->pci_data.cdata[i]);

	drm_info(dev, "pcie recovered done\n");

	if (!retry) {
		/*wait u-boot ddr config */
		mdelay(BMC_RESET_WAIT);
		ready = ls2kbmc_bar0_addr_is_set(parent);
		if (!ready) {
			retry = 1;
			goto retrain;
		}
	}

	return 0;
}

static int ls2kbmc_push_drm_mode(struct drm_device *dev)
{
	struct ls2kbmc_device *sdev = ls2kbmc_device_of_dev(dev);
	struct drm_crtc *crtc = &sdev->crtc;
	struct drm_plane *plane = crtc->primary;
	struct drm_connector *connector = &sdev->connector;
	struct drm_framebuffer *fb = NULL;
	struct drm_mode_set set;
	struct drm_modeset_acquire_ctx ctx;
	int ret;

	mutex_lock(&dev->mode_config.mutex);
	connector->funcs->fill_modes(connector, 4096, 4096);
	mutex_unlock(&dev->mode_config.mutex);

	DRM_MODESET_LOCK_ALL_BEGIN(dev, ctx,
				   DRM_MODESET_ACQUIRE_INTERRUPTIBLE, ret);

	if (plane->state)
		fb = plane->state->fb;
	else
		fb = plane->fb;

	if (!fb) {
		drm_dbg(dev, "CRTC doesn't have current FB\n");
		ret = -EINVAL;
		goto out;
	}

	drm_framebuffer_get(fb);

	set.crtc = crtc;
	set.x = 0;
	set.y = 0;
	set.mode = &sdev->mode;
	set.connectors = &connector;
	set.num_connectors = 1;
	set.fb = fb;

	ret = crtc->funcs->set_config(&set, &ctx);

out:
	DRM_MODESET_LOCK_ALL_END(dev, ctx, ret);

	return ret;
}

static void ls2kbmc_events_fn(struct work_struct *work)
{
	struct ls2kbmc_pdata *priv = container_of(work, struct ls2kbmc_pdata, bmc_work);

	/*
	 * The pcie is lost when the BMC resets,
	 * at which point access to the pcie from other CPUs
	 * is suspended to prevent a crash.
	 */
	stop_machine(ls2kbmc_recove_pci_data, priv, NULL);

	drm_info(priv->ddev, "redraw console\n");

	/* We need to re-push the display due to previous pcie loss. */
	ls2kbmc_push_drm_mode(priv->ddev);
}

static irqreturn_t ls2kbmc_interrupt(int irq, void *arg)
{
	struct ls2kbmc_pdata *priv = arg;

	if (system_state != SYSTEM_RUNNING)
		return IRQ_HANDLED;

	/* skip interrupt in BMC_RESET_DELAY */
	if (time_after(jiffies, priv->reset_time + BMC_RESET_DELAY))
		schedule_work(&priv->bmc_work);

	priv->reset_time = jiffies;

	return IRQ_HANDLED;
}

#define BMC_RESET_GPIO			14
#define LOONGSON_GPIO_REG_BASE		0x1fe00500
#define LOONGSON_GPIO_REG_SIZE		0x18
#define LOONGSON_GPIO_OEN		0x0
#define LOONGSON_GPIO_FUNC		0x4
#define LOONGSON_GPIO_INTPOL		0x10
#define LOONGSON_GPIO_INTEN		0x14

/* The gpio interrupt is a watchdog interrupt that is triggered when the BMC resets. */
static int ls2kbmc_gpio_reset_handler(struct ls2kbmc_pdata *priv)
{
	int irq, ret = 0;
	int gsi = 16 + (BMC_RESET_GPIO & 7);
	void __iomem *gpio_base;

	/* Since Loongson-3A hardware does not support GPIO interrupt cascade,
	 * chip->gpio_to_irq() cannot be implemented,
	 * here acpi_register_gsi() is used to get gpio irq.
	 */
	irq = acpi_register_gsi(NULL, gsi, ACPI_EDGE_SENSITIVE, ACPI_ACTIVE_LOW);
	if (irq < 0)
		return irq;

	gpio_base = ioremap(LOONGSON_GPIO_REG_BASE, LOONGSON_GPIO_REG_SIZE);
	if (!gpio_base) {
		acpi_unregister_gsi(gsi);
		return PTR_ERR(gpio_base);
	}

	writel(readl(gpio_base + LOONGSON_GPIO_OEN) | BIT(BMC_RESET_GPIO),
	       gpio_base + LOONGSON_GPIO_OEN);
	writel(readl(gpio_base + LOONGSON_GPIO_FUNC) & ~BIT(BMC_RESET_GPIO),
	       gpio_base + LOONGSON_GPIO_FUNC);
	writel(readl(gpio_base + LOONGSON_GPIO_INTPOL) & ~BIT(BMC_RESET_GPIO),
	       gpio_base + LOONGSON_GPIO_INTPOL);
	writel(readl(gpio_base + LOONGSON_GPIO_INTEN) | BIT(BMC_RESET_GPIO),
	       gpio_base + LOONGSON_GPIO_INTEN);

	ret = request_irq(irq, ls2kbmc_interrupt, IRQF_SHARED | IRQF_TRIGGER_FALLING,
			  "ls2kbmc gpio", priv);

	acpi_unregister_gsi(gsi);
	iounmap(gpio_base);

	return ret;
}

static int ls2kbmc_pdata_initial(struct platform_device *pdev, struct ls2kbmc_pdata *priv)
{
	int ret;

	priv->pdev = *(struct pci_dev **)dev_get_platdata(&pdev->dev);

	ls2kbmc_save_pci_data(priv);

	INIT_WORK(&priv->bmc_work, ls2kbmc_events_fn);

	ret = request_irq(priv->pdev->irq, ls2kbmc_interrupt,
			  IRQF_SHARED | IRQF_TRIGGER_RISING, "ls2kbmc pcie", priv);
	if (ret) {
		pr_err("request_irq(%d) failed\n", priv->pdev->irq);
		return ret;
	}

	return ls2kbmc_gpio_reset_handler(priv);
}

/*
 * Platform driver
 */

static int ls2kbmc_probe(struct platform_device *pdev)
{
	struct ls2kbmc_device *sdev;
	struct ls2kbmc_pdata *priv;
	struct drm_device *dev;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (IS_ERR(priv))
		return -ENOMEM;

	ret = ls2kbmc_pdata_initial(pdev, priv);
	if (ret)
		return ret;

	sdev = ls2kbmc_device_create(&ls2kbmc_driver, pdev, priv);
	if (IS_ERR(sdev))
		return PTR_ERR(sdev);
	dev = &sdev->dev;
	priv->ddev = &sdev->dev;

	ret = drm_dev_register(dev, 0);
	if (ret)
		return ret;

	drm_client_setup(dev, sdev->format);

	return 0;
}

static void ls2kbmc_remove(struct platform_device *pdev)
{
	struct ls2kbmc_device *sdev = platform_get_drvdata(pdev);
	struct drm_device *dev = &sdev->dev;

	drm_dev_unplug(dev);
}

static struct platform_device_id ls2kbmc_id_table[] = {
	{ "ls2kbmc-framebuffer", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, ls2kbmc_id_table);

static struct platform_driver ls2kbmc_platform_driver = {
	.driver = {
		.name = "ls2kbmc-framebuffer",
	},
	.id_table = ls2kbmc_id_table,
	.probe = ls2kbmc_probe,
	.remove = ls2kbmc_remove,
};

module_platform_driver(ls2kbmc_platform_driver);

MODULE_DESCRIPTION("DRM driver for Loongson-2K BMC");
MODULE_LICENSE("GPL");
