// SPDX-License-Identifier: GPL-2.0 or MIT
/* Copyright 2018 Marty E. Plummer <hanetzer@startmail.com> */
/* Copyright 2019 Linaro, Ltd, Rob Herring <robh@kernel.org> */
/* Copyright 2023 Collabora ltd. */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iommu.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/property.h>
#include <linux/pm_runtime.h>
#include <linux/arm-smccc.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#include <drm/drm_drv.h>
#include <drm/drm_managed.h>
#include <drm/drm_print.h>

#include "panthor_devfreq.h"
#include "panthor_device.h"
#include "panthor_fw.h"
#include "panthor_gem.h"
#include "panthor_gpu.h"
#include "panthor_hw.h"
#include "panthor_mmu.h"
#include "panthor_pwr.h"
#include "panthor_regs.h"
#include "panthor_sched.h"

static bool panthor_is_sky1(struct panthor_device *ptdev)
{
	struct device *dev = ptdev->base.dev;

	if (of_device_is_compatible(dev->of_node, "arm,mali-valhall"))
		return true;

	return acpi_dev_hid_uid_match(ACPI_COMPANION(dev), "CIXH5000", NULL);
}

/*
 * Sky1 GPU power-on via raw SMC SCMI call to TFA (Trusted Firmware-A).
 *
 * Under DT, smc_devpd (arm,scmi-smc) sends SCMI POWER_STATE_SET to TFA
 * via SMC function 0xc2000001 with shared memory at 0x84380000.
 * Under ACPI, there's no smc_devpd, so we make the raw SMC call directly.
 *
 * TFA not only powers the GPU domain but also configures the IDM
 * (Interconnect Domain Manager) to allow non-secure (Linux) access
 * to GPU registers.  Without this, any GPU register read causes an
 * unrecoverable SError.
 */
#define SKY1_SMC_SCMI_FUNC_ID		0xc2000001
#define SKY1_SMC_SCMI_SHMEM_PHYS	0x84380000UL
#define SKY1_SMC_SCMI_SHMEM_SIZE	0x80
#define SKY1_PD_GPU			21

/* SCMI shared memory offsets */
#define SCMI_SHMEM_CHAN_STATUS		0x04
#define SCMI_SHMEM_FLAGS		0x10
#define SCMI_SHMEM_LENGTH		0x14
#define SCMI_SHMEM_MSG_HEADER		0x18
#define SCMI_SHMEM_MSG_PAYLOAD		0x1c

static int sky1_smc_scmi_power_set(struct device *dev, u32 domain, u32 state)
{
	void __iomem *shmem;
	struct arm_smccc_res res;
	u32 msg_header, resp_status;
	int timeout = 1000;

	shmem = ioremap(SKY1_SMC_SCMI_SHMEM_PHYS, SKY1_SMC_SCMI_SHMEM_SIZE);
	if (!shmem)
		return -ENOMEM;

	/* Wait for channel free */
	while (!(ioread32(shmem + SCMI_SHMEM_CHAN_STATUS) & BIT(0))) {
		if (--timeout <= 0) {
			dev_err(dev, "SCMI SMC channel busy timeout\n");
			iounmap(shmem);
			return -ETIMEDOUT;
		}
		udelay(10);
	}

	/* Clear channel status */
	iowrite32(0, shmem + SCMI_SHMEM_CHAN_STATUS);

	/* Polling mode (no interrupt) */
	iowrite32(0, shmem + SCMI_SHMEM_FLAGS);

	/*
	 * Message header:
	 *   Bits  0-7:  MSG_ID = 0x04 (POWER_STATE_SET)
	 *   Bits  8-9:  MSG_TYPE = 0 (command)
	 *   Bits 10-17: PROTOCOL_ID = 0x11 (POWER)
	 *   Bits 18-27: TOKEN = 0
	 */
	msg_header = 0x04 | (0x11 << 10);
	iowrite32(msg_header, shmem + SCMI_SHMEM_MSG_HEADER);

	/* Payload: flags(4) + domain(4) + state(4) = 12 bytes */
	iowrite32(0, shmem + SCMI_SHMEM_MSG_PAYLOAD);		/* flags: sync */
	iowrite32(domain, shmem + SCMI_SHMEM_MSG_PAYLOAD + 4);	/* domain_id */
	iowrite32(state, shmem + SCMI_SHMEM_MSG_PAYLOAD + 8);	/* power_state */

	/* Length = msg_header(4) + payload(12) = 16 */
	iowrite32(16, shmem + SCMI_SHMEM_LENGTH);

	/* SMC call: param_page = phys >> 12, param_offset = phys & 0xFFF */
	arm_smccc_smc(SKY1_SMC_SCMI_FUNC_ID,
		      SKY1_SMC_SCMI_SHMEM_PHYS >> 12,
		      SKY1_SMC_SCMI_SHMEM_PHYS & 0xFFF,
		      0, 0, 0, 0, 0, &res);

	if (res.a0) {
		dev_err(dev, "SCMI SMC returned error: 0x%lx\n", res.a0);
		iounmap(shmem);
		return -EIO;
	}

	/* Response: first word of payload is SCMI status */
	resp_status = ioread32(shmem + SCMI_SHMEM_MSG_PAYLOAD);
	iounmap(shmem);

	if (resp_status != 0) {
		dev_err(dev, "SCMI POWER_STATE_SET domain %u failed: %d\n",
			domain, resp_status);
		return -EIO;
	}

	dev_info(dev, "GPU power domain %u powered on via SMC SCMI\n", domain);
	return 0;
}

static bool panthor_force_noncoherent;
module_param_named(force_noncoherent, panthor_force_noncoherent, bool, 0444);
MODULE_PARM_DESC(force_noncoherent,
	"Force GPU to non-coherent mode even when firmware reports coherent");

static int panthor_gpu_coherency_init(struct panthor_device *ptdev)
{
	bool dma_coherent = device_get_dma_attr(ptdev->base.dev) == DEV_DMA_COHERENT;

	/*
	 * Sky1 SoC: The GPU hardware supports full ACE (128/256-bit AMBA 4
	 * ACE master), but the platform uses ACE-Lite (DT system-coherency=0).
	 * The DPU is non-snooping and reads from SLC/DRAM, so ACE-Lite is the
	 * correct choice: GPU L2 evictions route through HN-F and are absorbed
	 * into the SLC, making WB data visible to the DPU without NC memattr.
	 */
	if (panthor_is_sky1(ptdev)) {
		ptdev->coherency_mode = PANTHOR_COHERENCY_ACE_LITE;
		drm_info(&ptdev->base, "Using ACE-Lite bus coherency (Sky1)\n");
	} else if (panthor_force_noncoherent || !dma_coherent) {
		ptdev->coherency_mode = PANTHOR_COHERENCY_NONE;
	} else {
		ptdev->coherency_mode = PANTHOR_COHERENCY_ACE_LITE;
	}

	if (ptdev->coherency_mode == PANTHOR_COHERENCY_NONE)
		return 0;

	/* Verify hardware supports the selected protocol. */
	if (gpu_read(ptdev, GPU_COHERENCY_FEATURES) &
	    GPU_COHERENCY_PROT_BIT(ACE_LITE))
		return 0;

	drm_err(&ptdev->base, "ACE-Lite not supported by hardware");
	return -ENOTSUPP;
}

static int panthor_clk_init(struct panthor_device *ptdev)
{
	/* Try vendor name first (gpu_clk_core), then unnamed default */
	ptdev->clks.core = devm_clk_get_optional(ptdev->base.dev, "gpu_clk_core");
	if (IS_ERR(ptdev->clks.core))
		return dev_err_probe(ptdev->base.dev,
				     PTR_ERR(ptdev->clks.core),
				     "get 'gpu_clk_core' clock failed");
	if (!ptdev->clks.core) {
		ptdev->clks.core = devm_clk_get(ptdev->base.dev, NULL);
		if (IS_ERR(ptdev->clks.core))
			return dev_err_probe(ptdev->base.dev,
					     PTR_ERR(ptdev->clks.core),
					     "get 'core' clock failed");
	}

	/* Try vendor name first (gpu_clk_stacks), then mainline name (stacks) */
	ptdev->clks.stacks = devm_clk_get_optional(ptdev->base.dev, "gpu_clk_stacks");
	if (IS_ERR(ptdev->clks.stacks))
		return dev_err_probe(ptdev->base.dev,
				     PTR_ERR(ptdev->clks.stacks),
				     "get 'gpu_clk_stacks' clock failed");
	if (!ptdev->clks.stacks) {
		ptdev->clks.stacks = devm_clk_get_optional(ptdev->base.dev, "stacks");
		if (IS_ERR(ptdev->clks.stacks))
			return dev_err_probe(ptdev->base.dev,
					     PTR_ERR(ptdev->clks.stacks),
					     "get 'stacks' clock failed");
	}

	ptdev->clks.coregroup = devm_clk_get_optional(ptdev->base.dev, "coregroup");
	if (IS_ERR(ptdev->clks.coregroup))
		return dev_err_probe(ptdev->base.dev,
				     PTR_ERR(ptdev->clks.coregroup),
				     "get 'coregroup' clock failed");

	/* CIX Sky1 needs additional backup clocks */
	if (panthor_is_sky1(ptdev)) {
		ptdev->clks.backup[0] = devm_clk_get_optional(ptdev->base.dev, "gpu_clk_200M");
		if (IS_ERR(ptdev->clks.backup[0]))
			return dev_err_probe(ptdev->base.dev,
					     PTR_ERR(ptdev->clks.backup[0]),
					     "get 'gpu_clk_200M' clock failed");

		ptdev->clks.backup[1] = devm_clk_get_optional(ptdev->base.dev, "gpu_clk_400M");
		if (IS_ERR(ptdev->clks.backup[1]))
			return dev_err_probe(ptdev->base.dev,
					     PTR_ERR(ptdev->clks.backup[1]),
					     "get 'gpu_clk_400M' clock failed");
	}

	drm_dbg(&ptdev->base, "clock rate = %lu\n", clk_get_rate(ptdev->clks.core));
	return 0;
}

static void panthor_pm_domain_fini(struct panthor_device *ptdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ptdev->pm_domain_devs); i++) {
		if (!ptdev->pm_domain_devs[i])
			break;

		if (ptdev->pm_domain_links[i])
			device_link_del(ptdev->pm_domain_links[i]);

		dev_pm_domain_detach(ptdev->pm_domain_devs[i], true);
	}
}

static int panthor_pm_domain_init(struct panthor_device *ptdev)
{
	int err;
	int i, num_domains;

	/*
	 * Under ACPI, power on the GPU domain via raw SMC SCMI to TFA.
	 * This replicates what smc_devpd does under DT: sends
	 * POWER_STATE_SET for SKY1_PD_GPU, which powers the domain
	 * and configures IDM non-secure access permissions.
	 */
	if (!ptdev->base.dev->of_node) {
		if (panthor_is_sky1(ptdev)) {
			err = sky1_smc_scmi_power_set(ptdev->base.dev,
						      SKY1_PD_GPU, 0);
			if (err)
				return err;
		}
		return 0;
	}

	num_domains = of_count_phandle_with_args(ptdev->base.dev->of_node,
						 "power-domains",
						 "#power-domain-cells");

	/*
	 * Single domain is handled by the core, and, if only a single power
	 * the power domain is requested, the property is optional.
	 */
	if (num_domains < 2)
		return 0;

	if (WARN(num_domains > ARRAY_SIZE(ptdev->pm_domain_devs),
			"Too many supplies in compatible structure.\n"))
		return -EINVAL;

	for (i = 0; i < num_domains; i++) {
		ptdev->pm_domain_devs[i] =
			dev_pm_domain_attach_by_id(ptdev->base.dev, i);
		if (IS_ERR_OR_NULL(ptdev->pm_domain_devs[i])) {
			err = PTR_ERR(ptdev->pm_domain_devs[i]) ? : -ENODATA;
			ptdev->pm_domain_devs[i] = NULL;
			dev_err(ptdev->base.dev,
				"failed to get pm-domain %d: %d\n",
				i, err);
			goto err;
		}
		dev_dbg(ptdev->base.dev, "attached pm-domain %d: %s\n",
			i, dev_name(ptdev->pm_domain_devs[i]));

		ptdev->pm_domain_links[i] = device_link_add(ptdev->base.dev,
				ptdev->pm_domain_devs[i], DL_FLAG_PM_RUNTIME |
				DL_FLAG_STATELESS | DL_FLAG_RPM_ACTIVE);
		if (!ptdev->pm_domain_links[i]) {
			dev_err(ptdev->pm_domain_devs[i],
				"adding device link failed!\n");
			err = -ENODEV;
			goto err;
		}
		dev_dbg(ptdev->base.dev, "pm-domain device link %d created\n", i);
	}

	dev_dbg(ptdev->base.dev, "pm_domain_init completed\n");
	return 0;

err:
	panthor_pm_domain_fini(ptdev);
	return err;
}

static int panthor_resets_init(struct panthor_device *ptdev)
{
	/*
	 * Under ACPI, the reset framework has no DT bindings to look up.
	 * Use optional variant so NULL is returned instead of -ENOENT.
	 * reset_control_assert/deassert(NULL) are no-ops.
	 */
	if (!ptdev->base.dev->of_node)
		ptdev->gpu_reset = devm_reset_control_get_optional_exclusive(
					ptdev->base.dev, "gpu_reset");
	else
		ptdev->gpu_reset = devm_reset_control_get_optional(ptdev->base.dev,
								   "gpu_reset");
	if (IS_ERR(ptdev->gpu_reset))
		return dev_err_probe(ptdev->base.dev, PTR_ERR(ptdev->gpu_reset),
				     "failed to get gpu_reset\n");

	return 0;
}

void panthor_device_unplug(struct panthor_device *ptdev)
{
	/* This function can be called from two different path: the reset work
	 * and the platform device remove callback. drm_dev_unplug() doesn't
	 * deal with concurrent callers, so we have to protect drm_dev_unplug()
	 * calls with our own lock, and bail out if the device is already
	 * unplugged.
	 */
	mutex_lock(&ptdev->unplug.lock);
	if (drm_dev_is_unplugged(&ptdev->base)) {
		/* Someone beat us, release the lock and wait for the unplug
		 * operation to be reported as done.
		 **/
		mutex_unlock(&ptdev->unplug.lock);
		wait_for_completion(&ptdev->unplug.done);
		return;
	}

	drm_WARN_ON(&ptdev->base, pm_runtime_get_sync(ptdev->base.dev) < 0);

	/* Call drm_dev_unplug() so any access to HW blocks happening after
	 * that point get rejected.
	 */
	drm_dev_unplug(&ptdev->base);

	/* We do the rest of the unplug with the unplug lock released,
	 * future callers will wait on ptdev->unplug.done anyway.
	 */
	mutex_unlock(&ptdev->unplug.lock);

	/* Now, try to cleanly shutdown the GPU before the device resources
	 * get reclaimed.
	 */
	panthor_sched_unplug(ptdev);
	panthor_fw_unplug(ptdev);
	panthor_mmu_unplug(ptdev);
	panthor_gpu_unplug(ptdev);
	panthor_pwr_unplug(ptdev);

	pm_runtime_dont_use_autosuspend(ptdev->base.dev);
	pm_runtime_put_sync_suspend(ptdev->base.dev);

	/* If PM is disabled, we need to call the suspend handler manually. */
	if (!IS_ENABLED(CONFIG_PM))
		panthor_device_suspend(ptdev->base.dev);

	/* Report the unplug operation as done to unblock concurrent
	 * panthor_device_unplug() callers.
	 */
	complete_all(&ptdev->unplug.done);
}

static void panthor_device_reset_cleanup(struct drm_device *ddev, void *data)
{
	struct panthor_device *ptdev = container_of(ddev, struct panthor_device, base);

	disable_work_sync(&ptdev->reset.work);
	destroy_workqueue(ptdev->reset.wq);
}

static void panthor_device_reset_work(struct work_struct *work)
{
	struct panthor_device *ptdev = container_of(work, struct panthor_device, reset.work);
	int ret = 0, cookie;

	/* If the device is entering suspend, we don't reset. A slow reset will
	 * be forced at resume time instead.
	 */
	if (atomic_read(&ptdev->pm.state) != PANTHOR_DEVICE_PM_STATE_ACTIVE)
		return;

	if (!drm_dev_enter(&ptdev->base, &cookie))
		return;

	panthor_sched_pre_reset(ptdev);
	panthor_fw_pre_reset(ptdev, true);
	panthor_mmu_pre_reset(ptdev);
	panthor_hw_soft_reset(ptdev);
	panthor_hw_l2_power_on(ptdev);
	panthor_mmu_post_reset(ptdev);
	ret = panthor_fw_post_reset(ptdev);
	atomic_set(&ptdev->reset.pending, 0);
	panthor_sched_post_reset(ptdev, ret != 0);
	drm_dev_exit(cookie);

	if (ret) {
		panthor_device_unplug(ptdev);
		drm_err(&ptdev->base, "Failed to boot MCU after reset, making device unusable.");
	}
}

static bool panthor_device_is_initialized(struct panthor_device *ptdev)
{
	return !!ptdev->scheduler;
}

static void panthor_device_free_page(struct drm_device *ddev, void *data)
{
	__free_page(data);
}

int panthor_device_init(struct panthor_device *ptdev)
{
	u32 *dummy_page_virt;
	struct resource *res;
	struct page *p;
	int ret;

	ptdev->soc_data = of_device_get_match_data(ptdev->base.dev);
	dev_dbg(ptdev->base.dev, "probe starting\n");

	init_completion(&ptdev->unplug.done);
	ret = drmm_mutex_init(&ptdev->base, &ptdev->unplug.lock);
	if (ret)
		return ret;

	ret = drmm_mutex_init(&ptdev->base, &ptdev->pm.mmio_lock);
	if (ret)
		return ret;

#ifdef CONFIG_DEBUG_FS
	drmm_mutex_init(&ptdev->base, &ptdev->gems.lock);
	INIT_LIST_HEAD(&ptdev->gems.node);
#endif

	atomic_set(&ptdev->pm.state, PANTHOR_DEVICE_PM_STATE_SUSPENDED);
	p = alloc_page(GFP_KERNEL | __GFP_ZERO);
	if (!p)
		return -ENOMEM;

	ptdev->pm.dummy_latest_flush = p;
	dummy_page_virt = page_address(p);
	ret = drmm_add_action_or_reset(&ptdev->base, panthor_device_free_page,
				       ptdev->pm.dummy_latest_flush);
	if (ret)
		return ret;

	/*
	 * Set the dummy page holding the latest flush to 1. This will cause the
	 * flush to avoided as we know it isn't necessary if the submission
	 * happens while the dummy page is mapped. Zero cannot be used because
	 * that means 'always flush'.
	 */
	*dummy_page_virt = 1;

	INIT_WORK(&ptdev->reset.work, panthor_device_reset_work);
	ptdev->reset.wq = alloc_ordered_workqueue("panthor-reset-wq", 0);
	if (!ptdev->reset.wq)
		return -ENOMEM;

	ret = drmm_add_action_or_reset(&ptdev->base, panthor_device_reset_cleanup, NULL);
	if (ret)
		return ret;

	ret = panthor_clk_init(ptdev);
	if (ret)
		return ret;

	ret = panthor_pm_domain_init(ptdev);
	if (ret)
		return ret;

	ret = panthor_devfreq_init(ptdev);
	if (ret)
		goto err_release_pm_domains;

	ret = panthor_resets_init(ptdev);
	if (ret)
		goto err_release_pm_domains;

	/* Sky1 uses resource 0 for RCSU, resource 1 for GPU registers.
	 * Other platforms use resource 0 for GPU registers.
	 */
	if (panthor_is_sky1(ptdev)) {
		ptdev->iomem = devm_platform_get_and_ioremap_resource(
				to_platform_device(ptdev->base.dev), 1, &res);
		if (IS_ERR(ptdev->iomem)) {
			ret = PTR_ERR(ptdev->iomem);
			goto err_release_pm_domains;
		}

		ptdev->sky1_rcsu_reg = devm_platform_ioremap_resource(
				to_platform_device(ptdev->base.dev), 0);
		if (IS_ERR(ptdev->sky1_rcsu_reg)) {
			ret = PTR_ERR(ptdev->sky1_rcsu_reg);
			goto err_release_pm_domains;
		}
	} else {
		ptdev->iomem = devm_platform_get_and_ioremap_resource(
				to_platform_device(ptdev->base.dev), 0, &res);
		if (IS_ERR(ptdev->iomem)) {
			ret = PTR_ERR(ptdev->iomem);
			goto err_release_pm_domains;
		}
	}

	ptdev->phys_addr = res->start;

	ret = devm_pm_runtime_enable(ptdev->base.dev);
	if (ret)
		goto err_release_pm_domains;

	ret = pm_runtime_resume_and_get(ptdev->base.dev);
	if (ret)
		goto err_release_pm_domains;

	/* If PM is disabled, we need to call panthor_device_resume() manually. */
	if (!IS_ENABLED(CONFIG_PM)) {
		ret = panthor_device_resume(ptdev->base.dev);
		if (ret)
			goto err_rpm_put;
	}

	/*
	 * Sky1 GPU power-on sequence (from CIX GPU Development Guide):
	 * 1. Power domain on  (done in panthor_pm_domain_init)
	 * 2. Clock enable      (done in panthor_device_resume via pm_runtime)
	 * 3. IP Reset assert
	 * 4. IP Reset de-assert
	 * 5. Qchannel clock gating enable
	 */
	if (panthor_is_sky1(ptdev)) {
		if (ptdev->gpu_reset) {
			ret = reset_control_assert(ptdev->gpu_reset);
			if (ret) {
				dev_err(ptdev->base.dev, "GPU reset assert failed: %d\n", ret);
				goto err_rpm_put;
			}

			usleep_range(10, 20);

			ret = reset_control_deassert(ptdev->gpu_reset);
			if (ret) {
				dev_err(ptdev->base.dev, "GPU reset deassert failed: %d\n", ret);
				goto err_rpm_put;
			}

		}

		if (ptdev->sky1_rcsu_reg) {
			u32 pgctrl;

			pgctrl = readl(ptdev->sky1_rcsu_reg + 0x218);
			pgctrl |= BIT(0);  /* QCHANNEL_CLOCK_GATE_ENABLE */
			writel(pgctrl, ptdev->sky1_rcsu_reg + 0x218);

			dev_dbg(ptdev->base.dev, "qchannel clock gating enabled\n");
		}
	}

	ret = panthor_hw_init(ptdev);
	if (ret)
		goto err_rpm_put;

	ret = panthor_pwr_init(ptdev);
	if (ret)
		goto err_rpm_put;

	ret = panthor_gpu_init(ptdev);
	if (ret)
		goto err_unplug_pwr;

	ret = panthor_gpu_coherency_init(ptdev);
	if (ret)
		goto err_unplug_gpu;

	if (ptdev->coherency_mode == PANTHOR_COHERENCY_NONE &&
	    !device_iommu_mapped(ptdev->base.dev))
		drm_info(&ptdev->base,
			 "GPU not behind IOMMU; using non-cacheable memory attributes\n");

	ret = panthor_mmu_init(ptdev);
	if (ret)
		goto err_unplug_gpu;

	ret = panthor_fw_init(ptdev);
	if (ret)
		goto err_unplug_mmu;

	ret = panthor_sched_init(ptdev);
	if (ret)
		goto err_unplug_fw;

	panthor_gem_init(ptdev);

	/* ~3 frames */
	pm_runtime_set_autosuspend_delay(ptdev->base.dev, 50);
	pm_runtime_use_autosuspend(ptdev->base.dev);

	ret = drm_dev_register(&ptdev->base, 0);
	if (ret)
		goto err_disable_autosuspend;

	pm_runtime_put_autosuspend(ptdev->base.dev);
	return 0;

err_disable_autosuspend:
	pm_runtime_dont_use_autosuspend(ptdev->base.dev);
	panthor_sched_unplug(ptdev);

err_unplug_fw:
	panthor_fw_unplug(ptdev);

err_unplug_mmu:
	panthor_mmu_unplug(ptdev);

err_unplug_gpu:
	panthor_gpu_unplug(ptdev);

err_unplug_pwr:
	panthor_pwr_unplug(ptdev);

err_rpm_put:
	pm_runtime_put_sync_suspend(ptdev->base.dev);

err_release_pm_domains:
	panthor_pm_domain_fini(ptdev);
	return ret;
}

#define PANTHOR_EXCEPTION(id) \
	[DRM_PANTHOR_EXCEPTION_ ## id] = { \
		.name = #id, \
	}

struct panthor_exception_info {
	const char *name;
};

static const struct panthor_exception_info panthor_exception_infos[] = {
	PANTHOR_EXCEPTION(OK),
	PANTHOR_EXCEPTION(TERMINATED),
	PANTHOR_EXCEPTION(KABOOM),
	PANTHOR_EXCEPTION(EUREKA),
	PANTHOR_EXCEPTION(ACTIVE),
	PANTHOR_EXCEPTION(CS_RES_TERM),
	PANTHOR_EXCEPTION(CS_CONFIG_FAULT),
	PANTHOR_EXCEPTION(CS_UNRECOVERABLE),
	PANTHOR_EXCEPTION(CS_ENDPOINT_FAULT),
	PANTHOR_EXCEPTION(CS_BUS_FAULT),
	PANTHOR_EXCEPTION(CS_INSTR_INVALID),
	PANTHOR_EXCEPTION(CS_CALL_STACK_OVERFLOW),
	PANTHOR_EXCEPTION(CS_INHERIT_FAULT),
	PANTHOR_EXCEPTION(INSTR_INVALID_PC),
	PANTHOR_EXCEPTION(INSTR_INVALID_ENC),
	PANTHOR_EXCEPTION(INSTR_BARRIER_FAULT),
	PANTHOR_EXCEPTION(DATA_INVALID_FAULT),
	PANTHOR_EXCEPTION(TILE_RANGE_FAULT),
	PANTHOR_EXCEPTION(ADDR_RANGE_FAULT),
	PANTHOR_EXCEPTION(IMPRECISE_FAULT),
	PANTHOR_EXCEPTION(OOM),
	PANTHOR_EXCEPTION(CSF_FW_INTERNAL_ERROR),
	PANTHOR_EXCEPTION(CSF_RES_EVICTION_TIMEOUT),
	PANTHOR_EXCEPTION(GPU_BUS_FAULT),
	PANTHOR_EXCEPTION(GPU_SHAREABILITY_FAULT),
	PANTHOR_EXCEPTION(SYS_SHAREABILITY_FAULT),
	PANTHOR_EXCEPTION(GPU_CACHEABILITY_FAULT),
	PANTHOR_EXCEPTION(TRANSLATION_FAULT_0),
	PANTHOR_EXCEPTION(TRANSLATION_FAULT_1),
	PANTHOR_EXCEPTION(TRANSLATION_FAULT_2),
	PANTHOR_EXCEPTION(TRANSLATION_FAULT_3),
	PANTHOR_EXCEPTION(TRANSLATION_FAULT_4),
	PANTHOR_EXCEPTION(PERM_FAULT_0),
	PANTHOR_EXCEPTION(PERM_FAULT_1),
	PANTHOR_EXCEPTION(PERM_FAULT_2),
	PANTHOR_EXCEPTION(PERM_FAULT_3),
	PANTHOR_EXCEPTION(ACCESS_FLAG_1),
	PANTHOR_EXCEPTION(ACCESS_FLAG_2),
	PANTHOR_EXCEPTION(ACCESS_FLAG_3),
	PANTHOR_EXCEPTION(ADDR_SIZE_FAULT_IN),
	PANTHOR_EXCEPTION(ADDR_SIZE_FAULT_OUT0),
	PANTHOR_EXCEPTION(ADDR_SIZE_FAULT_OUT1),
	PANTHOR_EXCEPTION(ADDR_SIZE_FAULT_OUT2),
	PANTHOR_EXCEPTION(ADDR_SIZE_FAULT_OUT3),
	PANTHOR_EXCEPTION(MEM_ATTR_FAULT_0),
	PANTHOR_EXCEPTION(MEM_ATTR_FAULT_1),
	PANTHOR_EXCEPTION(MEM_ATTR_FAULT_2),
	PANTHOR_EXCEPTION(MEM_ATTR_FAULT_3),
};

const char *panthor_exception_name(struct panthor_device *ptdev, u32 exception_code)
{
	if (exception_code >= ARRAY_SIZE(panthor_exception_infos) ||
	    !panthor_exception_infos[exception_code].name)
		return "Unknown exception type";

	return panthor_exception_infos[exception_code].name;
}

static vm_fault_t panthor_mmio_vm_fault(struct vm_fault *vmf)
{
	struct vm_area_struct *vma = vmf->vma;
	struct panthor_device *ptdev = vma->vm_private_data;
	u64 offset = (u64)vma->vm_pgoff << PAGE_SHIFT;
	unsigned long pfn;
	pgprot_t pgprot;
	vm_fault_t ret;
	bool active;
	int cookie;

	if (!drm_dev_enter(&ptdev->base, &cookie))
		return VM_FAULT_SIGBUS;

	mutex_lock(&ptdev->pm.mmio_lock);
	active = atomic_read(&ptdev->pm.state) == PANTHOR_DEVICE_PM_STATE_ACTIVE;

	switch (offset) {
	case DRM_PANTHOR_USER_FLUSH_ID_MMIO_OFFSET:
		if (active)
			pfn = __phys_to_pfn(ptdev->phys_addr + CSF_GPU_LATEST_FLUSH_ID);
		else
			pfn = page_to_pfn(ptdev->pm.dummy_latest_flush);
		break;

	default:
		ret = VM_FAULT_SIGBUS;
		goto out_unlock;
	}

	pgprot = vma->vm_page_prot;
	if (active)
		pgprot = pgprot_noncached(pgprot);

	ret = vmf_insert_pfn_prot(vma, vmf->address, pfn, pgprot);

out_unlock:
	mutex_unlock(&ptdev->pm.mmio_lock);
	drm_dev_exit(cookie);
	return ret;
}

static const struct vm_operations_struct panthor_mmio_vm_ops = {
	.fault = panthor_mmio_vm_fault,
};

int panthor_device_mmap_io(struct panthor_device *ptdev, struct vm_area_struct *vma)
{
	u64 offset = (u64)vma->vm_pgoff << PAGE_SHIFT;

	if ((vma->vm_flags & VM_SHARED) == 0)
		return -EINVAL;

	switch (offset) {
	case DRM_PANTHOR_USER_FLUSH_ID_MMIO_OFFSET:
		if (vma->vm_end - vma->vm_start != PAGE_SIZE ||
		    (vma->vm_flags & (VM_WRITE | VM_EXEC)))
			return -EINVAL;
		vm_flags_clear(vma, VM_MAYWRITE);

		break;

	default:
		return -EINVAL;
	}

	/* Defer actual mapping to the fault handler. */
	vma->vm_private_data = ptdev;
	vma->vm_ops = &panthor_mmio_vm_ops;
	vm_flags_set(vma,
		     VM_IO | VM_DONTCOPY | VM_DONTEXPAND |
		     VM_NORESERVE | VM_DONTDUMP | VM_PFNMAP);
	return 0;
}

static int panthor_device_resume_hw_components(struct panthor_device *ptdev)
{
	int ret;

	panthor_pwr_resume(ptdev);
	panthor_gpu_resume(ptdev);
	panthor_mmu_resume(ptdev);

	ret = panthor_fw_resume(ptdev);
	if (!ret)
		return 0;

	panthor_mmu_suspend(ptdev);
	panthor_gpu_suspend(ptdev);
	panthor_pwr_suspend(ptdev);
	return ret;
}

int panthor_device_resume(struct device *dev)
{
	struct panthor_device *ptdev = dev_get_drvdata(dev);
	int ret, cookie;

	if (atomic_read(&ptdev->pm.state) != PANTHOR_DEVICE_PM_STATE_SUSPENDED)
		return -EINVAL;

	atomic_set(&ptdev->pm.state, PANTHOR_DEVICE_PM_STATE_RESUMING);

	ret = clk_prepare_enable(ptdev->clks.core);
	if (ret)
		goto err_set_suspended;

	ret = clk_prepare_enable(ptdev->clks.stacks);
	if (ret)
		goto err_disable_core_clk;

	ret = clk_prepare_enable(ptdev->clks.coregroup);
	if (ret)
		goto err_disable_stacks_clk;

	panthor_devfreq_resume(ptdev);

	if (panthor_device_is_initialized(ptdev) &&
	    drm_dev_enter(&ptdev->base, &cookie)) {
		/* If there was a reset pending at the time we suspended the
		 * device, we force a slow reset.
		 */
		if (atomic_read(&ptdev->reset.pending)) {
			ptdev->reset.fast = false;
			atomic_set(&ptdev->reset.pending, 0);
		}

		ret = panthor_device_resume_hw_components(ptdev);
		if (ret && ptdev->reset.fast) {
			drm_err(&ptdev->base, "Fast reset failed, trying a slow reset");
			ptdev->reset.fast = false;
			ret = panthor_device_resume_hw_components(ptdev);
		}

		if (!ret)
			panthor_sched_resume(ptdev);

		drm_dev_exit(cookie);

		if (ret)
			goto err_suspend_devfreq;
	}

	/* Clear all IOMEM mappings pointing to this device after we've
	 * resumed. This way the fake mappings pointing to the dummy pages
	 * are removed and the real iomem mapping will be restored on next
	 * access.
	 */
	mutex_lock(&ptdev->pm.mmio_lock);
	unmap_mapping_range(ptdev->base.anon_inode->i_mapping,
			    DRM_PANTHOR_USER_MMIO_OFFSET, 0, 1);
	atomic_set(&ptdev->pm.state, PANTHOR_DEVICE_PM_STATE_ACTIVE);
	mutex_unlock(&ptdev->pm.mmio_lock);
	return 0;

err_suspend_devfreq:
	panthor_devfreq_suspend(ptdev);
	clk_disable_unprepare(ptdev->clks.coregroup);

err_disable_stacks_clk:
	clk_disable_unprepare(ptdev->clks.stacks);

err_disable_core_clk:
	clk_disable_unprepare(ptdev->clks.core);

err_set_suspended:
	atomic_set(&ptdev->pm.state, PANTHOR_DEVICE_PM_STATE_SUSPENDED);
	atomic_set(&ptdev->pm.recovery_needed, 1);
	return ret;
}

int panthor_device_suspend(struct device *dev)
{
	struct panthor_device *ptdev = dev_get_drvdata(dev);
	int cookie;

	if (atomic_read(&ptdev->pm.state) != PANTHOR_DEVICE_PM_STATE_ACTIVE)
		return -EINVAL;

	/* Clear all IOMEM mappings pointing to this device before we
	 * shutdown the power-domain and clocks. Failing to do that results
	 * in external aborts when the process accesses the iomem region.
	 * We change the state and call unmap_mapping_range() with the
	 * mmio_lock held to make sure the vm_fault handler won't set up
	 * invalid mappings.
	 */
	mutex_lock(&ptdev->pm.mmio_lock);
	atomic_set(&ptdev->pm.state, PANTHOR_DEVICE_PM_STATE_SUSPENDING);
	unmap_mapping_range(ptdev->base.anon_inode->i_mapping,
			    DRM_PANTHOR_USER_MMIO_OFFSET, 0, 1);
	mutex_unlock(&ptdev->pm.mmio_lock);

	if (panthor_device_is_initialized(ptdev) &&
	    drm_dev_enter(&ptdev->base, &cookie)) {
		cancel_work_sync(&ptdev->reset.work);

		/* We prepare everything as if we were resetting the GPU.
		 * The end of the reset will happen in the resume path though.
		 */
		panthor_sched_suspend(ptdev);
		panthor_fw_suspend(ptdev);
		panthor_mmu_suspend(ptdev);
		panthor_gpu_suspend(ptdev);
		panthor_pwr_suspend(ptdev);
		drm_dev_exit(cookie);
	}

	panthor_devfreq_suspend(ptdev);

	clk_disable_unprepare(ptdev->clks.coregroup);
	clk_disable_unprepare(ptdev->clks.stacks);
	clk_disable_unprepare(ptdev->clks.core);
	atomic_set(&ptdev->pm.state, PANTHOR_DEVICE_PM_STATE_SUSPENDED);
	return 0;
}
