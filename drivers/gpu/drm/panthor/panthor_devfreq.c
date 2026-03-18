// SPDX-License-Identifier: GPL-2.0 or MIT
/* Copyright 2019 Collabora ltd. */

#include <linux/clk.h>
#include <linux/devfreq.h>
#include <linux/devfreq_cooling.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_opp.h>

#include <drm/drm_managed.h>
#include <drm/drm_print.h>

#include "panthor_devfreq.h"
#include "panthor_device.h"

/*
 * SCMI rate limiting for GPU DVFS.
 *
 * The SCP firmware on Sky1 processes SCMI perf_level_set messages through a
 * shared mailbox channel used by all 12 performance domains. Under sustained
 * devfreq load (governor polling every 50-100ms), the firmware falls behind,
 * responds out of order, and the SCMI transport locks up — blocking all SCMI
 * users including networking.
 *
 * Fast channels are advertised by firmware but return -EOPNOTSUPP for all
 * DESCRIBE_FASTCHANNEL requests, so we must rate-limit at the driver level.
 */
#define PANTHOR_SCMI_RATE_LIMIT_US	100000		/* 100ms min between SCMI calls */
#define PANTHOR_SCMI_MAX_ERRORS		3		/* errors before safe-freq boost */
#define PANTHOR_SCMI_BACKOFF_BASE_US	500000		/* 500ms base backoff */
#define PANTHOR_SCMI_BACKOFF_MAX_US	4000000		/* 4s max backoff */

/**
 * struct panthor_devfreq - Device frequency management
 */
struct panthor_devfreq {
	/** @devfreq: devfreq device. */
	struct devfreq *devfreq;

	/** @gov_data: Governor data. */
	struct devfreq_simple_ondemand_data gov_data;

	/**
	 * @opp_dev: Virtual device for SCMI perf domain OPP control.
	 *
	 * On platforms with a named "perf" power domain (e.g. SCMI DVFS),
	 * frequency changes must go through this virtual device rather than
	 * the main GPU device, so the OPP framework routes through the SCMI
	 * performance state (which manages both frequency and voltage).
	 * NULL if not using SCMI perf domain.
	 */
	struct device *opp_dev;

	/** @busy_time: Busy time. */
	ktime_t busy_time;

	/** @idle_time: Idle time. */
	ktime_t idle_time;

	/** @time_last_update: Last update time. */
	ktime_t time_last_update;

	/** @last_busy_state: True if the GPU was busy last time we updated the state. */
	bool last_busy_state;

	/**
	 * @lock: Lock used to protect busy_time, idle_time, time_last_update,
	 * last_busy_state, and the SCMI rate-limiting fields below.
	 *
	 * These fields can be accessed concurrently by panthor_devfreq_get_dev_status()
	 * and panthor_devfreq_record_{busy,idle}().
	 */
	spinlock_t lock;

	/** @last_set_freq: Last frequency successfully sent to SCMI. */
	unsigned long last_set_freq;

	/** @last_set_time: ktime of the last actual SCMI frequency change. */
	ktime_t last_set_time;

	/** @scmi_err_count: Consecutive SCMI error count for backoff. */
	unsigned int scmi_err_count;

	/** @backoff_until: ktime before which SCMI calls are suppressed. */
	ktime_t backoff_until;

	/** @safe_freq: Mid-range OPP for error recovery (computed at init). */
	unsigned long safe_freq;
};

static void panthor_devfreq_update_utilization(struct panthor_devfreq *pdevfreq)
{
	ktime_t now, last;

	now = ktime_get();
	last = pdevfreq->time_last_update;

	if (pdevfreq->last_busy_state)
		pdevfreq->busy_time += ktime_sub(now, last);
	else
		pdevfreq->idle_time += ktime_sub(now, last);

	pdevfreq->time_last_update = now;
}

/*
 * Set GPU frequency via the SCMI perf domain's performance state.
 * Unlike dev_pm_opp_set_rate(), this does not require a clock — the genpd's
 * set_performance_state callback handles the SCMI call.
 */
static int panthor_devfreq_scmi_set_freq(struct device *dev, unsigned long freq)
{
	struct dev_pm_opp *opp;
	int ret;

	opp = dev_pm_opp_find_freq_ceil(dev, &freq);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	ret = dev_pm_opp_set_opp(dev, opp);
	dev_pm_opp_put(opp);
	return ret;
}

static int panthor_devfreq_target(struct device *dev, unsigned long *freq,
				  u32 flags)
{
	struct panthor_device *ptdev = dev_get_drvdata(dev);
	struct panthor_devfreq *pdevfreq = ptdev->devfreq;
	struct dev_pm_opp *opp;
	unsigned long target_freq;
	unsigned long irqflags;
	ktime_t now;
	int err;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	target_freq = *freq;
	dev_pm_opp_put(opp);

	/* Non-SCMI platforms: pass through unchanged */
	if (!pdevfreq->opp_dev)
		return dev_pm_opp_set_rate(dev, target_freq);

	now = ktime_get();
	spin_lock_irqsave(&pdevfreq->lock, irqflags);

	/* Suppress calls during error backoff */
	if (ktime_before(now, pdevfreq->backoff_until)) {
		spin_unlock_irqrestore(&pdevfreq->lock, irqflags);
		return 0;
	}

	/* Skip if already at the requested frequency */
	if (target_freq == pdevfreq->last_set_freq) {
		spin_unlock_irqrestore(&pdevfreq->lock, irqflags);
		return 0;
	}

	/* Rate limit: enforce minimum interval between SCMI calls */
	if (ktime_us_delta(now, pdevfreq->last_set_time) <
	    PANTHOR_SCMI_RATE_LIMIT_US) {
		spin_unlock_irqrestore(&pdevfreq->lock, irqflags);
		return 0;
	}

	spin_unlock_irqrestore(&pdevfreq->lock, irqflags);

	/* Actual SCMI frequency change (may sleep on mailbox) */
	err = panthor_devfreq_scmi_set_freq(dev, target_freq);

	spin_lock_irqsave(&pdevfreq->lock, irqflags);

	if (!err) {
		pdevfreq->last_set_freq = target_freq;
		pdevfreq->last_set_time = now;
		pdevfreq->scmi_err_count = 0;
		pdevfreq->backoff_until = 0;
	} else {
		unsigned int errcnt = ++pdevfreq->scmi_err_count;
		unsigned long backoff_us;

		/* Exponential backoff: 500ms, 1s, 2s, capped at 4s */
		backoff_us = PANTHOR_SCMI_BACKOFF_BASE_US <<
			     min(errcnt - 1, 3u);
		if (backoff_us > PANTHOR_SCMI_BACKOFF_MAX_US)
			backoff_us = PANTHOR_SCMI_BACKOFF_MAX_US;
		pdevfreq->backoff_until = ktime_add_us(now, backoff_us);

		drm_warn(&ptdev->base,
			 "GPU DVFS: SCMI error %d (count=%u), backoff %lu ms\n",
			 err, errcnt, backoff_us / 1000);

		/* After repeated errors, boost to safe mid-range frequency */
		if (errcnt >= PANTHOR_SCMI_MAX_ERRORS && pdevfreq->safe_freq &&
		    pdevfreq->last_set_freq != pdevfreq->safe_freq) {
			spin_unlock_irqrestore(&pdevfreq->lock, irqflags);

			drm_info(&ptdev->base,
				 "GPU DVFS: boosting to safe freq %lu MHz\n",
				 pdevfreq->safe_freq / 1000000);
			err = panthor_devfreq_scmi_set_freq(dev,
						       pdevfreq->safe_freq);

			spin_lock_irqsave(&pdevfreq->lock, irqflags);
			if (!err) {
				pdevfreq->last_set_freq = pdevfreq->safe_freq;
				pdevfreq->last_set_time = ktime_get();
			}
		}
	}

	spin_unlock_irqrestore(&pdevfreq->lock, irqflags);

	/* Always return 0 — errors cause devfreq to spam retries */
	return 0;
}

static void panthor_devfreq_reset(struct panthor_devfreq *pdevfreq)
{
	pdevfreq->busy_time = 0;
	pdevfreq->idle_time = 0;
	pdevfreq->time_last_update = ktime_get();
}

static int panthor_devfreq_get_dev_status(struct device *dev,
					  struct devfreq_dev_status *status)
{
	struct panthor_device *ptdev = dev_get_drvdata(dev);
	struct panthor_devfreq *pdevfreq = ptdev->devfreq;
	unsigned long irqflags;

	status->current_frequency = clk_get_rate(ptdev->clks.core);

	spin_lock_irqsave(&pdevfreq->lock, irqflags);

	panthor_devfreq_update_utilization(pdevfreq);

	status->total_time = ktime_to_ns(ktime_add(pdevfreq->busy_time,
						   pdevfreq->idle_time));

	status->busy_time = ktime_to_ns(pdevfreq->busy_time);

	panthor_devfreq_reset(pdevfreq);

	spin_unlock_irqrestore(&pdevfreq->lock, irqflags);

	drm_dbg(&ptdev->base, "busy %lu total %lu %lu %% freq %lu MHz\n",
		status->busy_time, status->total_time,
		status->busy_time / (status->total_time / 100),
		status->current_frequency / 1000 / 1000);

	return 0;
}

static int panthor_devfreq_get_cur_freq(struct device *dev, unsigned long *freq)
{
	struct panthor_device *ptdev = dev_get_drvdata(dev);

	*freq = clk_get_rate(ptdev->clks.core);

	return 0;
}

static struct devfreq_dev_profile panthor_devfreq_profile = {
	.timer = DEVFREQ_TIMER_DELAYED,
	.polling_ms = 50, /* ~3 frames */
	.target = panthor_devfreq_target,
	.get_dev_status = panthor_devfreq_get_dev_status,
	.get_cur_freq = panthor_devfreq_get_cur_freq,
};

/**
 * panthor_devfreq_scmi_init() - Try to set up DVFS via SCMI perf domain.
 * @ptdev: Panthor device
 * @pdevfreq: Panthor devfreq state
 *
 * On platforms with multiple power domains (e.g. Sky1 with pd_gpu + perf),
 * the SCMI firmware provides an OPP table via the perf domain's attach_dev
 * callback.
 *
 * Under DT: panthor_pm_domain_init() already created virtual devices for each
 * domain.  Find the "perf" domain's virtual device, read OPPs from it, and
 * copy them to the main device.  Frequency changes route through the virtual
 * device so the SCMI firmware manages both frequency and voltage.
 *
 * Under ACPI: attach the GPU device directly to the SCMI perf genpd (looked
 * up by firmware name).  The attach_dev callback populates OPPs on the main
 * device.  Frequency changes go through the genpd's set_performance_state.
 *
 * Return: number of OPPs added, 0 if no perf domain found, negative on error.
 */
static int panthor_devfreq_scmi_init(struct panthor_device *ptdev,
				     struct panthor_devfreq *pdevfreq)
{
	struct device *dev = ptdev->base.dev;
	struct device *opp_dev;
	struct dev_pm_opp *opp;
	unsigned long freq;
	int count, i, ret;

	if (dev->of_node) {
		int index;

		/* DT: find "perf" power domain virtual device */
		index = of_property_match_string(dev->of_node,
						 "power-domain-names", "perf");
		if (index < 0 || index >= ARRAY_SIZE(ptdev->pm_domain_devs))
			return 0;

		opp_dev = ptdev->pm_domain_devs[index];
		if (!opp_dev)
			return 0;

		/* Copy OPPs from virtual device to main device */
		count = dev_pm_opp_get_opp_count(opp_dev);
		if (count <= 0)
			return 0;

		for (i = 0, freq = 0; ; i++, freq++) {
			opp = dev_pm_opp_find_freq_ceil(opp_dev, &freq);
			if (IS_ERR(opp))
				break;
			dev_pm_opp_put(opp);

			ret = dev_pm_opp_add(dev, freq, 0);
			if (ret) {
				drm_warn(&ptdev->base,
					 "Failed to add OPP %lu Hz: %d\n",
					 freq, ret);
				break;
			}
		}

		if (i == 0)
			return 0;
	} else {
		/*
		 * ACPI: attach directly to the SCMI perf genpd.  The
		 * genpd's attach_dev callback populates OPPs on the
		 * main device.  No virtual device or OPP copy needed.
		 */
		struct generic_pm_domain *perf_genpd;

		perf_genpd = pm_genpd_lookup_by_name("gpu_core");
		if (!perf_genpd) {
			drm_dbg(&ptdev->base,
				"GPU DVFS: gpu_core genpd not found\n");
			return 0;
		}

		ret = pm_genpd_add_device(perf_genpd, dev);
		if (ret) {
			drm_warn(&ptdev->base,
				 "Failed to attach SCMI perf domain: %d\n",
				 ret);
			return 0;
		}

		count = dev_pm_opp_get_opp_count(dev);
		if (count <= 0) {
			pm_genpd_remove_device(dev);
			return 0;
		}

		/* Main device IS the OPP device under ACPI */
		opp_dev = dev;
	}

	pdevfreq->opp_dev = opp_dev;

	drm_dbg(&ptdev->base,
		"GPU DVFS: %d OPPs from SCMI perf domain\n",
		dev_pm_opp_get_opp_count(dev));

	return dev_pm_opp_get_opp_count(dev);
}

int panthor_devfreq_init(struct panthor_device *ptdev)
{
	/* There's actually 2 regulators (mali and sram), but the OPP core only
	 * supports one.
	 *
	 * We assume the sram regulator is coupled with the mali one and let
	 * the coupling logic deal with voltage updates.
	 */
	static const char * const reg_names[] = { "mali", NULL };
	struct thermal_cooling_device *cooling;
	struct device *dev = ptdev->base.dev;
	struct panthor_devfreq *pdevfreq;
	struct opp_table *table;
	struct dev_pm_opp *opp;
	unsigned long cur_freq;
	unsigned long freq = ULONG_MAX;
	int ret;

	pdevfreq = drmm_kzalloc(&ptdev->base, sizeof(*ptdev->devfreq), GFP_KERNEL);
	if (!pdevfreq)
		return -ENOMEM;

	ptdev->devfreq = pdevfreq;

	/*
	 * On platforms with a named "perf" power domain (SCMI DVFS), the
	 * platform bus won't auto-attach it when multiple power domains are
	 * present. Try to attach explicitly and populate OPPs from firmware.
	 */
	ret = panthor_devfreq_scmi_init(ptdev, pdevfreq);
	if (ret < 0)
		return ret;

	/*
	 * The power domain associated with the GPU may have already added an
	 * OPP table, complete with OPPs, as part of the platform bus
	 * initialization. If this is the case, the power domain is in charge of
	 * also controlling the performance, with a set_performance callback.
	 * Only add a new OPP table from DT if there isn't such a table present
	 * already.
	 */
	table = dev_pm_opp_get_opp_table(dev);
	if (IS_ERR_OR_NULL(table)) {
		ret = devm_pm_opp_set_regulators(dev, reg_names);
		if (ret) {
			/* Continue if the optional regulator is missing */
			if (ret != -ENODEV) {
				if (ret != -EPROBE_DEFER)
					DRM_DEV_ERROR(dev, "Couldn't set OPP regulators\n");
				return ret;
			}
		}

		ret = devm_pm_opp_of_add_table(dev);
		if (ret) {
			/* Optional, continue without devfreq */
			if (ret == -ENODEV)
				ret = 0;
			return ret;
		}
	} else {
		dev_pm_opp_put_opp_table(table);
	}

	spin_lock_init(&pdevfreq->lock);

	panthor_devfreq_reset(pdevfreq);

	cur_freq = clk_get_rate(ptdev->clks.core);

	/* Regulator coupling only takes care of synchronizing/balancing voltage
	 * updates, but the coupled regulator needs to be enabled manually.
	 *
	 * We use devm_regulator_get_enable_optional() and keep the sram supply
	 * enabled until the device is removed, just like we do for the mali
	 * supply, which is enabled when dev_pm_opp_set_opp(dev, opp) is called,
	 * and disabled when the opp_table is torn down, using the devm action.
	 *
	 * If we really care about disabling regulators on suspend, we should:
	 * - use devm_regulator_get_optional() here
	 * - call dev_pm_opp_set_opp(dev, NULL) before leaving this function
	 *   (this disables the regulator passed to the OPP layer)
	 * - call dev_pm_opp_set_opp(dev, NULL) and
	 *   regulator_disable(ptdev->regulators.sram) in
	 *   panthor_devfreq_suspend()
	 * - call dev_pm_opp_set_opp(dev, default_opp) and
	 *   regulator_enable(ptdev->regulators.sram) in
	 *   panthor_devfreq_resume()
	 *
	 * But without knowing if it's beneficial or not (in term of power
	 * consumption), or how much it slows down the suspend/resume steps,
	 * let's just keep regulators enabled for the device lifetime.
	 */
	ret = devm_regulator_get_enable_optional(dev, "sram");
	if (ret && ret != -ENODEV) {
		if (ret != -EPROBE_DEFER)
			DRM_DEV_ERROR(dev, "Couldn't retrieve/enable sram supply\n");
		return ret;
	}

	opp = devfreq_recommended_opp(dev, &cur_freq, 0);
	if (IS_ERR(opp))
		return PTR_ERR(opp);

	panthor_devfreq_profile.initial_freq = cur_freq;

	/*
	 * Set the recommended OPP. This will enable and configure the regulator
	 * if any and will avoid a switch off by regulator_late_cleanup().
	 *
	 * Use dev_pm_opp_set_opp() (not dev_pm_opp_set_rate) so this works
	 * both with regulators and with SCMI perf domains (which have no
	 * clock — frequency changes go through the genpd performance state).
	 */
	ret = dev_pm_opp_set_opp(dev, opp);
	dev_pm_opp_put(opp);
	if (ret) {
		DRM_DEV_ERROR(dev, "Couldn't set recommended OPP\n");
		return ret;
	}

	/* Find the fastest defined rate  */
	opp = dev_pm_opp_find_freq_floor(dev, &freq);
	if (IS_ERR(opp))
		return PTR_ERR(opp);
	ptdev->fast_rate = freq;

	dev_pm_opp_put(opp);

	/*
	 * For SCMI platforms, compute a safe mid-range frequency for error
	 * recovery and increase the polling interval to match our rate limit.
	 */
	if (pdevfreq->opp_dev) {
		unsigned long min_f = 0, max_f = ULONG_MAX, mid, safe;
		struct dev_pm_opp *opp_tmp;

		opp_tmp = dev_pm_opp_find_freq_ceil(dev, &min_f);
		if (!IS_ERR(opp_tmp)) {
			dev_pm_opp_put(opp_tmp);
			opp_tmp = dev_pm_opp_find_freq_floor(dev, &max_f);
			if (!IS_ERR(opp_tmp)) {
				dev_pm_opp_put(opp_tmp);
				mid = min_f + (max_f - min_f) / 2;
				safe = mid;
				opp_tmp = dev_pm_opp_find_freq_ceil(dev, &safe);
				if (!IS_ERR(opp_tmp)) {
					pdevfreq->safe_freq = safe;
					dev_pm_opp_put(opp_tmp);
				}
			}
		}

		panthor_devfreq_profile.polling_ms = 100;
	}

	/*
	 * Setup default thresholds for the simple_ondemand governor.
	 * The values are chosen based on experiments.
	 */
	pdevfreq->gov_data.upthreshold = 45;
	pdevfreq->gov_data.downdifferential = 5;

	pdevfreq->devfreq = devm_devfreq_add_device(dev, &panthor_devfreq_profile,
						    DEVFREQ_GOV_SIMPLE_ONDEMAND,
						    &pdevfreq->gov_data);
	if (IS_ERR(pdevfreq->devfreq)) {
		DRM_DEV_ERROR(dev, "Couldn't initialize GPU devfreq\n");
		ret = PTR_ERR(pdevfreq->devfreq);
		pdevfreq->devfreq = NULL;
		return ret;
	}

	cooling = devfreq_cooling_em_register(pdevfreq->devfreq, NULL);
	if (IS_ERR(cooling))
		DRM_DEV_INFO(dev, "Failed to register cooling device\n");

	return 0;
}

void panthor_devfreq_resume(struct panthor_device *ptdev)
{
	struct panthor_devfreq *pdevfreq = ptdev->devfreq;

	if (!pdevfreq->devfreq)
		return;

	panthor_devfreq_reset(pdevfreq);

	/* Clear SCMI rate-limiting state on resume */
	pdevfreq->last_set_freq = 0;
	pdevfreq->last_set_time = 0;
	pdevfreq->scmi_err_count = 0;
	pdevfreq->backoff_until = 0;

	drm_WARN_ON(&ptdev->base, devfreq_resume_device(pdevfreq->devfreq));
}

void panthor_devfreq_suspend(struct panthor_device *ptdev)
{
	struct panthor_devfreq *pdevfreq = ptdev->devfreq;

	if (!pdevfreq->devfreq)
		return;

	drm_WARN_ON(&ptdev->base, devfreq_suspend_device(pdevfreq->devfreq));
}

void panthor_devfreq_record_busy(struct panthor_device *ptdev)
{
	struct panthor_devfreq *pdevfreq = ptdev->devfreq;
	unsigned long irqflags;

	if (!pdevfreq->devfreq)
		return;

	spin_lock_irqsave(&pdevfreq->lock, irqflags);

	panthor_devfreq_update_utilization(pdevfreq);
	pdevfreq->last_busy_state = true;

	spin_unlock_irqrestore(&pdevfreq->lock, irqflags);
}

void panthor_devfreq_record_idle(struct panthor_device *ptdev)
{
	struct panthor_devfreq *pdevfreq = ptdev->devfreq;
	unsigned long irqflags;

	if (!pdevfreq->devfreq)
		return;

	spin_lock_irqsave(&pdevfreq->lock, irqflags);

	panthor_devfreq_update_utilization(pdevfreq);
	pdevfreq->last_busy_state = false;

	spin_unlock_irqrestore(&pdevfreq->lock, irqflags);
}

unsigned long panthor_devfreq_get_freq(struct panthor_device *ptdev)
{
	struct panthor_devfreq *pdevfreq = ptdev->devfreq;
	unsigned long freq = 0;
	int ret;

	if (!pdevfreq->devfreq)
		return 0;

	ret = pdevfreq->devfreq->profile->get_cur_freq(ptdev->base.dev, &freq);
	if (ret)
		return 0;

	return freq;
}
