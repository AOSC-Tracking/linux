// SPDX-License-Identifier: GPL-2.0+
/*
 * CPU power state driver for Apple M1 SoC
 *
 * Copyright (C) 2021 Corellium LLC
 */

#include <linux/cpufreq.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#define NUM_BASES			4
#define NUM_CLUSTERS			2
#define PMGR_BASE			2
#define MCC_BASE			3

#define CLUSTER_ECPU			0
#define CLUSTER_PCPU			1

#define DFLT_STATE			2
#define SETVOLT_MIN_STATE		6

#define DRAMCFG_SLOW			0
#define DRAMCFG_FAST			1
#define FAST_FREQ_INVALID		(-1u)

#define CLUSTER_VCTRL			0x20000
#define   CLUSTER_VCTRL_SETVOLT		BIT(29)
#define CLUSTER_PSTATE			0x20020
#define   CLUSTER_PSTATE_LOCKCTRL	BIT(42)
#define   CLUSTER_PSTATE_BUSY		BIT(31)
#define   CLUSTER_PSTATE_SET		BIT(25)
#define   CLUSTER_PSTATE_DISABLE	BIT(22)
#define   CLUSTER_PSTATE_TGT0_MASK	15
#define   CLUSTER_PSTATE_TGT_MASK	0xF00F
#define   CLUSTER_PSTATE_TGT1_SHIFT	12
#define CLUSTER_CPUTVM			0x20048
#define   CLUSTER_CPUTVM_ENABLE		BIT(0)
#define CLUSTER_PSCTRL			0x200f8
#define   CLUSTER_PSCTRL_ENABLE		BIT(40)
#define CLUSTER_DVMR			0x206b8
#define   CLUSTER_DVMR_ENABLE		BIT(63)
#define   CLUSTER_DVMR_ENABLE_PCPU	(BIT(32) | BIT(31))

#define CLUSTER_LIMIT2			0x40240
#define CLUSTER_LIMIT3			0x40250
#define CLUSTER_CTRL			0x440f8
#define   CLUSTER_CTRL_ENABLE		BIT(0)
#define CLUSTER_LIMIT1			0x48400
#define   CLUSTER_LIMIT_ENABLE		BIT(63)

#define CLUSTER_PSINFO1_SET		0x70210
#define CLUSTER_PSINFO2_SET		0x70218
#define CLUSTER_PSINFO1_GET(i)		(0x70000 + 0x20 * (i))
#define CLUSTER_PSINFO2_GET(i)		(0x70008 + 0x20 * (i))

#define PMGR_CPUGATING(cl)		(0x1c080 + 8 * (cl))
#define   PMGR_CPUGATING_ENABLE		BIT(31)
#define PMGR_CPUTVM0			0x48000
#define PMGR_CPUTVM1			0x48c00
#define PMGR_CPUTVM2			0x48800
#define PMGR_CPUTVM3			0x48400
#define   PMGR_CPUTVM_ENABLE		BIT(0)

#define MCC_NUM_LANES			8
#define MCC_DRAMCFG0(ln)		(0xdc4 + 0x40000 * (ln))
#define MCC_DRAMCFG1(ln)		(0xdbc + 0x40000 * (ln))

struct apple_m1_cpufreq_data {
	struct device *dev;
	void __iomem *base[NUM_BASES];
	struct apple_m1_cpufreq_cluster {
		void __iomem *base;
		struct apple_m1_cpufreq_data *hc;
		struct cpufreq_frequency_table *freqs;
		unsigned set_volt;
	} cluster[NUM_CLUSTERS];
	u32 pcpu_fast_freq;
	u32 pcpu_dramcfg[2][MCC_NUM_LANES][2];
};

static inline void rmwq(u64 clr, u64 set, void __iomem *base)
	{ writeq((readq(base) & ~clr) | set, base); }
static inline void rmwl(u32 clr, u32 set, void __iomem *base)
	{ writel((readl(base) & ~clr) | set, base); }

static int apple_m1_cpufreq_wait_idle(struct apple_m1_cpufreq_cluster *hcc)
{
	unsigned max = 1000;
	u32 state;

	while(max --) {
		state = readq(hcc->base + CLUSTER_PSTATE);
		if(!(state & CLUSTER_PSTATE_BUSY))
			return 0;

		udelay(50);
	}

	dev_err(hcc->hc->dev, "timed out waiting for pstate idle on cluster %d.\n",
		(int)(hcc - hcc->hc->cluster));
	return -ETIMEDOUT;
}

static int apple_m1_cpufreq_set_target(struct cpufreq_policy *policy, unsigned int index)
{
	struct apple_m1_cpufreq_cluster *hcc = policy->driver_data;
	struct apple_m1_cpufreq_data *hc = hcc->hc;
	int cluster = hcc - hc->cluster;
	void __iomem *mcc;
	unsigned set_volt, cfg, ln;
	int res;

	res = apple_m1_cpufreq_wait_idle(hcc);
	if(res < 0)
		return res;

	set_volt = (index >= SETVOLT_MIN_STATE);
	if(set_volt != hcc->set_volt)
		rmwq(0, CLUSTER_VCTRL_SETVOLT, hcc->base + CLUSTER_VCTRL);
	else
		set_volt = 0;

	rmwq(CLUSTER_PSTATE_TGT_MASK,
		CLUSTER_PSTATE_SET | (index << CLUSTER_PSTATE_TGT1_SHIFT) | index,
		hcc->base + CLUSTER_PSTATE);
	writeq(readq(hcc->base + CLUSTER_PSINFO1_GET(index)), hcc->base + CLUSTER_PSINFO1_SET);
	writeq(readq(hcc->base + CLUSTER_PSINFO2_GET(index)), hcc->base + CLUSTER_PSINFO2_SET);

	if(set_volt != hcc->set_volt)
		rmwq(0, CLUSTER_VCTRL_SETVOLT, hcc->base + CLUSTER_VCTRL);
	else
		rmwq(CLUSTER_VCTRL_SETVOLT, 0, hcc->base + CLUSTER_VCTRL);
	hcc->set_volt = set_volt;

	if(cluster == CLUSTER_PCPU && hc->pcpu_fast_freq != FAST_FREQ_INVALID) {
		/* tolerance for roundoff  */
		cfg = (hcc->freqs[index].frequency > (hc->pcpu_fast_freq * 1000 + 1000)) ? DRAMCFG_FAST : DRAMCFG_SLOW;
		mcc = hc->base[MCC_BASE];
		for(ln=0; ln<MCC_NUM_LANES; ln++) {
			writel(hc->pcpu_dramcfg[cfg][ln][0], mcc + MCC_DRAMCFG0(ln));
			writel(hc->pcpu_dramcfg[cfg][ln][1], mcc + MCC_DRAMCFG1(ln));
		}
	}

	res = apple_m1_cpufreq_wait_idle(hcc);
	if(res < 0)
		return res;

	return 0;
}

static int apple_m1_cpufreq_cpu_cluster(struct device *dev, int cpu)
{
	struct device_node *np;
	u32 cpucl;

	np = of_cpu_device_node_get(cpu);
	if(!np)
		return -1;

	if(of_property_read_u32(np, "apple,cluster", &cpucl) < 0) {
		of_node_put(np);
		dev_err(dev, "missing apple,cluster property on cpu %d.\n", cpu);
		return -1;
	}

	of_node_put(np);
	return cpucl;
}

static void apple_m1_cpufreq_cluster_mask(struct device *dev, int cluster, struct cpumask *mask)
{
	int cpu, cpucl;

	for_each_possible_cpu(cpu) {
		cpucl = apple_m1_cpufreq_cpu_cluster(dev, cpu);
		if(cluster == cpucl)
			cpumask_set_cpu(cpu, mask);
	}
}

static int apple_m1_cpufreq_init(struct cpufreq_policy *policy)
{
	struct apple_m1_cpufreq_data *hc = cpufreq_get_driver_data();
	struct apple_m1_cpufreq_cluster *hcc;
	int cluster, res;

	cluster = apple_m1_cpufreq_cpu_cluster(hc->dev, policy->cpu);
	if(cluster < 0)
		return -ENODEV;
	hcc = &hc->cluster[cluster];

	res = apple_m1_cpufreq_wait_idle(hcc);
	if(res < 0)
		return res;

	writeq(readq(hcc->base + CLUSTER_PSINFO1_GET(DFLT_STATE)),
		hcc->base + CLUSTER_PSINFO1_SET);
	writeq(readq(hcc->base + CLUSTER_PSINFO2_GET(DFLT_STATE)),
		hcc->base + CLUSTER_PSINFO2_SET);
	rmwq(CLUSTER_PSTATE_TGT_MASK,
	     CLUSTER_PSTATE_SET | (DFLT_STATE << CLUSTER_PSTATE_TGT1_SHIFT) | DFLT_STATE,
	     hcc->base + CLUSTER_PSTATE);

	res = apple_m1_cpufreq_wait_idle(hcc);
	if(res < 0)
		return res;

	apple_m1_cpufreq_cluster_mask(hc->dev, cluster, policy->cpus);
	policy->driver_data = &hc->cluster[cluster];
	policy->freq_table = hc->cluster[cluster].freqs;
	policy->cpuinfo.transition_latency = 100;
	policy->cur = hc->cluster[cluster].freqs[DFLT_STATE].frequency;

	return 0;
}

static struct cpufreq_driver apple_m1_cpufreq_driver = {
	.name = "apple-m1-pmgr",
	.flags = CPUFREQ_STICKY | CPUFREQ_HAVE_GOVERNOR_PER_POLICY,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = apple_m1_cpufreq_set_target,
	.init = apple_m1_cpufreq_init,
	.attr = cpufreq_generic_attr,
};

static struct cpufreq_frequency_table *apple_m1_cpufreq_dt_to_table(struct device *dev,
								    const char *ftname)
{
	int cnt, idx;
	u32 freq;
	struct cpufreq_frequency_table *ft;

	/* table in dtree is in clock cycle (in fixpoint-16 microseconds), voltage pairs */
	cnt = of_property_count_elems_of_size(dev->of_node, ftname, sizeof(u32) * 2);
	if(cnt < 0) {
		dev_err(dev, "missing frequency table '%s' in dt.\n", ftname);
		return ERR_PTR(-ENOENT);
	}

	ft = devm_kzalloc(dev, sizeof(ft[0]) * (cnt + 2), GFP_KERNEL);
	if(!ft)
		return ERR_PTR(-ENOMEM);

	ft[0].frequency = CPUFREQ_ENTRY_INVALID;
	for(idx=0; idx<cnt; idx++) {
		if(of_property_read_u32_index(dev->of_node, ftname, idx * 2, &freq) < 0)
			return ERR_PTR(-EINVAL);

		ft[idx+1].frequency = CPUFREQ_ENTRY_INVALID;
		if(freq >= 16) { /* result should fit in u32 */
			freq = (65536 * 1000000ull) / freq; /* fixpoint cycle time to kHz */
			ft[idx+1].frequency = freq;
		}
	}
	ft[cnt+1].frequency = CPUFREQ_TABLE_END;

	return ft;
}

static void apple_m1_cpufreq_hwsetup_cluster(struct apple_m1_cpufreq_data *hc, int cluster)
{
	void __iomem *base = hc->base[cluster];
	void __iomem *pmgr = hc->base[PMGR_BASE];
	unsigned ps;

	rmwq(0, CLUSTER_DVMR_ENABLE | (cluster ? CLUSTER_DVMR_ENABLE_PCPU : 0),
	     base + CLUSTER_DVMR);

	rmwq(0, CLUSTER_LIMIT_ENABLE, base + CLUSTER_LIMIT1);
	rmwq(CLUSTER_LIMIT_ENABLE, 0, base + CLUSTER_LIMIT2);
	rmwq(0, CLUSTER_LIMIT_ENABLE, base + CLUSTER_LIMIT3);

	rmwq(0, CLUSTER_PSTATE_LOCKCTRL, base + CLUSTER_PSTATE);

	rmwl(0, PMGR_CPUGATING_ENABLE, pmgr + PMGR_CPUGATING(cluster));

	writeq(CLUSTER_CTRL_ENABLE, base + CLUSTER_CTRL);

	ps = readq(base + CLUSTER_PSTATE) & CLUSTER_PSTATE_TGT0_MASK;
	rmwq(0, CLUSTER_PSCTRL_ENABLE, base + CLUSTER_PSCTRL);
	writeq(readq(base + CLUSTER_PSINFO1_GET(ps)), base + CLUSTER_PSINFO1_SET);
	writeq(readq(base + CLUSTER_PSINFO2_GET(ps)), base + CLUSTER_PSINFO2_SET);
}

static void apple_m1_cpufreq_hwsetup(struct apple_m1_cpufreq_data *hc)
{
	void __iomem **base = hc->base;
	void __iomem *pmgr = hc->base[PMGR_BASE];
	void __iomem *mcc = hc->base[MCC_BASE];
	unsigned ln;

	for(ln=0; ln<MCC_NUM_LANES; ln++) {
		hc->pcpu_dramcfg[DRAMCFG_SLOW][ln][0] = readl(mcc + MCC_DRAMCFG0(ln));
		hc->pcpu_dramcfg[DRAMCFG_SLOW][ln][1] = readl(mcc + MCC_DRAMCFG1(ln));
		if(!ln)
			continue;
		hc->pcpu_dramcfg[DRAMCFG_FAST][ln][0] = hc->pcpu_dramcfg[DRAMCFG_FAST][0][0];
		hc->pcpu_dramcfg[DRAMCFG_FAST][ln][1] = hc->pcpu_dramcfg[DRAMCFG_FAST][0][1];
	}

	rmwq(CLUSTER_PSTATE_DISABLE, 0, base[0] + CLUSTER_PSTATE);
	rmwq(CLUSTER_PSTATE_DISABLE, 0, base[1] + CLUSTER_PSTATE);

	rmwq(0, CLUSTER_CPUTVM_ENABLE, base[0] + CLUSTER_CPUTVM);
	rmwq(0, CLUSTER_CPUTVM_ENABLE, base[1] + CLUSTER_CPUTVM);
	rmwl(0, PMGR_CPUTVM_ENABLE, pmgr + PMGR_CPUTVM0);
	rmwl(0, PMGR_CPUTVM_ENABLE, pmgr + PMGR_CPUTVM1);
	rmwl(0, PMGR_CPUTVM_ENABLE, pmgr + PMGR_CPUTVM2);
	rmwl(0, PMGR_CPUTVM_ENABLE, pmgr + PMGR_CPUTVM3);

	apple_m1_cpufreq_hwsetup_cluster(hc, 0);
	apple_m1_cpufreq_hwsetup_cluster(hc, 1);
}

static int apple_m1_cpufreq_probe(struct platform_device *pdev)
{
	struct apple_m1_cpufreq_data *hc;
	struct device_node *np = pdev->dev.of_node;
	struct resource *res;
	unsigned int i = 0, err;

	hc = devm_kzalloc(&pdev->dev, sizeof(*hc), GFP_KERNEL);
	if(!hc)
		return -ENOMEM;

	hc->dev = &pdev->dev;

	for(i=0; i<NUM_BASES; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		hc->base[i] = devm_ioremap_resource(&pdev->dev, res);
		if(IS_ERR(hc->base[i])) {
			err = PTR_ERR(hc->base[i]);
			return err;
		}
	}

	for(i=0; i<NUM_CLUSTERS; i++) {
		hc->cluster[i].hc = hc;
		hc->cluster[i].base = hc->base[i];
	}

	hc->cluster[0].freqs = apple_m1_cpufreq_dt_to_table(&pdev->dev, "tunable-ecpu-states");
	if(IS_ERR(hc->cluster[0].freqs))
		return PTR_ERR(hc->cluster[0].freqs);

	hc->cluster[1].freqs = apple_m1_cpufreq_dt_to_table(&pdev->dev, "tunable-pcpu-states");
	if(IS_ERR(hc->cluster[1].freqs))
		return PTR_ERR(hc->cluster[1].freqs);

	if(of_property_read_u32(np, "tunable-pcpu-fast-freq", &hc->pcpu_fast_freq) < 0 ||
	   of_property_read_u32_array(np, "tunable-pcpu-fast-dcfg",
				      hc->pcpu_dramcfg[DRAMCFG_FAST][0], 2) < 0) {
		dev_warn(&pdev->dev, "tunable-pcpu-fast-* entries not in dtree, limited clock scaling.\n");
		hc->pcpu_fast_freq = FAST_FREQ_INVALID;
	}

	apple_m1_cpufreq_hwsetup(hc);

	apple_m1_cpufreq_driver.driver_data = hc;
	err = cpufreq_register_driver(&apple_m1_cpufreq_driver);
	if(err)
		return err;

	dev_info(&pdev->dev, "registered cpufreq driver.\n");
	return 0;
}

static const struct of_device_id apple_m1_cpufreq_of_match[] = {
	{ .compatible = "apple,pmgr-cpufreq-m1" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_m1_cpufreq_of_match);

static struct platform_driver apple_m1_cpufreq_platform_driver = {
	.driver = {
		.name = "apple-m1-cpufreq",
		.of_match_table = apple_m1_cpufreq_of_match,
	},
	.probe = apple_m1_cpufreq_probe,
};
module_platform_driver(apple_m1_cpufreq_platform_driver);

MODULE_DESCRIPTION("Apple M1 SoC cpufreq driver");
MODULE_LICENSE("GPL v2");
