// SPDX-License-Identifier: GPL-2.0-only
/*
 * CPUFreq driver for the Loongson-3 processors.
 *
 * All revisions of Loongson-3 processor support cpu_has_scalefreq feature.
 *
 * Author: Huacai Chen <chenhuacai@loongson.cn>
 * Copyright (C) 2024 Loongson Technology Corporation Limited
 */
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/units.h>

#include <asm/idle.h>
#include <asm/loongarch.h>
#include <asm/loongson.h>
#include <asm/time.h>

/* Message */
union smc_message {
	u32 value;

	/* Generic message type */
	struct {
		u32 id		: 4;
		u32 info	: 4;
		u32 val		: 16;
		u32 cmd		: 6;
		u32 extra	: 1;
		u32 complete	: 1;
	};

	/* Returned by SMC0_GET_DVFS_INFO */
	struct {
		u32 min_level		: 4;
		u32 max_level		: 4;
		u32 boost_freq		: 16;
		u32 normal_core_limit	: 4;
		u32 boost_cores		: 4;
	};
};

/* Command return values */
#define CMD_OK				0 /* No error */
#define CMD_ERROR			1 /* Regular error */
#define CMD_NOCMD			2 /* Command does not support */
#define CMD_INVAL			3 /* Invalid Parameter */

/* Version commands */
/*
 * CMD_GET_VERSION - Get interface version
 * Input: none
 * Output: version
 */
#define CMD_GET_VERSION			0x1

/* SMC version 0 service calls */
/*
 * SMC0_CMD_SET_FREQ_LEVEL - Set frequency level
 * Input: CPU ID, level ID
 * Output: none
 */
#define SMC0_CMD_SET_FREQ_LEVEL		0x21

/*
 * SMC0_CMD_GET_DVFS_INFO - Get DVFS information
 * Input: CPU ID
 * Output: DVFS Information
 */
#define SMC0_CMD_GET_DVFS_INFO		0x22

/* SMC version 1 service calls */
/* Feature commands */
/*
 * SMC1_CMD_GET_FEATURE - Get feature state
 * Input: feature ID
 * Output: feature flag
 */
#define SMC1_CMD_GET_FEATURE		0x2

/*
 * SMC1_CMD_SET_FEATURE - Set feature state
 * Input: feature ID, feature flag
 * output: none
 */
#define SMC1_CMD_SET_FEATURE		0x3

/* Feature IDs */
#define SMC1_FEATURE_SENSOR		0
#define SMC1_FEATURE_FAN		1
#define SMC1_FEATURE_DVFS		2

/* Sensor feature flags */
#define SMC1_FEATURE_SENSOR_ENABLE	BIT(0)
#define SMC1_FEATURE_SENSOR_SAMPLE	BIT(1)

/* Fan feature flags */
#define SMC1_FEATURE_FAN_ENABLE		BIT(0)
#define SMC1_FEATURE_FAN_AUTO		BIT(1)

/* DVFS feature flags */
#define SMC1_FEATURE_DVFS_ENABLE	BIT(0)
#define SMC1_FEATURE_DVFS_BOOST		BIT(1)
#define SMC1_FEATURE_DVFS_AUTO		BIT(2)
#define SMC1_FEATURE_DVFS_SINGLE_BOOST	BIT(3)

/* Sensor commands */
/*
 * SMC1_CMD_GET_SENSOR_NUM - Get number of sensors
 * Input: none
 * Output: number
 */
#define SMC1_CMD_GET_SENSOR_NUM		0x4

/*
 * SMC1_CMD_GET_SENSOR_STATUS - Get sensor status
 * Input: sensor ID, type
 * Output: sensor status
 */
#define SMC1_CMD_GET_SENSOR_STATUS	0x5

/* Sensor types */
#define SMC1_SENSOR_INFO_TYPE		0
#define SMC1_SENSOR_INFO_TYPE_TEMP	1

/* Fan commands */
/*
 * SMC1_CMD_GET_FAN_NUM - Get number of fans
 * Input: none
 * Output: number
 */
#define SMC1_CMD_GET_FAN_NUM		0x6

/*
 * SMC1_CMD_GET_FAN_INFO - Get fan status
 * Input: fan ID, type
 * Output: fan info
 */
#define SMC1_CMD_GET_FAN_INFO		0x7

/*
 * SMC1_CMD_SET_FAN_INFO - Set fan status
 * Input: fan ID, type, value
 * Output: none
 */
#define SMC1_CMD_SET_FAN_INFO		0x8

/* Fan types */
#define SMC1_FAN_INFO_TYPE_LEVEL	0

/* DVFS commands */
/*
 * SMC1_CMD_GET_FREQ_LEVEL_NUM - Get number of freq levels
 * Input: CPU ID
 * Output: number
 */
#define SMC1_CMD_GET_FREQ_LEVEL_NUM	0x9

/*
 * SMC1_CMD_GET_FREQ_BOOST_LEVEL - Get the first boost level
 * Input: CPU ID
 * Output: number
 */
#define SMC1_CMD_GET_FREQ_BOOST_LEVEL	0x10

/*
 * SMC1_CMD_GET_FREQ_LEVEL_INFO - Get freq level info
 * Input: CPU ID, level ID
 * Output: level info
 */
#define SMC1_CMD_GET_FREQ_LEVEL_INFO	0x11

/*
 * SMC1_CMD_GET_FREQ_INFO - Get freq info
 * Input: CPU ID, type
 * Output: freq info
 */
#define SMC1_CMD_GET_FREQ_INFO		0x12

/*
 * SMC1_CMD_SET_FREQ_INFO - Set freq info
 * Input: CPU ID, type, value
 * Output: none
 */
#define SMC1_CMD_SET_FREQ_INFO		0x13

/* Freq types */
#define SMC1_FREQ_INFO_TYPE_FREQ	0
#define SMC1_FREQ_INFO_TYPE_LEVEL	1

#define SMC1_FREQ_MAX_LEVEL		16

struct loongson3_freq_data {
	unsigned int min_freq_level, def_freq_level;
	struct cpufreq_frequency_table table[];
};

static struct mutex cpufreq_mutex[MAX_PACKAGES];
static struct cpufreq_driver *loongson3_cpufreq_current_driver;
static DEFINE_PER_CPU(struct loongson3_freq_data *, freq_data);

static inline int do_service_request_raw(u32 id, u32 info, u32 cmd, u32 val,
					 u32 extra, union smc_message *raw)
{
	int retries;
	unsigned int cpu = raw_smp_processor_id();
	unsigned int package = cpu_data[cpu].package;
	union smc_message msg, last;

	mutex_lock(&cpufreq_mutex[package]);

	last.value = iocsr_read32(LOONGARCH_IOCSR_SMCMBX);
	if (!last.complete) {
		mutex_unlock(&cpufreq_mutex[package]);
		return -EPERM;
	}

	msg.id		= id;
	msg.info	= info;
	msg.cmd		= cmd;
	msg.val		= val;
	msg.extra	= extra;
	msg.complete	= 0;

	iocsr_write32(msg.value, LOONGARCH_IOCSR_SMCMBX);
	iocsr_write32(iocsr_read32(LOONGARCH_IOCSR_MISC_FUNC) | IOCSR_MISC_FUNC_SOFT_INT,
		      LOONGARCH_IOCSR_MISC_FUNC);

	for (retries = 0; retries < 10000; retries++) {
		msg.value = iocsr_read32(LOONGARCH_IOCSR_SMCMBX);
		if (msg.complete)
			break;

		usleep_range(8, 12);
	}

	if (!msg.complete || msg.cmd != CMD_OK) {
		mutex_unlock(&cpufreq_mutex[package]);
		return -EPERM;
	}

	mutex_unlock(&cpufreq_mutex[package]);

	if (raw)
		*raw = msg;

	return msg.val;
}

#define do_service_request(id, info, cmd, val, extra) \
	do_service_request_raw(id, info, cmd, val, extra, NULL)

static unsigned int loongson3_cpufreq_smc1_get(unsigned int cpu)
{
	int ret;

	ret = do_service_request(cpu, SMC1_FREQ_INFO_TYPE_FREQ,
				 SMC1_CMD_GET_FREQ_INFO, 0, 0);

	return ret * KILO;
}

static int loongson3_cpufreq_smc0_target(struct cpufreq_policy *policy, unsigned int index)
{
	unsigned int cpu = policy->cpu;
	int ret;

	index += per_cpu(freq_data, cpu)->min_freq_level;

	ret = do_service_request(cpu_data[cpu].core, index, SMC0_CMD_SET_FREQ_LEVEL,
				 0, 0);

	return (ret >= 0) ? 0 : ret;
}

static int loongson3_cpufreq_smc1_target(struct cpufreq_policy *policy, unsigned int index)
{
	int ret;

	ret = do_service_request(cpu_data[policy->cpu].core,
				 SMC1_FREQ_INFO_TYPE_LEVEL, SMC1_CMD_SET_FREQ_INFO, index, 0);

	return (ret >= 0) ? 0 : ret;
}

static int configure_smc0_freq_table(int cpu)
{
	struct platform_device *pdev = cpufreq_get_driver_data();
	struct loongson3_freq_data *data;
	int ret, freq_level, i;
	union smc_message msg;

	if (per_cpu(freq_data, cpu))
		return 0;

	ret = do_service_request_raw(cpu, 0, SMC0_CMD_GET_DVFS_INFO, 0, 0, &msg);
	if (ret < 0)
		return ret;

	freq_level = msg.max_level - msg.min_level + 1;
	data = devm_kzalloc(&pdev->dev, struct_size(data, table, freq_level + 1),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->def_freq_level = 0;
	data->min_freq_level = msg.min_level;

	for (i = 0; i < freq_level; i++) {
		unsigned long frequency;

		frequency = cpu_clock_freq / KILO;
		frequency = frequency * (freq_level - i) / freq_level;

		data->table[i].frequency = frequency;
		data->table[i].flags = 0;
	}

	data->table[freq_level].frequency = CPUFREQ_TABLE_END;
	data->table[freq_level].flags = 0;

	per_cpu(freq_data, cpu) = data;

	return 0;
}

static int configure_smc1_freq_table(int cpu)
{
	int i, ret, boost_level, max_level, freq_level;
	struct platform_device *pdev = cpufreq_get_driver_data();
	struct loongson3_freq_data *data;

	if (per_cpu(freq_data, cpu))
		return 0;

	ret = do_service_request(cpu, 0, SMC1_CMD_GET_FREQ_LEVEL_NUM, 0, 0);
	if (ret < 0)
		return ret;
	max_level = ret;

	ret = do_service_request(cpu, 0, SMC1_CMD_GET_FREQ_BOOST_LEVEL, 0, 0);
	if (ret < 0)
		return ret;
	boost_level = ret;

	freq_level = min(max_level, SMC1_FREQ_MAX_LEVEL);
	data = devm_kzalloc(&pdev->dev, struct_size(data, table, freq_level + 1), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->def_freq_level = boost_level - 1;

	for (i = 0; i < freq_level; i++) {
		ret = do_service_request(cpu, SMC1_FREQ_INFO_TYPE_FREQ,
					 SMC1_CMD_GET_FREQ_LEVEL_INFO, i, 0);
		if (ret < 0) {
			devm_kfree(&pdev->dev, data);
			return ret;
		}

		data->table[i].frequency = ret * KILO;
		data->table[i].flags = (i >= boost_level) ? CPUFREQ_BOOST_FREQ : 0;
	}

	data->table[freq_level].flags = 0;
	data->table[freq_level].frequency = CPUFREQ_TABLE_END;

	per_cpu(freq_data, cpu) = data;

	return 0;
}

static void loongson3_cpufreq_init_data(struct cpufreq_policy *policy)
{
	struct loongson3_freq_data *data;
	int i, cpu = policy->cpu;

	data = per_cpu(freq_data, cpu);
	policy->cpuinfo.transition_latency = 10000;
	policy->freq_table = data->table;
	policy->suspend_freq = data->table[data->def_freq_level].frequency;
	cpumask_copy(policy->cpus, topology_sibling_cpumask(cpu));

	for_each_cpu(i, policy->cpus) {
		if (i != cpu)
			per_cpu(freq_data, i) = per_cpu(freq_data, cpu);
	}
}

static int loongson3_cpufreq_cpu_smc0_init(struct cpufreq_policy *policy)
{
	int ret, cpu = policy->cpu;

	ret = configure_smc0_freq_table(cpu);
	if (ret < 0)
		return ret;

	loongson3_cpufreq_init_data(policy);

	return 0;
}

static int loongson3_cpufreq_cpu_smc1_init(struct cpufreq_policy *policy)
{
	int ret, cpu = policy->cpu;

	ret = configure_smc1_freq_table(cpu);
	if (ret < 0)
		return ret;

	loongson3_cpufreq_init_data(policy);

	if (policy_has_boost_freq(policy)) {
		ret = cpufreq_enable_boost_support();
		if (ret < 0) {
			pr_warn("cpufreq: Failed to enable boost: %d\n", ret);
			return ret;
		}
		loongson3_cpufreq_current_driver->boost_enabled = true;
	}

	return 0;
}

static void loongson3_cpufreq_cpu_exit(struct cpufreq_policy *policy)
{
	unsigned int def_freq_level;
	int cpu = policy->cpu;

	def_freq_level = per_cpu(freq_data, cpu)->def_freq_level;
	loongson3_cpufreq_current_driver->target_index(policy, def_freq_level);
}

static int loongson3_cpufreq_cpu_online(struct cpufreq_policy *policy)
{
	return 0;
}

static int loongson3_cpufreq_cpu_offline(struct cpufreq_policy *policy)
{
	return 0;
}

static struct cpufreq_driver loongson3_cpufreq_smc0_driver = {
	.name = "loongson3",
	.flags = CPUFREQ_CONST_LOOPS,
	.init = loongson3_cpufreq_cpu_smc0_init,
	.exit = loongson3_cpufreq_cpu_exit,
	.online = loongson3_cpufreq_cpu_online,
	.offline = loongson3_cpufreq_cpu_offline,
	.target_index = loongson3_cpufreq_smc0_target,
	.verify = cpufreq_generic_frequency_table_verify,
	.suspend = cpufreq_generic_suspend,
};

static struct cpufreq_driver loongson3_cpufreq_smc1_driver = {
	.name = "loongson3",
	.flags = CPUFREQ_CONST_LOOPS,
	.init = loongson3_cpufreq_cpu_smc1_init,
	.exit = loongson3_cpufreq_cpu_exit,
	.online = loongson3_cpufreq_cpu_online,
	.offline = loongson3_cpufreq_cpu_offline,
	.get = loongson3_cpufreq_smc1_get,
	.target_index = loongson3_cpufreq_smc1_target,
	.verify = cpufreq_generic_frequency_table_verify,
	.suspend = cpufreq_generic_suspend,
};

static int loongson3_cpufreq_probe(struct platform_device *pdev)
{
	int i, ret, version;

	for (i = 0; i < MAX_PACKAGES; i++) {
		ret = devm_mutex_init(&pdev->dev, &cpufreq_mutex[i]);
		if (ret)
			return ret;
	}

	version = do_service_request(0, 0, CMD_GET_VERSION, 0, 0);
	if (version < 0)
		return -EPERM;

	pr_info("loongson3_cpufreq: firmware version %d\n", version);

	if (version == 0) {
		loongson3_cpufreq_current_driver = &loongson3_cpufreq_smc0_driver;
	} else {
		ret = do_service_request(SMC1_FEATURE_DVFS, 0, SMC1_CMD_SET_FEATURE,
					 SMC1_FEATURE_DVFS_ENABLE | SMC1_FEATURE_DVFS_BOOST, 0);
		if (ret < 0)
			return -EPERM;

		loongson3_cpufreq_current_driver = &loongson3_cpufreq_smc1_driver;
	}

	loongson3_cpufreq_current_driver->driver_data = pdev;

	ret = cpufreq_register_driver(loongson3_cpufreq_current_driver);
	if (ret)
		return ret;

	pr_info("cpufreq: Loongson-3 CPU frequency driver.\n");

	return 0;
}

static void loongson3_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(loongson3_cpufreq_current_driver);
}

static struct platform_device_id cpufreq_id_table[] = {
	{ "loongson3_cpufreq", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, cpufreq_id_table);

static struct platform_driver loongson3_platform_driver = {
	.driver = {
		.name = "loongson3_cpufreq",
	},
	.id_table = cpufreq_id_table,
	.probe = loongson3_cpufreq_probe,
	.remove = loongson3_cpufreq_remove,
};
module_platform_driver(loongson3_platform_driver);

MODULE_AUTHOR("Huacai Chen <chenhuacai@loongson.cn>");
MODULE_DESCRIPTION("CPUFreq driver for Loongson-3 processors");
MODULE_LICENSE("GPL");
