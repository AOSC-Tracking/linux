// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Apple SMC as found on M1 SoC
 *
 * Copyright (C) 2021 Corellium LLC
 */

#include <linux/bitops.h>
#include <linux/gpio/driver.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regmap.h>
#include <linux/nvmem-consumer.h>
#include <linux/mailbox_client.h>

#define MAX_GPIO		32

#define SMC_READ_KEY		0x10
#define SMC_WRITE_KEY		0x11
#define SMC_GET_KEY_BY_INDEX	0x12
#define SMC_GET_KEY_INFO	0x13
#define SMC_GET_SRAM_ADDR	0x17
#define SMC_NOTIFICATION	0x18
#define SMC_READ_KEY_PAYLOAD	0x20

#define SMC_BUF_SIZE		0x4000
#define SMC_MAX_KEYS		1024
#define SMC_TIMEOUT_MSEC	250

#define MAX_POWEROFF		4

struct apple_m1_smc {
	struct device *dev;
	struct mbox_client mbox;
	struct mbox_chan *chan;
	struct completion cmdcompl;
	unsigned long timeout;
	u64 rxmsg[2];
	void __iomem *buf;
	struct mutex lock;
	u32 msgid;

	struct gpio_chip gpio;
	u32 gpio_present_mask;
	u32 gpio_bits[MAX_GPIO];

	void (*old_pm_power_off)(void);
	struct nvmem_device *nvmem;
	u32 nvmem_poweroff[MAX_POWEROFF*3]; /* addr, mask, val */
	unsigned num_poweroff;
};

static struct apple_m1_smc *apple_m1_smc_instance = NULL;

struct apple_m1_smc_key_info {
	u8 size;
	u32 type;
	u8 flags;
} __packed;

static void apple_m1_smc_write_buf(void __iomem *buf, const void *mem, size_t size)
{
	u32 __iomem *bufw = buf;
	const u32 *memw = mem;
	u32 tmp;

	while(size >= 4) {
		writel(*(memw ++), bufw);
		bufw ++;
		size -= 4;
	}

	if(size) {
		tmp = 0;
		memcpy(&tmp, memw, size);
		writel(tmp, bufw);
	}
}

static void apple_m1_smc_read_buf(void *mem, void __iomem *buf, size_t size)
{
	u32 __iomem *bufw = buf;
	u32 *memw = mem;
	u32 tmp;

	while(size >= 4) {
		*(memw ++) = readl(bufw);
		bufw ++;
		size -= 4;
	}

	if(size) {
		tmp = readl(bufw);
		memcpy(memw, &tmp, size);
	}
}

static int apple_m1_smc_cmd(struct apple_m1_smc *smc, u8 cmd, u16 hparam, u32 wparam,
			    const void *din, unsigned dilen, void *dout, unsigned dolen, u64 *out)
{
	int ret;
	u64 msg[2], msg0;

	if(dilen > SMC_BUF_SIZE || dolen > SMC_BUF_SIZE)
		return -EFBIG;

	mutex_lock(&smc->lock);
	if(dilen && din)
		apple_m1_smc_write_buf(smc->buf, din, dilen);
	msg[0] = cmd | ((u64)hparam << 16) | ((u64)wparam << 32) | (smc->msgid << 12);
	msg[1] = 0;
	smc->msgid = (smc->msgid + 1) & 15;

	init_completion(&smc->cmdcompl);
	ret = mbox_send_message(smc->chan, msg);
	if(ret >= 0)
		ret = wait_for_completion_timeout(&smc->cmdcompl, smc->timeout) ? 0 : -ETIMEDOUT;

	if(ret >= 0) {
		if(dolen && dout)
			apple_m1_smc_read_buf(dout, smc->buf, dolen);
		if(out)
			*out = smc->rxmsg[0];
		msg0 = smc->rxmsg[0];
	}
	mutex_unlock(&smc->lock);

	if(ret < 0) {
		dev_warn(smc->dev, "command [%016llx] failed: %d.\n", msg[0], ret);
		return ret;
	}

	ret = msg0 & 255;
	if(ret) {
		if(cmd != SMC_GET_KEY_BY_INDEX) /* key enumeration would be noisy */
			dev_warn(smc->dev, "command [%016llx] failed: %d.\n", msg[0], ret);
		return -EIO;
	}
	return 0;
}

static int apple_m1_smc_write_key(struct apple_m1_smc *smc, u32 key, const void *data, size_t size)
{
	return apple_m1_smc_cmd(smc, SMC_WRITE_KEY, size, key, data, size, NULL, 0, NULL);
}

static int apple_m1_smc_get_key_info(struct apple_m1_smc *smc, u32 key,
				     struct apple_m1_smc_key_info *ki)
{
	return apple_m1_smc_cmd(smc, SMC_GET_KEY_INFO, 0, key, NULL, 0, ki, sizeof(*ki), NULL);
}

static int apple_m1_smc_read_key_payload(struct apple_m1_smc *smc, u32 key,
					 const void *pld, size_t psize, void *data, size_t size)
{
	u64 out;
	int ret, outlen;

	ret = apple_m1_smc_cmd(smc, pld ? SMC_READ_KEY_PAYLOAD : SMC_READ_KEY, size | (psize << 8),
			       key, pld, psize, (size > 4) ? data : NULL, size, &out);
	if(ret < 0)
		return ret;

	outlen = (out >> 16) & 0xFFFF;
	if(outlen < size) {
		dev_warn(smc->dev, "READ_KEY [%08x, %d] result too big: %d.\n",
			 key, (int)size, outlen);
		return -ENOSPC;
	}

	if(outlen <= 4)
		memcpy(data, (void *)out + 4, size);
	return outlen;
}

static int apple_m1_smc_read_key(struct apple_m1_smc *smc, u32 key, void *data, size_t size)
{
	return apple_m1_smc_read_key_payload(smc, key, NULL, 0, data, size);
}

static int apple_m1_smc_get_key_by_index(struct apple_m1_smc *smc, u32 index, u32 *key)
{
	u64 out;
	int ret;

	ret = apple_m1_smc_cmd(smc, SMC_GET_KEY_BY_INDEX, 0, index, NULL, 0, NULL, 0, &out);
	if(ret < 0)
		return ret;

	if(key)
		*key = swab32(out >> 32);
	return 0;
}

static u64 apple_m1_pack_hex(u32 val, unsigned len)
{
	unsigned i;
	u64 res = 0;
	for(i=0; i<len; i++) {
		res |= (u64)("0123456789abcdef"[val & 15]) << (i * 8);
		val >>= 4;
	}
	return res;
}

static int apple_m1_smc_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return -EINVAL;
}

static int apple_m1_smc_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct apple_m1_smc *smc = gpiochip_get_data(chip);
	u32 key, data;

	if(!(smc->gpio_present_mask & (1 << offset)))
		return -ENODEV;

	key = 0x67500000 | apple_m1_pack_hex(offset, 2); /* gP-- */
	data = (!!value) | smc->gpio_bits[offset];
	return apple_m1_smc_write_key(smc, key, &data, sizeof(data));
}

static int apple_m1_smc_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

static int apple_m1_smc_get(struct gpio_chip *chip, unsigned int offset)
{
	struct apple_m1_smc *smc = gpiochip_get_data(chip);
	u32 key, data;
	int ret;

	if(!(smc->gpio_present_mask & (1 << offset)))
		return -ENODEV;

	key = 0x67500000 | apple_m1_pack_hex(offset, 2); /* gP-- */
	ret = apple_m1_smc_read_key(smc, key, &data, sizeof(data));
	if(ret < 0)
		return ret;
	return data & 1;
}

static void apple_m1_smc_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	apple_m1_smc_direction_output(chip, offset, value);
}

static void apple_m1_smc_power_off(void)
{
	struct apple_m1_smc *smc = apple_m1_smc_instance;
	int ret, idx;
	u32 data;

	if(!smc)
		return;

	if(smc->nvmem)
		for(idx=0; idx<smc->num_poweroff; idx++) {
			ret = nvmem_device_read(smc->nvmem, smc->nvmem_poweroff[idx * 3], 1, &data);
			if(ret != 1) {
				pr_err("SMC: %d reading NVMEM %d.\n", ret, idx);
				break;
			}
			data &= ~smc->nvmem_poweroff[idx * 3 + 1];
			data |= smc->nvmem_poweroff[idx * 3 + 2];
			ret = nvmem_device_write(smc->nvmem, smc->nvmem_poweroff[idx * 3], 1, &data);
			if(ret != 1) {
				pr_err("SMC: %d writing NVMEM %d.\n", ret, idx);
				break;
			}
		}

	mdelay(50);

	data = 0x6f666631; /* 'off1' */
	ret = apple_m1_smc_write_key(smc, 0x4d425345, &data, sizeof(data)); /* 'MBSE' */
	if(ret < 0)
		pr_err("Failed to write SMC power-off key: %d\n", ret);
	mdelay(1000);

	pm_power_off = smc->old_pm_power_off;
	if(pm_power_off)
		pm_power_off();
}

static int apple_m1_smc_enumerate(struct apple_m1_smc *smc)
{
	unsigned idx;
	u32 key;
	struct apple_m1_smc_key_info ki;
	int ret;

	for(idx=0; idx<SMC_MAX_KEYS; idx++) {
		ret = apple_m1_smc_get_key_by_index(smc, idx, &key);
		if(ret)
			break;

		ret = apple_m1_smc_get_key_info(smc, key, &ki);
		if(ret)
			continue;
	}

	return 0;
}

static void apple_m1_smc_mbox_msg(struct mbox_client *cl, void *msg)
{
	struct apple_m1_smc *smc = container_of(cl, struct apple_m1_smc, mbox);
	u64 *rxmsg = msg;

	if((rxmsg[0] & 0xFF) == SMC_NOTIFICATION) {
		dev_info(smc->dev, "notification: %016llx.\n", rxmsg[0]);
		return;
	}

	smc->rxmsg[0] = rxmsg[0];
	smc->rxmsg[1] = rxmsg[1];
	complete(&smc->cmdcompl);
}

static void apple_m1_smc_free_channel(void *data)
{
	struct apple_m1_smc *smc = data;

	mbox_free_channel(smc->chan);
}

static int apple_m1_smc_probe(struct platform_device *pdev)
{
	struct apple_m1_smc *smc;
	struct device_node *node = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	char name[16];
	u64 msg[2];
	int ret, i;

	smc = devm_kzalloc(&pdev->dev, sizeof(*smc), GFP_KERNEL);
	if (!smc)
		return -ENOMEM;
	smc->dev = dev;
	mutex_init(&smc->lock);
	init_completion(&smc->cmdcompl);
	smc->timeout = msecs_to_jiffies(SMC_TIMEOUT_MSEC);

	for(i=0; i<MAX_GPIO; i++) {
		snprintf(name, sizeof(name), "gpio-%d", i);
		if(device_property_read_u32(dev, name, &smc->gpio_bits[i]) >= 0)
			smc->gpio_present_mask |= 1 << i;
	}

	smc->mbox.dev = dev;
	smc->mbox.rx_callback = apple_m1_smc_mbox_msg;
	smc->mbox.tx_block = true;
	smc->mbox.tx_tout = SMC_TIMEOUT_MSEC;
	smc->chan = mbox_request_channel(&smc->mbox, 0);
	if(IS_ERR(smc->chan)) {
		dev_err(dev, "failed to attach to mailbox: %ld.\n", PTR_ERR(smc->chan));
		return PTR_ERR(smc->chan);
	}

	ret = devm_add_action(dev, apple_m1_smc_free_channel, smc);
	if (ret)
		return ret;

	ret = apple_m1_smc_cmd(smc, SMC_GET_SRAM_ADDR, 0, 0, NULL, 0, NULL, 0, msg);
	if(ret) {
		dev_err(dev, "failed to start SMC: %d.\n", ret);
		return ret;
	}
	smc->buf = devm_ioremap(dev, msg[0], SMC_BUF_SIZE);
	if(!smc->buf) {
		dev_err(dev, "failed to map SMC buffer at 0x%llx.\n", msg[0]);
		return -EINVAL;
	}

	smc->nvmem = of_nvmem_device_get(node, NULL);
	if(IS_ERR(smc->nvmem) || !smc->nvmem) {
		if(PTR_ERR(smc->nvmem) == -EPROBE_DEFER)
			return PTR_ERR(smc->nvmem);
		dev_warn(&pdev->dev, "No 'nvmem' parameter, will not be able to power off: %ld.\n", PTR_ERR(smc->nvmem));
		smc->nvmem = NULL;
	}

	if(smc->nvmem) {
		ret = of_property_read_variable_u32_array(node, "nvmem-poweroff", smc->nvmem_poweroff, 3, MAX_POWEROFF * 3);
		if(ret < 3 || (ret % 3)) {
			dev_err(&pdev->dev, "'nvmem-poweroff' parameter must have triplets of elements: %d.\n", ret);
			return ret;
		}
		smc->num_poweroff = ret / 3;
	}

	ret = apple_m1_smc_enumerate(smc);
	if(ret < 0)
		return ret;

	smc->gpio.direction_input = apple_m1_smc_direction_input;
	smc->gpio.direction_output = apple_m1_smc_direction_output;
	smc->gpio.get_direction = apple_m1_smc_get_direction;
	smc->gpio.get = apple_m1_smc_get;
	smc->gpio.set = apple_m1_smc_set;

	smc->gpio.ngpio = MAX_GPIO;
	smc->gpio.label = "apple-m1-smc";

	smc->gpio.base = -1;
	smc->gpio.can_sleep = true;
	smc->gpio.parent = &pdev->dev;
	smc->gpio.owner = THIS_MODULE;

	ret = devm_gpiochip_add_data(&pdev->dev, &smc->gpio, smc);
	if(ret)
		return ret;

	if(!apple_m1_smc_instance) {
		apple_m1_smc_instance = smc;
		smc->old_pm_power_off = pm_power_off;
		pm_power_off = apple_m1_smc_power_off;
	}

	return 0;
}

static const struct of_device_id apple_m1_smc_of_match[] = {
	{ .compatible = "apple,smc-m1" },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_m1_smc_of_match);

static struct platform_driver apple_m1_smc_platform_driver = {
	.driver = {
		.name = "apple-m1-smc",
		.of_match_table = apple_m1_smc_of_match,
	},
	.probe = apple_m1_smc_probe,
};
module_platform_driver(apple_m1_smc_platform_driver);

MODULE_DESCRIPTION("Apple M1 SMC driver");
MODULE_LICENSE("GPL v2");
