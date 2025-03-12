/*
 * LOONGSON fast speed I2C controller
 *
 * Copyright (C) 2024 Loongson Technology Corporation Limited
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

/* Loongson I2C offset registers */
#define I2C_CR1         0x00
#define I2C_CR2         0x04
#define I2C_DR          0x10
#define I2C_SR1         0x14
#define I2C_SR2         0x18
#define I2C_CCR         0x1C
#define I2C_TRISE       0x20
#define I2C_FLTR        0x24

/* Loongson I2C control 1*/
#define I2C_CR1_POS     BIT(11)
#define I2C_CR1_ACK     BIT(10)
#define I2C_CR1_STOP    BIT(9)
#define I2C_CR1_START   BIT(8)
#define I2C_CR1_PE      BIT(0)

/* Loongson I2C control 2 */
#define I2C_CR2_FREQ_MASK   GENMASK(5, 0)
#define I2C_CR2_FREQ(n)     ((n) & I2C_CR2_FREQ_MASK)
#define I2C_CR2_ITBUFEN     BIT(10)
#define I2C_CR2_ITEVTEN     BIT(9)
#define I2C_CR2_ITERREN     BIT(8)
#define I2C_CR2_IRQ_MASK    (I2C_CR2_ITBUFEN | \
				I2C_CR2_ITEVTEN | \
				I2C_CR2_ITERREN)

/* Loongson I2C Status 1 */
#define I2C_SR1_AF      BIT(10)
#define I2C_SR1_ARLO    BIT(9)
#define I2C_SR1_BERR    BIT(8)
#define I2C_SR1_TXE     BIT(7)
#define I2C_SR1_RXNE    BIT(6)
#define I2C_SR1_BTF     BIT(2)
#define I2C_SR1_ADDR    BIT(1)
#define I2C_SR1_SB      BIT(0)
#define I2C_SR1_ITEVTEN_MASK  (I2C_SR1_BTF | \
				I2C_SR1_ADDR | \
				I2C_SR1_SB)
#define I2C_SR1_ITBUFEN_MASK  (I2C_SR1_TXE | \
				I2C_SR1_RXNE)
#define I2C_SR1_ITERREN_MASK  (I2C_SR1_AF | \
				I2C_SR1_ARLO | \
				I2C_SR1_BERR)

/* Loongson I2C Status 2 */
#define I2C_SR2_BUSY		BIT(1)

/* Loongson I2C Control Clock */
#define I2C_CCR_FS		BIT(15)
#define I2C_CCR_DUTY		BIT(14)

enum ls_i2c_speed {
	LS_I2C_SPEED_STANDARD,	/* 100 kHz */
	LS_I2C_SPEED_FAST,	/* 400 kHz */
};

/**
 * struct priv_msg - client specific data
 * @addr: 8-bit slave addr, including r/w bit
 * @count: number of bytes to be transferred
 * @buf: data buffer
 * @stop: last I2C msg to be sent, i.e. STOP to be generated
 * @result: result of the transfer
 */
struct priv_msg {
	u8 addr;
	u32 count;
	u8 *buf;
	bool stop;
	int result;
};

/**
 * struct ls_i2c_dev - private data of the controller
 * @adap: I2C adapter for this controller
 * @dev: device for this controller
 * @base: virtual memory area
 * @complete: completion of I2C message
 * @speed: Standard or Fast are supported
 * @msg: I2C transfer information
 * @freq: I2C controller input clock freq
 */
struct ls_i2c_dev {
	struct i2c_adapter adap;
	struct device *dev;
	void __iomem *base;
	struct completion complete;
	int speed;
	struct priv_msg msg;
	int freq;
};

static inline void i2c_set_bits(void __iomem *reg, u32 mask)
{
	writel(readl(reg) | mask, reg);
}

static inline void i2c_clr_bits(void __iomem *reg, u32 mask)
{
	writel(readl(reg) & ~mask, reg);
}

static void ls_i2c_disable_irq(struct ls_i2c_dev *i2c_dev)
{
	void __iomem *reg = i2c_dev->base + I2C_CR2;

	i2c_clr_bits(reg, I2C_CR2_IRQ_MASK);
}

#define INPUT_DEFAULT_CLK 200000000

static int ls_i2c_hw_config(struct ls_i2c_dev *i2c_dev)
{
	u32 val;
	u32 ccr = 0;

	/* reference clock determination the cnfigure val(0x3f) */
	i2c_set_bits(i2c_dev->base + I2C_CR2, 0x3f);
	i2c_set_bits(i2c_dev->base + I2C_TRISE, 0x3f);

	if (i2c_dev->speed == LS_I2C_SPEED_STANDARD) {
		val = DIV_ROUND_UP(i2c_dev->freq, 100000 * 2);
	} else {
		val = DIV_ROUND_UP(i2c_dev->freq, 400000 * 3);

		/* Select Fast mode */
		ccr |= I2C_CCR_FS;
	}
	ccr |= val & 0xfff;
	writel(ccr, i2c_dev->base + I2C_CCR);

	/* Enable I2C */
	writel(I2C_CR1_PE, i2c_dev->base + I2C_CR1);

	return 0;
}

static int ls_i2c_wait_free_bus(struct ls_i2c_dev *i2c_dev)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(i2c_dev->base + I2C_SR2,
					status,
					!(status & I2C_SR2_BUSY),
					10, 2000);
	if (ret) {
		dev_dbg(i2c_dev->dev, "bus not free\n");
		ret = -EBUSY;
	}

	return ret;
}

static int i2c_wait_EV_signal(struct ls_i2c_dev *i2c_dev, int reg, int mask, int target)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(i2c_dev->base + reg,
					status,
					(!!(status & mask) == target),
					20, 1000);
	if (ret) {
		dev_dbg(i2c_dev->dev, "wait timeout\n");
		ret = -EBUSY;
	}

	return ret;
}

// i2c_wait_EV_signal 返回非 0 时所在对应函数返回 I2C_XFER_TIMEOUT_ERROR
#define i2c_wait_EV_signal_t(dev, reg_offset, mask, target) if (i2c_wait_EV_signal(dev, reg_offset, mask, target)) { return -EBUSY;}

/*
 * @brief: i2c主模式，接收状态结束时的设置
 * @param: base: i2c控制器基地址
 * @param: stop: 是否发送关闭i2c总线信号
 */
static inline void i2c_setup_CR1_end_read(unsigned char *base, uint8_t stop)
{
	unsigned int temp;

	temp = readl((base + I2C_CR1));
	temp |= (I2C_CR1_ACK | I2C_CR1_POS);
	temp ^= (I2C_CR1_ACK | I2C_CR1_POS);
	temp |= (stop << 9);

	writel(temp, (base + I2C_CR1));
}

/*
 * @brief: i2c主模式接收状态，读取i2c DR 寄存器，放到msg的data里面
 * @param: base: i2c控制器基地址
 * @param: msg: i2c读取描述(i2c_msg)实例指针
 * @param: i: 当前读取数据所在data数组的下标
 */
static inline void i2c_update_read_msg_data(unsigned char *base, struct i2c_msg *msg, int i)
{
	msg->buf[i] = ((readl(base + I2C_DR)) & 0xff);
}

/*
 * @brief: i2c主模式接收状态，当只读取1个字节时的操作函数(从地址发送后的操作)
 * @param: i2c_dev: ls_i2c_dev
 * @param: msg: i2c读取描述(i2c_msg)实例指针
 * @return: 0代表操作成功，否则操作失败
 */
static int i2c_xfer_r_1(struct ls_i2c_dev *i2c_dev, struct i2c_msg *msg, int is_stop)
{
	unsigned char* base;
	base = i2c_dev->base;

	i2c_setup_CR1_end_read(base, is_stop);
	i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_RXNE, 1);
	i2c_update_read_msg_data(base, msg, 0);
	return 0;
}

/*
 * @brief: i2c主模式接收状态，当只读取2个字节时的操作函数(从地址发送后的操作)
 * @param: i2c_dev: ls_i2c_dev
 * @param: msg: i2c读取描述(i2c_msg)实例指针
 * @return: 0代表操作成功，否则操作失败
 */
static int i2c_xfer_r_2(struct ls_i2c_dev *i2c_dev, struct i2c_msg *msg, int is_stop)
{
	unsigned char* base;
	base = i2c_dev->base;

	i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_RXNE, 1);
	i2c_update_read_msg_data(base, msg, 0);
	i2c_setup_CR1_end_read(base, is_stop);
	i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_RXNE, 1);
	i2c_update_read_msg_data(base, msg, 1);
	return 0;
}

/*
 * @brief: i2c主模式接收状态，当读取大于等于3个字节时的操作函数(从地址发送后的操作)
 * @param: i2c_dev: ls_i2c_dev
 * @param: msg: i2c读取描述(i2c_msg)实例指针
 * @return: 0代表操作成功，否则操作失败
 */
static int i2c_xfer_r_3m(struct ls_i2c_dev *i2c_dev, struct i2c_msg *msg, int is_stop)
{
	unsigned char* base;
	int i;
	int remain_read;

	base = i2c_dev->base;

	i = 0;
	remain_read = msg->len;
	while (1)
	{
		if (remain_read <= 3)
		{
			break;
		}
		// 倒数3个字节之前，看 I2C_SR1_RXNE 信号即可，有就读取DR
		i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_RXNE, 1);
		i2c_update_read_msg_data(base, msg, i++);
		--remain_read;
	}
	// 见2k300手册 主接收模式（非及时、3 字节）示意图
	i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_BTF, 1); // 倒数3个字节，I2C_SR1_BTF信号为1时，代表已经接收了两个字节
	i2c_update_read_msg_data(base, msg, i++); // 读取DR，此时倒数第二个字节开始从移位寄存器到DR
	i2c_setup_CR1_end_read(base, is_stop); // 下一个字节（也就是最后一个字节）之后需要发送 NACK 和 stop 命令了
	i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_BTF, 1); // 这时已经最后一个字节到来
	i2c_update_read_msg_data(base, msg, i++); // 这次读取之后，等待移位寄存器填充DR
	i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_RXNE, 1); // 没有字节传输动作，但是移位寄存器已经把最后一个字节填充到DR
	i2c_update_read_msg_data(base, msg, i++); // 读取DR
	return 0;
}

/*
 * @brief: i2c主模式读取数据处理函数
 * @param: i2c_dev: ls_i2c_dev
 * @param: msg: i2c读取描述(i2c_msg)实例指针
 * @return: 0代表操作成功，否则操作失败
 */
static int i2c_xfer_r(struct ls_i2c_dev *i2c_dev, struct i2c_msg *msg, int is_stop)
{
	unsigned char* base;
	unsigned int temp;
	unsigned char xfer_data;
	int ret;

	base = i2c_dev->base;

	// START信号发送开始命令，ACK则会在接收到数据后发送ACK命令
	temp = readl((base + I2C_CR1));
	temp |= I2C_CR1_START;
	temp |= I2C_CR1_ACK;
	writel(temp, (base + I2C_CR1));

	i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_SB, 1);

	// 把7bit地址左移一位，然后最后1bit设置为1，代表此从地址用于读取
	xfer_data = msg->addr;
	xfer_data <<= 1;
	xfer_data |= 1; // bit 0 == 1 read
	writeb(xfer_data, (base + I2C_DR));

	i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_ADDR, 1);
	temp = readl((base + I2C_SR2));

	// 针对不同的需要读取的字节数，调用不同的处理函数
	if (msg->len == 1) // read 1 byte
	{
		ret = i2c_xfer_r_1(i2c_dev, msg, is_stop);
	}
	else if (msg->len == 2) // read 2 byte
	{
		ret = i2c_xfer_r_2(i2c_dev, msg, is_stop);
	}
	else if (msg->len > 3) // read 3 or more than 3 byte
	{
		ret = i2c_xfer_r_3m(i2c_dev, msg, is_stop);
	}

	if (ret)
		return ret;

	// 如果需要关闭总线，等待busy信号为0，代表总线总体已经空闲
	if (is_stop)
	{
		i2c_wait_EV_signal_t(i2c_dev, I2C_SR2, I2C_SR2_BUSY, 0);
	}

	return 0;
}

/*
 * @brief: i2c主模式写入数据处理函数
 * @param: i2c_dev: ls_i2c_dev
 * @param: msg: i2c写入描述(i2c_msg)实例指针
 * @return: 0代表操作成功，否则操作失败
 */
static int i2c_xfer_w(struct ls_i2c_dev *i2c_dev, struct i2c_msg *msg, int is_stop)
{
	unsigned char* base;
	unsigned int temp;
	int i;
	unsigned char xfer_data;

	base = i2c_dev->base;

	// START信号发送开始命令，ACK则会在接收到数据后发送ACK命令
	temp = readl((base + I2C_CR1));
	temp |= I2C_CR1_START;
	writel(temp, (base + I2C_CR1));

	i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_SB, 1);

	// 把7bit地址左移一位，然后最后1bit设置为1，代表此从地址用于读取
	xfer_data = msg->addr;
	xfer_data <<= 1; // bit 0 == 0 write
	writeb(xfer_data, (base + I2C_DR));

	i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_ADDR, 1);
	temp = readl((base + I2C_SR2));

	// 等待发送数据寄存器为空
	i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_TXE, 1);

	i = 0;
	while (1)
	{
		if (i == msg->len)
		{
			break;
		}

		xfer_data = msg->buf[i];
		++i;
		writeb(xfer_data, (base + I2C_DR));

		// 等待传送结束
		i2c_wait_EV_signal_t(i2c_dev, I2C_SR1, I2C_SR1_BTF, 1);
	}

	if (is_stop)
	{
		// 关闭总线
		temp = readl((base + I2C_CR1));
		temp |= (I2C_CR1_STOP);
		writel(temp, (base + I2C_CR1));
		// 等待busy信号为0，代表总线总体已经空闲
		i2c_wait_EV_signal_t(i2c_dev, I2C_SR2, I2C_SR2_BUSY, 0);
	}

	return 0;
}

/**
 * ls_i2c_xfer() - Transfer combined I2C message
 * @i2c_adap: Adapter pointer to the controller
 * @msgs: Pointer to data to be written.
 * @num: Number of messages to be executed
 */
static int ls_i2c_xfer(struct i2c_adapter *i2c_adap, struct i2c_msg msgs[], int num)
{
	struct ls_i2c_dev *i2c_dev = i2c_get_adapdata(i2c_adap);
	int ret = 0;
	struct i2c_msg* cur_msg;
	int i;
	int stop;

	ret = ls_i2c_wait_free_bus(i2c_dev);
	if (ret)
		return ret;

	for (i = 0; i < num; i++) {
		cur_msg = msgs + i;
		stop = (i == (num - 1));
		if (cur_msg->flags & I2C_M_RD)
		{
			ret += i2c_xfer_r(i2c_dev, cur_msg, stop);
		}
		else
		{
			ret += i2c_xfer_w(i2c_dev, cur_msg, stop);
		}
		if (ret)
			dev_err(i2c_dev->dev, "i2c %s error(%d)\r\n", (cur_msg->flags & I2C_M_RD) ? "read" : "write", ret);
	}

	return (ret < 0) ? ret : num;
}

static u32 ls_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm ls_i2c_algo = {
	.master_xfer = ls_i2c_xfer,
	.functionality = ls_i2c_func,
};

static void ls_i2c_bus_clock_probe(struct ls_i2c_dev *i2c_dev)
{
	int ret;
	struct clk* clk;
	int of_clk_freq;

	i2c_dev->freq = INPUT_DEFAULT_CLK;
	ret = 1;
	clk = devm_clk_get_optional_enabled(i2c_dev->dev, NULL);
	if (clk) {
		ret = devm_clk_rate_exclusive_get(i2c_dev->dev, clk);
		if (!ret)
			i2c_dev->freq = clk_get_rate(clk);
	} else {
		if (!of_property_read_u32(i2c_dev->dev->of_node, "clock-frequency", &of_clk_freq)) {
			ret = 0;
			i2c_dev->freq = of_clk_freq;
		}
	}

	if (ret)
		pr_info("%s warning: use default clk freq %d\n", __func__, INPUT_DEFAULT_CLK);
}

static int ls_i2c_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct ls_i2c_dev *i2c_dev;
	struct resource *res;
	u32 clk_rate;
	struct i2c_adapter *adap;
	int ret;
	int name_buffer_len;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;

	i2c_dev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(i2c_dev->base)) {
		ret = PTR_ERR(i2c_dev->base);
		goto free_i2c_mem;
	}

	i2c_dev->speed = LS_I2C_SPEED_STANDARD;
	ret = of_property_read_u32(np, "i2c-speed", &clk_rate);
	if (!ret && clk_rate >= 400000)
		i2c_dev->speed = LS_I2C_SPEED_FAST;

	init_completion(&i2c_dev->complete);

	i2c_dev->dev = &pdev->dev;

	ls_i2c_bus_clock_probe(i2c_dev);
	ls_i2c_hw_config(i2c_dev);
	ls_i2c_disable_irq(i2c_dev); // 不需要中断 轮询等待

	adap = &i2c_dev->adap;
	i2c_set_adapdata(adap, i2c_dev);
	adap->nr = pdev->id;
	name_buffer_len = strlen(pdev->name);
	name_buffer_len = (name_buffer_len >= sizeof(adap->name)) ? sizeof(adap->name) - 1 : name_buffer_len;
	memset(adap->name, 0, name_buffer_len + 1);
	strncpy(adap->name, pdev->name, name_buffer_len);
	adap->name[name_buffer_len] = 0;
	adap->owner = THIS_MODULE;
	adap->retries = 5;
	adap->algo = &ls_i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;
	adap->timeout = 2 * HZ;

	ret = i2c_add_adapter(adap);
	if (ret) {
		dev_err(&pdev->dev, "failure adding adapter\n");
		goto free_i2c_ioremap;
	}

	platform_set_drvdata(pdev, i2c_dev);

	return 0;

free_i2c_ioremap:
	devm_iounmap(&pdev->dev, i2c_dev->base);
free_i2c_mem:
	kfree(i2c_dev);

	return ret;
}

static void ls_i2c_remove(struct platform_device *pdev)
{
	struct ls_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&i2c_dev->adap);
	iounmap(i2c_dev->base);
	kfree(i2c_dev);
}

static const struct of_device_id ls_i2c_match[] = {
	{.compatible = "loongson,lsfs-i2c"},
	{},
};
MODULE_DEVICE_TABLE(of, ls_i2c_match);

static struct platform_driver ls_i2c_driver = {
	.driver = {
		.name = "lsfs-i2c",
		.of_match_table = ls_i2c_match,
	},
	.probe = ls_i2c_probe,
	.remove = ls_i2c_remove,
};

module_platform_driver(ls_i2c_driver);

MODULE_AUTHOR("Loongson Technology Corporation Limited");
MODULE_DESCRIPTION("Loongson fast speed I2C bus adapter");
MODULE_LICENSE("GPL");
