#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/delay.h>
#include "loongson-2k300-adc-core.h"

#define ADC_DATA_MASK 0x0FFF

#define ADC_MAX_VOLTAGE 1800
#define ADC_SCALE 4096

struct ls2k300_adc_data {
	void __iomem *base;
	spinlock_t read_raw_lock; // 添加读取的自旋锁 扫描模式，单次读取时使用
};

// 这里是有节点可以读取的关键
#define LS2K300_CHANNEL(num) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = num, \
	.address = ADC_DR, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.scan_index = num, \
	.scan_type = { \
		.sign = 's', \
		.realbits = 12, \
		.storagebits = 16, \
		.shift = 12, \
		.endianness = IIO_CPU, \
	}, \
}

static const struct iio_chan_spec ls2k300_adc_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(8),
	LS2K300_CHANNEL(0),
	LS2K300_CHANNEL(1),
	LS2K300_CHANNEL(2),
	LS2K300_CHANNEL(3),
	LS2K300_CHANNEL(4),
	LS2K300_CHANNEL(5),
	LS2K300_CHANNEL(6),
	LS2K300_CHANNEL(7),
	// 注入通道
	// { .type = IIO_VOLTAGE, .differential = 1, .channel = 0, .address = ADC_JDR1 },
	// { .type = IIO_VOLTAGE, .differential = 1, .channel = 1, .address = ADC_JDR2 },
	// { .type = IIO_VOLTAGE, .differential = 1, .channel = 2, .address = ADC_JDR3 },
	// { .type = IIO_VOLTAGE, .differential = 1, .channel = 3, .address = ADC_JDR4 },
};

// sysfs 下的节点读取时的具体调用函数
static int ls2k300_adc_read_raw(struct iio_dev *indio_dev,
						struct iio_chan_spec const *chan,
						int *val, int *val2, long mask)
{
	struct ls2k300_adc_data *data = iio_priv(indio_dev);
	unsigned int reg_offset;
	unsigned long flags;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		// 针对 in_voltagex_raw 节点
		if (chan->differential) {
			reg_offset = chan->address;
		} else {
			reg_offset = chan->address;
		}

		// 由于规则通道组只有一个通道，所以这里需要用锁来限定一段时间只能读取一个通道的值。
		spin_lock_irqsave(&data->read_raw_lock, flags);

		ADC_RegularChannelConfig(data->base, chan->channel, 1, ADC_SampleTime_64Cycles);
		adc_software_start_conv_trigger(data->base, ENABLE, 0);
		adc_eoc_check_conv_end(data->base);
		*val = ioread16(data->base + reg_offset) & ADC_DATA_MASK;

		spin_unlock_irqrestore(&data->read_raw_lock, flags);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		// 针对 in_voltagex_scale 节点
		*val = ADC_MAX_VOLTAGE;
		*val2 = ADC_SCALE;
		// 返回的值就是 val1 / val2 的那个小数值
		// scale * raw 就是对应的电压值，单位mV
		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ls2k300_adc_info = {
	.read_raw = ls2k300_adc_read_raw,
};

static int ls2k300_adc_probe(struct platform_device *pdev)
{
	adc_init_info adc_init_struct;
	struct iio_dev *indio_dev;
	struct ls2k300_adc_data *data;
	struct resource *res;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*data));
	if (!indio_dev) {
		dev_err(&pdev->dev, "devm_iio_device_alloc failed! probe failed!\n");
		return -ENOMEM;
	}

	data = iio_priv(indio_dev);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->base)) {
		dev_err(&pdev->dev, "devm_ioremap_resource failed! probe failed! (base: %.llx)\n", (unsigned long long)data->base);
		return PTR_ERR(data->base);
	}

	// 配置 ADC CR1 和 CR2 等寄存器，具体配置根据需要调整
	// 不使用 DMA, 扫描模式, 单次，规则序列转化数为1, 也就是每次都等待读取完毕。
	// EOC中断和JEOC中断没开
	adc_struct_init(&adc_init_struct);
	adc_init_struct.ADC_Mode = ADC_Mode_Independent;
	adc_init_struct.ADC_ScanConvMode = ENABLE;
	adc_init_struct.ADC_ContinuousConvMode = DISABLE;
	adc_init_struct.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	adc_init_struct.ADC_DataAlign = ADC_DataAlign_Right;
	adc_init_struct.ADC_NbrOfChannel = 1; //规则通道数量 为 1，这是一个一个的触发 扫描模式的单次扫描
	adc_init_struct.ADC_ClkDivider = 1;  //Loongson Feature
	adc_init_struct.ADC_JTrigMod = 0;    //Loongson Feature
	adc_init_struct.ADC_ADCEdge = 0;     //Loongson Feature
	adc_init_struct.ADC_DiffMod = 0;     //Loongson Feature
	adc_init(data->base, &adc_init_struct);

	adc_dev_enable(data->base, ENABLE);

	// iio 框架信息注册
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->info = &ls2k300_adc_info;
	indio_dev->name = pdev->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ls2k300_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(ls2k300_adc_channels);

	spin_lock_init(&data->read_raw_lock); // 初始化自旋锁

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "iio_device_register failed! probe failed! (ret: %d)\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, indio_dev);
	dev_info(&pdev->dev, "ADC Device registered successfully\n");
	return 0;
}

static void ls2k300_adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	iio_device_unregister(indio_dev);
}

static const struct of_device_id ls2k300_adc_of_match[] = {
	{ .compatible = "ls2k300-adc", },
	{ }
};
MODULE_DEVICE_TABLE(of, ls2k300_adc_of_match);

static struct platform_driver ls2k300_adc_driver = {
	.driver = {
		.name = "ls2k300_adc",
		.of_match_table = ls2k300_adc_of_match,
	},
	.probe = ls2k300_adc_probe,
	.remove = ls2k300_adc_remove,
};
module_platform_driver(ls2k300_adc_driver);

MODULE_AUTHOR("oujintao qujintao@loongson.cn");
MODULE_DESCRIPTION("2k300 ADC IIO driver");
MODULE_LICENSE("GPL");
