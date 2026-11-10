// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 by Aurelien Jarno
 */

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>

/* Power Control Register 2 */
#define PWR_CTRL2		0x7e
#define PWR_CTRL2_SHUTDOWN	BIT(2)	/* Shutdown request */
#define PWR_CTRL2_RST		BIT(1)	/* Reset request */

static int spacemit_p1_read_atomic(struct i2c_client *client,
				   u8 reg, u8 *val)
{
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = client->flags & I2C_M_TEN,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = client->addr,
			.flags = (client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = 1,
			.buf = val,
		},
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

	return ret == ARRAY_SIZE(msgs) ? 0 : ret < 0 ? ret : -EIO;
}

static int spacemit_p1_write_atomic(struct i2c_client *client,
				    u8 reg, u8 val)
{
	u8 buf[] = { reg, val };
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = client->flags & I2C_M_TEN,
		.len = sizeof(buf),
		.buf = buf,
	};
	int ret;

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret == 1 ? 0 : ret < 0 ? ret : -EIO;
}

static int spacemit_p1_update_bits_atomic(struct i2c_client *client,
					  u8 reg, u8 mask)
{
	u8 val;
	int ret;

	ret = spacemit_p1_read_atomic(client, reg, &val);
	if (ret)
		return ret;

	return spacemit_p1_write_atomic(client, reg, val | mask);
}

static int spacemit_p1_pwroff_handler(struct sys_off_data *data)
{
	struct i2c_client *client = data->cb_data;
	int ret;

	/* Put the PMIC into shutdown state */
	ret = spacemit_p1_update_bits_atomic(client, PWR_CTRL2,
					     PWR_CTRL2_SHUTDOWN);
	if (ret) {
		dev_err(data->dev, "shutdown failed: %d\n", ret);
		return notifier_from_errno(ret);
	}

	return NOTIFY_DONE;
}

static int spacemit_p1_restart_handler(struct sys_off_data *data)
{
	struct i2c_client *client = data->cb_data;
	int ret;

	/* Put the PMIC into reset state */
	ret = spacemit_p1_update_bits_atomic(client, PWR_CTRL2,
					     PWR_CTRL2_RST);
	if (ret) {
		dev_err(data->dev, "restart failed: %d\n", ret);
		return notifier_from_errno(ret);
	}

	return NOTIFY_DONE;
}

static int spacemit_p1_reboot_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct i2c_client *client;
	int ret;

	client = i2c_verify_client(dev->parent);
	if (!client)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to find parent I2C client\n");

	if (!client->adapter->algo->xfer_atomic)
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "Atomic I2C transfers not supported\n");

	ret = devm_register_power_off_handler(dev, spacemit_p1_pwroff_handler,
					      client);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to register power off handler\n");

	ret = devm_register_restart_handler(dev, spacemit_p1_restart_handler,
					    client);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to register restart handler\n");

	return 0;
}

static const struct platform_device_id spacemit_p1_reboot_id_table[] = {
	{ .name = "spacemit-p1-reboot" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, spacemit_p1_reboot_id_table);

static struct platform_driver spacemit_p1_reboot_driver = {
	.driver = {
		.name = "spacemit-p1-reboot",
	},
	.probe = spacemit_p1_reboot_probe,
	.id_table = spacemit_p1_reboot_id_table,
};
module_platform_driver(spacemit_p1_reboot_driver);

MODULE_DESCRIPTION("SpacemiT P1 reboot/poweroff driver");
MODULE_LICENSE("GPL");
