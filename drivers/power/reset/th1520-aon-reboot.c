// SPDX-License-Identifier: GPL-2.0
/*
 * T-HEAD TH1520 AON Firmware Reboot Driver
 *
 * Copyright (c) 2025 Icenowy Zheng <uwu@icenowy.me>
 */

#include <linux/auxiliary_bus.h>
#include <linux/delay.h>
#include <linux/firmware/thead/thead,th1520-aon.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include <linux/reboot.h>
#include <linux/slab.h>

#define TH1520_AON_REBOOT_PRIORITY 200
/* Allow an accepted asynchronous command to take effect before fallback. */
#define TH1520_AON_REBOOT_TIMEOUT_MS 1000

struct th1520_aon_msg_empty_body {
	struct th1520_aon_rpc_msg_hdr hdr;
	u16 reserved[12];
} __packed __aligned(1);

static int th1520_aon_reboot_request(struct sys_off_data *data, u8 func)
{
	struct th1520_aon_chan *aon_chan = data->cb_data;
	struct th1520_aon_msg_empty_body msg = {};
	int ret;

	msg.hdr.svc = TH1520_AON_RPC_SVC_WDG;
	msg.hdr.func = func;
	msg.hdr.size = TH1520_AON_RPC_MSG_NUM;

	ret = th1520_aon_call_rpc(aon_chan, &msg);
	if (ret)
		dev_err(data->dev, "AON WDG command %u failed: %d\n", func, ret);
	else
		msleep(TH1520_AON_REBOOT_TIMEOUT_MS);

	return NOTIFY_DONE;
}

static int th1520_aon_pwroff_handler(struct sys_off_data *data)
{
	return th1520_aon_reboot_request(data, TH1520_AON_WDG_FUNC_POWER_OFF);
}

static int th1520_aon_restart_handler(struct sys_off_data *data)
{
	return th1520_aon_reboot_request(data, TH1520_AON_WDG_FUNC_RESTART);
}

static int th1520_aon_pwroff_failed(struct sys_off_data *data)
{
	dev_err(data->dev, "AON did not power off the system\n");
	return NOTIFY_DONE;
}

static int th1520_aon_reboot_probe(struct auxiliary_device *adev,
				  const struct auxiliary_device_id *id)
{
	struct device *dev = &adev->dev;
	int ret;

	/*
	 * RPC takes a mutex and waits for mailbox interrupts. Send after device
	 * shutdown, but before syscore shutdown and the final atomic callbacks.
	 */
	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF_PREPARE,
					    TH1520_AON_REBOOT_PRIORITY,
					    th1520_aon_pwroff_handler,
					    adev->dev.platform_data);

	if (ret) {
		dev_err(dev, "Failed to register power off handler\n");
		return ret;
	}

	ret = devm_register_sys_off_handler(dev, SYS_OFF_MODE_RESTART_PREPARE,
					    TH1520_AON_REBOOT_PRIORITY,
					    th1520_aon_restart_handler,
					    adev->dev.platform_data);

	if (ret) {
		dev_err(dev, "Failed to register restart handler\n");
		return ret;
	}

	/*
	 * Advertise power-off capability to kernel_can_power_off(). This final
	 * callback must not send RPCs; let other providers try if AON failed.
	 */
	return devm_register_sys_off_handler(dev, SYS_OFF_MODE_POWER_OFF,
					     TH1520_AON_REBOOT_PRIORITY,
					     th1520_aon_pwroff_failed, dev_get_platdata(dev));
}

static const struct auxiliary_device_id th1520_aon_reboot_id_table[] = {
	{ .name = "th1520_pm_domains.reboot" },
	{},
};
MODULE_DEVICE_TABLE(auxiliary, th1520_aon_reboot_id_table);

static struct auxiliary_driver th1520_aon_reboot_driver = {
	.driver = {
		.name = "th1520-aon-reboot",
	},
	.probe = th1520_aon_reboot_probe,
	.id_table = th1520_aon_reboot_id_table,
};
module_auxiliary_driver(th1520_aon_reboot_driver);

MODULE_AUTHOR("Icenowy Zheng <uwu@icenowy.me>");
MODULE_DESCRIPTION("T-HEAD TH1520 AON-firmware-based reboot driver");
MODULE_LICENSE("GPL");
