/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Notifier registered with Apple type-C PHY to query / update connection
 * and cable state.
 */

#ifndef _LINUX_APPLE_ATC_PHY_H_
#define _LINUX_APPLE_ATC_PHY_H_

#include <linux/types.h>
#include <linux/usb/typec_altmode.h>

struct apple_atc_phy_conn {
	void *client, *phy;
	unsigned long mode;
	enum typec_orientation orientation;
	struct typec_usb4_cable cable;
	void (*notify)(struct apple_atc_phy_conn *);
};

int apple_atc_phy_conn_register(struct device *dev, struct apple_atc_phy_conn *conn);
void apple_atc_phy_conn_deregister(struct apple_atc_phy_conn *conn);

#endif
