// SPDX-License-Identifier: GPL-2.0
/*
 * Apple M1 SoC type-C PHY driver
 *
 * Copyright (c) 2021 Corellium LLC
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/pd.h>
#include <linux/usb/role.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_tbt.h>
#include <linux/usb/typec_altmode.h>

#define MAX_REG		8

struct apple_atcphy_m1 {
	struct device *dev;
	void __iomem *reg[MAX_REG];
	int num_reg;
	struct clk_bulk_data *clks;
	int num_clks;

	struct phy *phy;
	struct phy_provider *phy_provider;

	struct clk_hw clkhw;

	struct typec_mux *typec_mux;
	struct typec_switch *typec_sw;
	struct usb_role_switch *role_sw;

	enum typec_orientation orientation;
	enum usb_role role;

	unsigned long mode;
	unsigned usb2en;
};

#define REG_FORCE_CLK_ON				0x00000F0
#define   REG_FORCE_CLK_ON_PIPEHDLR			(1 << 4)
#define   REG_FORCE_CLK_ON_DRD				(1 << 3)
#define   REG_FORCE_CLK_ON_DBGWRAP			(1 << 2)
#define   REG_FORCE_CLK_ON_FABRIC			(1 << 1)
#define   REG_FORCE_CLK_ON_USB2PHY			(1 << 0)
#define REG_DRDRAM_STATUS				0x0200000
#define   REG_DRDRAM_STATUS_RAM0_WR_ACTIVE		(1 << 15)
#define REG_PIPEPHY_STATUS				0x0200020
#define   REG_PIPEPHY_STATUS_READY			(1 << 30)
#define REG_DWC3					0x0280000
#define REG_CIO_LFPSOFFSET				0x028DC38
#define   REG_CIO_LFPSOFFSET_MAC_MASK			(0xffff << 0)
#define     REG_CIO_LFPSOFFSET_MAC_DFLT			(0x0f80 << 0)
#define   REG_CIO_LFPSOFFSET_AP_MASK			(31 << 23)
#define REG_CIO_BWNGTOFFSET				0x028DC3C
#define   REG_CIO_BWNGTOFFSET_MAC_MASK			(0xffff << 0)
#define     REG_CIO_BWNGTOFFSET_MAC_DFLT		(0x0fc0 << 0)
#define   REG_CIO_BWNGTOFFSET_AP_MASK			(63 << 22)
#define REG_CIO_LINKTIMER1				0x028CD40
#define   REG_CIO_LINKTIMER1_PEND_HP_MASK		(255 << 16)
#define     REG_CIO_LINKTIMER1_PEND_HP_DFLT		(20 << 16)
#define   REG_CIO_LINKTIMER1_PM_LC_MASK			(255 << 8)
#define     REG_CIO_LINKTIMER1_PM_LC_DFLT		(10 << 8)
#define   REG_CIO_LINKTIMER1_PM_ENTRY_MASK		(255 << 0)
#define     REG_CIO_LINKTIMER1_PM_ENTRY_DFLT		(16 << 0)
#define REG_AUSBEVT_USB2CTL				0x0800000
#define   REG_AUSBEVT_USB2CTL_EVT_EN			(1 << 0)
#define   REG_AUSBEVT_USB2CTL_LOAD_CNT			(1 << 3)
#define REG_AUSBEVT_UTMIACT_EVTCNT			0x0800020
#define REG_PIPEHDLR_MUXSEL				0x0A8400C
#define   REG_PIPEHDLR_MUXSEL_MODE_MASK			(3 << 0)
#define     REG_PIPEHDLR_MUXSEL_MODE_USB30		(0 << 0)
#define     REG_PIPEHDLR_MUXSEL_MODE_USB31		(1 << 0)
#define     REG_PIPEHDLR_MUXSEL_MODE_USB2		(2 << 0)
#define   REG_PIPEHDLR_MUXSEL_CLKEN_MASK		(7 << 3)
#define     REG_PIPEHDLR_MUXSEL_CLKEN_USB30		(1 << 3)
#define     REG_PIPEHDLR_MUXSEL_CLKEN_USB31		(2 << 3)
#define     REG_PIPEHDLR_MUXSEL_CLKEN_TBT		(4 << 3)
#define REG_PIPEHDLR_PIPE_IF_REQ			0x0A84010
#define REG_PIPEHDLR_PIPE_IF_ACK			0x0A84014
#define REG_PIPEHDLR_AON_GEN				0x0A8401C
#define   REG_PIPEHDLR_AON_GEN_DRD_FORCE_CLAMP_EN	(1 << 4)
#define   REG_PIPEHDLR_AON_GEN_DRD_SW_VCC_RESET		(1 << 0)
#define REG_PIPEHDLR_OVRD				0x0A84000
#define   REG_PIPEHDLR_OVRD_RXVALID			(1 << 0)
#define   REG_PIPEHDLR_OVRD_RXDET			(1 << 2)
#define REG_PIPEHDLR_OVRD_VAL0				0x0A84004
#define   REG_PIPEHDLR_OVRD_VAL0_RXDET			(7 << 1)
#define REG_PIPEHDLR_NONSEL_OVRD			0x0A84020
#define   REG_PIPEHDLR_NONSEL_OVRD_STATE_MASK		(15 << 0)
#define     REG_PIPEHDLR_NONSEL_OVRD_STATE_DOWN		(2 << 0)
#define     REG_PIPEHDLR_NONSEL_OVRD_STATE_OFF		(3 << 0)
#define   REG_PIPEHDLR_NONSEL_OVRD_RESET		(1 << 12)
#define   REG_PIPEHDLR_NONSEL_OVRD_DUMMY_PHY_READY	(1 << 15)
#define REG_USB2PHY_USBCTL				0x0A90000
#define   REG_USB2PHY_USBCTL_MODE_MASK			(7 << 0)
#define     REG_USB2PHY_USBCTL_MODE_DEV			(0 << 0)
#define     REG_USB2PHY_USBCTL_MODE_HOST		(2 << 0)
#define     REG_USB2PHY_USBCTL_MODE_ISOLATION		(4 << 0)
#define REG_USB2PHY_CTL					0x0A90004
#define   REG_USB2PHY_CTL_RESET				(1 << 0)
#define   REG_USB2PHY_CTL_PORT_RESET			(1 << 1)
#define   REG_USB2PHY_CTL_APB_RESETN			(1 << 2)
#define   REG_USB2PHY_CTL_SIDDQ				(1 << 3)
#define REG_USB2PHY_SIG					0x0A90008
#define   REG_USB2PHY_SIG_VBUSDET_FORCE_VAL		(1 << 0)
#define   REG_USB2PHY_SIG_VBUSDET_FORCE_EN		(1 << 1)
#define   REG_USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL		(1 << 2)
#define   REG_USB2PHY_SIG_VBUSVLDEXT_FORCE_EN		(1 << 3)
#define   REG_USB2PHY_SIG_MODE_HOST			(7 << 12)
#define REG_USB2PHY_MISCTUNE				0x0A9001C
#define   REG_USB2PHY_MISCTUNE_APBCLK_GATE_OFF		(1 << 29)
#define   REG_USB2PHY_MISCTUNE_REFCLK_GATE_OFF		(1 << 30)
#define REG_ACIOPHY_CFG0				0x1000008
#define   REG_ACIOPHY_CFG0_CMN_BIG			(1 << 0)
#define   REG_ACIOPHY_CFG0_CMN_BIG_OV			(1 << 1)
#define   REG_ACIOPHY_CFG0_CMN_SML			(1 << 2)
#define   REG_ACIOPHY_CFG0_CMN_SML_OV			(1 << 3)
#define   REG_ACIOPHY_CFG0_CMN_CLAMP			(1 << 4)
#define   REG_ACIOPHY_CFG0_CMN_CLAMP_OV			(1 << 5)
#define   REG_ACIOPHY_CFG0_RX_BIG			(3 << 6)
#define   REG_ACIOPHY_CFG0_RX_BIG_OV			(3 << 8)
#define   REG_ACIOPHY_CFG0_RX_SML			(3 << 10)
#define   REG_ACIOPHY_CFG0_RX_SML_OV			(3 << 12)
#define   REG_ACIOPHY_CFG0_RX_CLAMP			(3 << 14)
#define   REG_ACIOPHY_CFG0_RX_CLAMP_OV			(3 << 16)
#define REG_ACIOPHY_MODE				0x1000048
#define     REG_ACIOPHY_MODE_VAL_TBT			0
#define     REG_ACIOPHY_MODE_VAL_USB			1
#define     REG_ACIOPHY_MODE_VAL_DP			2
#define     REG_ACIOPHY_MODE_VAL_OFF			3
#define   REG_ACIOPHY_MODE_RX0_MASK			(7 << 0)
#define     REG_ACIOPHY_MODE_RX0_SHIFT			0
#define   REG_ACIOPHY_MODE_TX0_MASK			(7 << 3)
#define     REG_ACIOPHY_MODE_TX0_SHIFT			3
#define   REG_ACIOPHY_MODE_RX1_MASK			(7 << 6)
#define     REG_ACIOPHY_MODE_RX1_SHIFT			6
#define   REG_ACIOPHY_MODE_TX1_MASK			(7 << 9)
#define     REG_ACIOPHY_MODE_TX1_SHIFT			9
#define REG_ACIOPHY_XBAR				0x100004C
#define   REG_ACIOPHY_XBAR_PROTO_MASK			(0x1F << 0)
#define     REG_ACIOPHY_XBAR_PROTO_TBT01		(0x00 << 0)
#define     REG_ACIOPHY_XBAR_PROTO_TBT10		(0x01 << 0)
#define     REG_ACIOPHY_XBAR_PROTO_USB0			(0x0A << 0)
#define     REG_ACIOPHY_XBAR_PROTO_USB1			(0x0B << 0)
#define     REG_ACIOPHY_XBAR_PROTO_USB0_DP1		(0x10 << 0)
#define     REG_ACIOPHY_XBAR_PROTO_DP0_USB1		(0x11 << 0)
#define     REG_ACIOPHY_XBAR_PROTO_DP01			(0x14 << 0)
#define   REG_ACIOPHY_XBAR_DPMODE_MASK			(0x1FFF << 5)
#define     REG_ACIOPHY_XBAR_DPMODE_DP01		(0x1100 << 5)
#define     REG_ACIOPHY_XBAR_DPMODE_DPX			(0x0008 << 5)
#define REG_ACIOPHY_BIST_EN				0x1000084
#define   REG_ACIOPHY_BIST_EN_CLK			(1 << 27)
#define   REG_ACIOPHY_BIST_EN_CIOBIST			(1 << 28)
#define REG_ACIOPHY_BIST_OV				0x100008C
#define   REG_ACIOPHY_BIST_OV_LN0_RESETN		(1 << 13)
#define   REG_ACIOPHY_BIST_OV_LN0_PWRDN			(1 << 25)
#define REG_ACIOPHY_BIST_CFG0				0x1000090
#define   REG_ACIOPHY_BIST_CFG0_SET			(1 << 2)
#define REG_ACIOPHY_BIST_STAT				0x100009C
#define   REG_ACIOPHY_BIST_STAT_LN0_READY		(1 << 0)
#define   REG_ACIOPHY_BIST_STAT_LN0_BUSY		(1 << 23)
#define REG_ACIOPHY_BIST_RST				0x10000A8
#define   REG_ACIOPHY_BIST_RST_RESETN			(1 << 0)
#define REG_ACIOPHY_BIST_CFG1				0x10000AC
#define   REG_ACIOPHY_BIST_CFG1_LN0_PWRDN_MASK		(15 << 10)
#define     REG_ACIOPHY_BIST_CFG1_LN0_PWRDN_OFF		(3 << 10)
#define REG_ACIOPHY_SLEEPCTRL				0x10001B0
#define   REG_ACIOPHY_SLEEPCTRL_TX_BIG			(3 << 0)
#define   REG_ACIOPHY_SLEEPCTRL_TX_BIG_OV		(3 << 2)
#define   REG_ACIOPHY_SLEEPCTRL_TX_SML			(3 << 4)
#define   REG_ACIOPHY_SLEEPCTRL_TX_SML_OV		(3 << 6)
#define   REG_ACIOPHY_SLEEPCTRL_TX_CLAMP		(3 << 8)
#define   REG_ACIOPHY_SLEEPCTRL_TX_CLAMP_OV		(3 << 10)
#define REG_AUSPLL_FSM_CTRL				0x1001014
#define   REG_AUSPLL_FSM_CTRL_APBREQ_OVSEL		(255 << 13)
#define REG_AUSPLL_CMD_OVRD				0x1002000
#define   REG_AUSPLL_CMD_OVRD_APB			(1 << 28)
#define REG_CIO3PLL_CLKOUT				0x1002A00
#define   REG_CIO3PLL_CLKOUT_PCLK_EN			(1 << 1)
#define   REG_CIO3PLL_CLKOUT_REFCLK_EN			(1 << 5)
#define REG_AUSPMA_RX0_FSM_CTRL				0x1009010
#define   REG_AUSPMA_RX_FSM_CTRL_OVRD			(1 << 0)
#define   REG_AUSPMA_RX_FSM_CTRL_REQ			(1 << 9)
#define REG_AUSPMA_RX1_FSM_CTRL				0x1010010
#define REG_ATCPHY_PWRDN_CTRL				0x1020000
#define   REG_ATCPHY_PWRDN_CTRL_SLEEP_SML		(1 << 0)
#define   REG_ATCPHY_PWRDN_CTRL_SLEEP_BIG		(1 << 1)
#define   REG_ATCPHY_PWRDN_CTRL_CLAMP			(1 << 2)
#define   REG_ATCPHY_PWRDN_CTRL_APB_RESETN		(1 << 3)
#define   REG_ATCPHY_PWRDN_CTRL_PHY_RESETN		(1 << 4)
#define REG_ATCPHY_PWRDN_STAT				0x1020004
#define   REG_ATCPHY_PWRDN_STAT_SLEEP_SML		(1 << 0)
#define   REG_ATCPHY_PWRDN_STAT_SLEEP_BIG		(1 << 1)
#define REG_ATCPHY_MISC					0x1020008
#define   REG_ATCPHY_MISC_AON_RESETN			(1 << 0)
#define   REG_ATCPHY_MISC_USB_LANE_SWAP			(1 << 2)

static inline void apple_atcphy_m1_rmwl(u32 clr, u32 set, void __iomem *io)
{
	writel((readl(io) & ~clr) | set, io);
}
#define reg_rmwl(clr,set,offs) apple_atcphy_m1_rmwl(clr, set, atc->reg[1] + (offs))

static int apple_atcphy_m1_poll(u32 msk, u32 val, int neq, void __iomem *io,
				struct apple_atcphy_m1 *atc, const char *name)
{
	int ret;
	u32 rdv;

	ret = readl_poll_timeout(io, rdv, ((rdv & msk) == val) ^ neq, 100, 100000);
	if(ret)
		pr_err("%pOFn: %s: [%s] error %d (0x%x).\n",
		       atc->dev->of_node, __func__, name, ret, rdv);
	return ret;
}
#define reg_poll(msk,val,neq,offs,name) \
	apple_atcphy_m1_poll(msk, val, neq, atc->reg[1] + (offs), atc, name)

/* this plays back tunables passed from bootloader. those are somewhat hardware specific,
   and ultimately provided by the first-stage platform bootloader */
static int apple_atcphy_m1_tunable(struct apple_atcphy_m1 *atc, const char *name)
{
	struct device_node *np = atc->dev->of_node;
	int cnt, idx;
	u32 addr, mask, val, range;

	cnt = of_property_count_elems_of_size(np, name, sizeof(u32));
	if(cnt < 0) {
		pr_warn("%pOFn: %s: missing tunable [%s].\n", np, __func__, name);
		return 0;
	}
	if(cnt % 3) {
		pr_warn("%pOFn: %s: tunable [%s] is not made of <addr, mask, val> triplets.\n",
			np, __func__, name);
		return -EINVAL;
	}

	for(idx=0; idx<cnt; idx+=3) {
		if(of_property_read_u32_index(np, name, idx, &addr) < 0 ||
		   of_property_read_u32_index(np, name, idx + 1, &mask) < 0 ||
		   of_property_read_u32_index(np, name, idx + 2, &val)) {
			pr_warn("%pOFn: %s: tunable [%s] not readable.\n", np, __func__, name);
			return -EINVAL;
		}
		range = addr >> 28;
		addr &= 0x0FFFFFFF;

		if(range < atc->num_reg)
			apple_atcphy_m1_rmwl(mask, val, atc->reg[range] + addr);
		else
			pr_warn("%pOFn: %s: tunable [%s] refers to nonexistent range %d.\n",
				np, __func__, name, range);
	}

	return 0;
}

static int apple_atcphy_m1_usb2_enable(struct apple_atcphy_m1 *atc, int host)
{
	int ret;

	if(atc->usb2en)
		return 0;

	ret = apple_atcphy_m1_tunable(atc, "tunable-ATC0AXI2AF");
	if(ret < 0)
		return ret;

	if(host)
		reg_rmwl(0, REG_USB2PHY_SIG_MODE_HOST, REG_USB2PHY_SIG);
	else
		reg_rmwl(REG_USB2PHY_SIG_MODE_HOST, 0, REG_USB2PHY_SIG);

	/* configure VBUS detection */

	reg_rmwl(0, REG_USB2PHY_SIG_VBUSDET_FORCE_VAL, REG_USB2PHY_SIG);
	reg_rmwl(0, REG_USB2PHY_SIG_VBUSDET_FORCE_EN, REG_USB2PHY_SIG);
	reg_rmwl(0, REG_USB2PHY_SIG_VBUSVLDEXT_FORCE_VAL, REG_USB2PHY_SIG);
	reg_rmwl(0, REG_USB2PHY_SIG_VBUSVLDEXT_FORCE_EN, REG_USB2PHY_SIG);

	/* power on the PHY and take it out of reset */

	reg_rmwl(REG_USB2PHY_CTL_SIDDQ, 0, REG_USB2PHY_CTL);
	udelay(10);
	reg_rmwl(REG_USB2PHY_CTL_RESET, 0, REG_USB2PHY_CTL);
	reg_rmwl(REG_USB2PHY_CTL_PORT_RESET, 0, REG_USB2PHY_CTL);

	reg_rmwl(0, REG_AUSBEVT_USB2CTL_LOAD_CNT, REG_AUSBEVT_USB2CTL);
	reg_rmwl(0, REG_AUSBEVT_USB2CTL_EVT_EN, REG_AUSBEVT_USB2CTL);

	reg_rmwl(0, REG_USB2PHY_CTL_APB_RESETN, REG_USB2PHY_CTL);

	/* enable PHY clocks */

	reg_rmwl(REG_USB2PHY_MISCTUNE_APBCLK_GATE_OFF, 0, REG_USB2PHY_MISCTUNE);
	reg_rmwl(REG_USB2PHY_MISCTUNE_REFCLK_GATE_OFF, 0, REG_USB2PHY_MISCTUNE);
	udelay(30);

	ret = reg_poll(-1, 0, 1, REG_AUSBEVT_UTMIACT_EVTCNT, "UTMI clock active (1)");
	if(ret < 0)
		return ret;

	/* set host mode on PHY */

	if(host)
		reg_rmwl(REG_USB2PHY_USBCTL_MODE_MASK, REG_USB2PHY_USBCTL_MODE_HOST,
			REG_USB2PHY_USBCTL);
	else
		reg_rmwl(REG_USB2PHY_USBCTL_MODE_MASK, REG_USB2PHY_USBCTL_MODE_DEV,
			REG_USB2PHY_USBCTL);

	/* power on the USB DRD core */

	reg_rmwl(REG_PIPEHDLR_AON_GEN_DRD_FORCE_CLAMP_EN, 0, REG_PIPEHDLR_AON_GEN);
	reg_rmwl(0, REG_PIPEHDLR_AON_GEN_DRD_SW_VCC_RESET, REG_PIPEHDLR_AON_GEN);

	/* wait for fast PHY ready */

	reg_rmwl(0, REG_PIPEHDLR_NONSEL_OVRD_DUMMY_PHY_READY,
		REG_PIPEHDLR_NONSEL_OVRD);

	ret = reg_poll(REG_PIPEPHY_STATUS_READY, REG_PIPEPHY_STATUS_READY, 0,
			REG_PIPEPHY_STATUS, "PIPE PHY ready");
	if(ret < 0)
		return ret;

	ret = reg_poll(-1u, 0, 1, REG_AUSBEVT_UTMIACT_EVTCNT, "UTMI clock active (2)");
	if(ret < 0)
		return ret;

	/* set up CIO bridge details */

	reg_rmwl(0, REG_CIO_LFPSOFFSET_AP_MASK, REG_CIO_LFPSOFFSET);
	reg_rmwl(REG_CIO_LFPSOFFSET_MAC_DFLT, REG_CIO_LFPSOFFSET_MAC_MASK,
		REG_CIO_LFPSOFFSET);
	reg_rmwl(0, REG_CIO_BWNGTOFFSET_AP_MASK, REG_CIO_BWNGTOFFSET);
	reg_rmwl(REG_CIO_BWNGTOFFSET_MAC_DFLT, REG_CIO_BWNGTOFFSET_MAC_MASK,
		REG_CIO_BWNGTOFFSET);
	reg_rmwl(REG_CIO_LINKTIMER1_PEND_HP_MASK, REG_CIO_LINKTIMER1_PEND_HP_DFLT,
		REG_CIO_LINKTIMER1);
	reg_rmwl(REG_CIO_LINKTIMER1_PM_LC_MASK, REG_CIO_LINKTIMER1_PM_LC_DFLT,
		REG_CIO_LINKTIMER1);
	reg_rmwl(REG_CIO_LINKTIMER1_PM_ENTRY_MASK, REG_CIO_LINKTIMER1_PM_ENTRY_DFLT,
		REG_CIO_LINKTIMER1);

	atc->usb2en = 1;

	return 0;
}

static int apple_atcphy_m1_usb2_disable(struct apple_atcphy_m1 *atc)
{
	reg_rmwl(REG_PIPEHDLR_AON_GEN_DRD_SW_VCC_RESET, 0, REG_PIPEHDLR_AON_GEN);
	reg_rmwl(0, REG_PIPEHDLR_AON_GEN_DRD_FORCE_CLAMP_EN, REG_PIPEHDLR_AON_GEN);
	reg_rmwl(REG_USB2PHY_USBCTL_MODE_MASK, REG_USB2PHY_USBCTL_MODE_ISOLATION,
		REG_USB2PHY_USBCTL);
	reg_rmwl(0, REG_USB2PHY_CTL_SIDDQ, REG_USB2PHY_CTL);
	reg_rmwl(0, REG_USB2PHY_CTL_PORT_RESET, REG_USB2PHY_CTL);
	reg_rmwl(0, REG_USB2PHY_CTL_RESET, REG_USB2PHY_CTL);
	reg_rmwl(REG_USB2PHY_CTL_APB_RESETN, 0, REG_USB2PHY_CTL);
	reg_rmwl(0, REG_USB2PHY_MISCTUNE_APBCLK_GATE_OFF, REG_USB2PHY_MISCTUNE);
	reg_rmwl(0, REG_USB2PHY_MISCTUNE_REFCLK_GATE_OFF, REG_USB2PHY_MISCTUNE);

	atc->usb2en = 0;

	return 0;
}

static int apple_atcphy_m1_usbpipe_req(struct apple_atcphy_m1 *atc, unsigned req)
{
	reg_rmwl(1, req, REG_PIPEHDLR_PIPE_IF_REQ);
	return reg_poll(1, req, 0, REG_PIPEHDLR_PIPE_IF_ACK, "PIPE req/ack");
}

static const char *const apple_atcphy_m1_lane_tunable_usb[] = {
	"tunable-USB_LN%d_AUSPMA_TX_TOP",
	"tunable-USB_LN%d_AUSPMA_RX_TOP",
	"tunable-USB_LN%d_AUSPMA_RX_SHM",
	"tunable-USB_LN%d_AUSPMA_RX_EQ",
	NULL
};
static const char *const apple_atcphy_m1_lane_tunable_dp[] = {
	"tunable-DP_LN%d_AUSPMA_TX_TOP",
	NULL
};
static const char *const apple_atcphy_m1_lane_tunable_tbt[] = {
	"tunable-CIO_LN%d_AUSPMA_TX_TOP",
	"tunable-CIO_LN%d_AUSPMA_RX_TOP",
	"tunable-CIO_LN%d_AUSPMA_RX_SHM",
	"tunable-CIO_LN%d_AUSPMA_RX_EQ",
	NULL
};
static const char *const *const apple_atcphy_m1_lane_tunable[4] = {
	[ REG_ACIOPHY_MODE_VAL_USB ] = apple_atcphy_m1_lane_tunable_usb,
	[ REG_ACIOPHY_MODE_VAL_DP ]  = apple_atcphy_m1_lane_tunable_dp,
	[ REG_ACIOPHY_MODE_VAL_TBT ] = apple_atcphy_m1_lane_tunable_tbt,
	[ REG_ACIOPHY_MODE_VAL_OFF]  = NULL,
};

/* lane0, lane1, orientation */
#define XBAR_VALID	0x80000000
#define DPMODE_VALID	0x40000000
static const unsigned apple_atcphy_m1_xbar_config[4][4][2] = {
	[ REG_ACIOPHY_MODE_VAL_USB ] = {
		[ REG_ACIOPHY_MODE_VAL_DP ]  = { [ 0 ] = XBAR_VALID | REG_ACIOPHY_XBAR_PROTO_USB0_DP1 |
						       DPMODE_VALID | REG_ACIOPHY_XBAR_DPMODE_DPX },
		[ REG_ACIOPHY_MODE_VAL_OFF ] = { [ 0 ] = XBAR_VALID | REG_ACIOPHY_XBAR_PROTO_USB0 },
	},
	[ REG_ACIOPHY_MODE_VAL_DP ] = {
		[ REG_ACIOPHY_MODE_VAL_USB ] = { [ 1 ] = XBAR_VALID | REG_ACIOPHY_XBAR_PROTO_DP0_USB1 |
						       DPMODE_VALID | REG_ACIOPHY_XBAR_DPMODE_DPX },
		[ REG_ACIOPHY_MODE_VAL_DP ]  = { [ 0 ] = XBAR_VALID | REG_ACIOPHY_XBAR_PROTO_DP01 |
						       DPMODE_VALID | REG_ACIOPHY_XBAR_DPMODE_DP01,
						 [ 1 ] = XBAR_VALID | REG_ACIOPHY_XBAR_PROTO_DP01 |
						       DPMODE_VALID | REG_ACIOPHY_XBAR_DPMODE_DPX },
	},
	[ REG_ACIOPHY_MODE_VAL_TBT ] = {
		[ REG_ACIOPHY_MODE_VAL_TBT ] = { [ 0 ] = XBAR_VALID | REG_ACIOPHY_XBAR_PROTO_TBT01,
						 [ 1 ] = XBAR_VALID | REG_ACIOPHY_XBAR_PROTO_TBT10 },
	},
	[ REG_ACIOPHY_MODE_VAL_OFF ] = {
		[ REG_ACIOPHY_MODE_VAL_USB ] = { [ 1 ] = XBAR_VALID | REG_ACIOPHY_XBAR_PROTO_USB1 },
	},
};

static int apple_atcphy_m1_cio_disable(struct apple_atcphy_m1 *atc);

static int apple_atcphy_m1_cio_enable(struct apple_atcphy_m1 *atc, unsigned long mode,
				      unsigned orientation)
{
	char name[64] = { 0 }; /* enough for any lane_tunable name */
	const char *const *tuna;
	unsigned lane[2], cfg, msk;
	int ret, idx;

	switch(mode) {
	case TYPEC_MODE_USB2:
	default:
		if(!atc->mode)
			return 0;
		return apple_atcphy_m1_cio_disable(atc);
	case TYPEC_MODE_USB3:
		lane[0] = orientation ? REG_ACIOPHY_MODE_VAL_DP : REG_ACIOPHY_MODE_VAL_USB;
		lane[1] = orientation ? REG_ACIOPHY_MODE_VAL_USB : REG_ACIOPHY_MODE_VAL_DP;
		break;
	case TYPEC_MODE_USB4:
		lane[0] = REG_ACIOPHY_MODE_VAL_TBT;
		lane[1] = REG_ACIOPHY_MODE_VAL_TBT;
		break;
	}

	if(atc->mode == mode)
		return 0;

	if(atc->mode)
		apple_atcphy_m1_cio_disable(atc);

	/* power on high speed PHY */

	reg_rmwl(0, REG_ATCPHY_MISC_AON_RESETN, REG_ATCPHY_MISC);

	reg_rmwl(0, REG_ATCPHY_PWRDN_CTRL_SLEEP_SML, REG_ATCPHY_PWRDN_CTRL);
	ret = reg_poll(REG_ATCPHY_PWRDN_STAT_SLEEP_SML, 0, 1, REG_ATCPHY_PWRDN_STAT,
		"CIO small wake");
	if(ret < 0)
		return ret;
	reg_rmwl(0, REG_ATCPHY_PWRDN_CTRL_SLEEP_BIG, REG_ATCPHY_PWRDN_CTRL);
	ret = reg_poll(REG_ATCPHY_PWRDN_STAT_SLEEP_BIG, 0, 1, REG_ATCPHY_PWRDN_STAT,
		"CIO big wake");
	if(ret < 0)
		return ret;

	reg_rmwl(REG_ATCPHY_PWRDN_CTRL_CLAMP, 0, REG_ATCPHY_PWRDN_CTRL);
	reg_rmwl(0, REG_ATCPHY_PWRDN_CTRL_APB_RESETN, REG_ATCPHY_PWRDN_CTRL);

	/* prepare high speed PHY tunables depending on mode */

	if((ret = apple_atcphy_m1_tunable(atc, "tunable-fuse")) < 0)
		return ret;

	if((ret = apple_atcphy_m1_tunable(atc, "tunable-ATC0AXI2AF")) < 0)
		return ret;
	if((ret = apple_atcphy_m1_tunable(atc, "tunable-ATC_FABRIC")) < 0)
		return ret;
	if(lane[0] == REG_ACIOPHY_MODE_VAL_USB || lane[1] == REG_ACIOPHY_MODE_VAL_USB)
		if((ret = apple_atcphy_m1_tunable(atc, "tunable-USB_ACIOPHY_TOP")) < 0)
			return ret;
	if((ret = apple_atcphy_m1_tunable(atc, "tunable-AUS_CMN_SHM")) < 0)
		return ret;
	if((ret = apple_atcphy_m1_tunable(atc, "tunable-AUS_CMN_TOP")) < 0)
		return ret;
	if((ret = apple_atcphy_m1_tunable(atc, "tunable-AUSPLL_CORE")) < 0)
		return ret;
	if((ret = apple_atcphy_m1_tunable(atc, "tunable-AUSPLL_TOP")) < 0)
		return ret;
	if((ret = apple_atcphy_m1_tunable(atc, "tunable-CIO3PLL_CORE")) < 0)
		return ret;
	if((ret = apple_atcphy_m1_tunable(atc, "tunable-CIO3PLL_TOP")) < 0)
		return ret;

	for(idx=0; idx<2; idx++) {
		tuna = apple_atcphy_m1_lane_tunable[lane[idx]];
		if(!tuna)
			continue;

		for(; *tuna; tuna++) {
			snprintf(name, sizeof(name) - 1, *tuna, idx);
			if((ret = apple_atcphy_m1_tunable(atc, name)) < 0)
				return ret;
		}
	}

	if(lane[0] == REG_ACIOPHY_MODE_VAL_USB || lane[1] == REG_ACIOPHY_MODE_VAL_USB) {
		reg_rmwl(0, REG_AUSPLL_FSM_CTRL_APBREQ_OVSEL, REG_AUSPLL_FSM_CTRL);
		reg_rmwl(0, REG_AUSPLL_CMD_OVRD_APB, REG_AUSPLL_CMD_OVRD);
	}

	/* configure sleep / wake */

	reg_rmwl(0, REG_ACIOPHY_CFG0_CMN_SML | REG_ACIOPHY_CFG0_CMN_SML_OV,
		REG_ACIOPHY_CFG0);
	udelay(2);
	reg_rmwl(0, REG_ACIOPHY_CFG0_CMN_BIG | REG_ACIOPHY_CFG0_CMN_BIG_OV,
		REG_ACIOPHY_CFG0);
	udelay(2);
	reg_rmwl(REG_ACIOPHY_CFG0_CMN_CLAMP, REG_ACIOPHY_CFG0_CMN_CLAMP_OV,
		REG_ACIOPHY_CFG0);
	udelay(2);

	reg_rmwl(0, REG_ACIOPHY_SLEEPCTRL_TX_SML | REG_ACIOPHY_SLEEPCTRL_TX_SML_OV,
		REG_ACIOPHY_SLEEPCTRL);
	udelay(2);
	reg_rmwl(0, REG_ACIOPHY_SLEEPCTRL_TX_BIG | REG_ACIOPHY_SLEEPCTRL_TX_BIG_OV,
		REG_ACIOPHY_SLEEPCTRL);
	udelay(2);
	reg_rmwl(REG_ACIOPHY_SLEEPCTRL_TX_CLAMP, REG_ACIOPHY_SLEEPCTRL_TX_CLAMP_OV,
		REG_ACIOPHY_SLEEPCTRL);
	udelay(2);

	reg_rmwl(0, REG_ACIOPHY_CFG0_RX_SML | REG_ACIOPHY_CFG0_RX_SML_OV,
		REG_ACIOPHY_CFG0);
	udelay(2);
	reg_rmwl(0, REG_ACIOPHY_CFG0_RX_BIG | REG_ACIOPHY_CFG0_RX_BIG_OV,
		REG_ACIOPHY_CFG0);
	udelay(2);
	reg_rmwl(REG_ACIOPHY_CFG0_RX_CLAMP, REG_ACIOPHY_CFG0_RX_CLAMP_OV,
		REG_ACIOPHY_CFG0);
	udelay(2);

	/* enable CIO high speed clock */

	reg_rmwl(0, REG_CIO3PLL_CLKOUT_PCLK_EN, REG_CIO3PLL_CLKOUT);
	reg_rmwl(0, REG_CIO3PLL_CLKOUT_REFCLK_EN, REG_CIO3PLL_CLKOUT);

	/* configure lanes to match selected mode */

	reg_rmwl(REG_ACIOPHY_MODE_RX0_MASK | REG_ACIOPHY_MODE_TX0_MASK,
		 (lane[0] << REG_ACIOPHY_MODE_RX0_SHIFT) |
		 (lane[0] << REG_ACIOPHY_MODE_TX0_SHIFT), REG_ACIOPHY_MODE);
	reg_rmwl(REG_ACIOPHY_MODE_RX1_MASK | REG_ACIOPHY_MODE_TX1_MASK,
		 (lane[1] << REG_ACIOPHY_MODE_RX1_SHIFT) |
		 (lane[1] << REG_ACIOPHY_MODE_TX1_SHIFT), REG_ACIOPHY_MODE);

	/* configure crossbar routing to PHY lanes */

	cfg = apple_atcphy_m1_xbar_config[lane[0]][lane[1]][orientation];
	if(!(cfg & XBAR_VALID))
		return -EINVAL;
	msk = REG_ACIOPHY_XBAR_PROTO_MASK;
	if(cfg & DPMODE_VALID)
		msk |= REG_ACIOPHY_XBAR_DPMODE_MASK;
	cfg &= ~(XBAR_VALID | DPMODE_VALID);

	reg_rmwl(msk, cfg, REG_ACIOPHY_XBAR);

	if(lane[0] == REG_ACIOPHY_MODE_VAL_DP) {
		reg_rmwl(0, REG_AUSPMA_RX_FSM_CTRL_OVRD, REG_AUSPMA_RX0_FSM_CTRL);
		reg_rmwl(REG_AUSPMA_RX_FSM_CTRL_REQ, 0, REG_AUSPMA_RX0_FSM_CTRL);
	}
	if(lane[1] == REG_ACIOPHY_MODE_VAL_DP) {
		reg_rmwl(0, REG_AUSPMA_RX_FSM_CTRL_OVRD, REG_AUSPMA_RX1_FSM_CTRL);
		reg_rmwl(REG_AUSPMA_RX_FSM_CTRL_REQ, 0, REG_AUSPMA_RX1_FSM_CTRL);
	}

	if(lane[1] == REG_ACIOPHY_MODE_VAL_USB && orientation)
		reg_rmwl(0, REG_ATCPHY_MISC_USB_LANE_SWAP, REG_ATCPHY_MISC);
	else
		reg_rmwl(REG_ATCPHY_MISC_USB_LANE_SWAP, 0, REG_ATCPHY_MISC);

	/* take high-speed PHY out of reset */

	reg_rmwl(0, REG_ATCPHY_PWRDN_CTRL_PHY_RESETN, REG_ATCPHY_PWRDN_CTRL);

	/* USB setup for SS PHY */

	if(lane[0] == REG_ACIOPHY_MODE_VAL_USB || lane[1] == REG_ACIOPHY_MODE_VAL_USB) {
		reg_rmwl(REG_PIPEHDLR_OVRD_VAL0_RXDET, 0, REG_PIPEHDLR_OVRD_VAL0);
		reg_rmwl(0, REG_PIPEHDLR_OVRD_RXVALID, REG_PIPEHDLR_OVRD);
		reg_rmwl(0, REG_PIPEHDLR_OVRD_RXDET, REG_PIPEHDLR_OVRD);

		if((ret = apple_atcphy_m1_usbpipe_req(atc, 1)) < 0)
			return ret;

		reg_rmwl(0, REG_ACIOPHY_BIST_RST_RESETN, REG_ACIOPHY_BIST_RST);
		reg_rmwl(0, REG_ACIOPHY_BIST_OV_LN0_RESETN, REG_ACIOPHY_BIST_OV);
		ret = reg_poll(REG_ACIOPHY_BIST_STAT_LN0_BUSY, 0, 0, REG_ACIOPHY_BIST_STAT,
			"Lane 0 not busy");
		if(ret < 0)
			return ret;

		reg_rmwl(0, REG_ACIOPHY_BIST_CFG0_SET, REG_ACIOPHY_BIST_CFG0);
		reg_rmwl(REG_ACIOPHY_BIST_CFG0_SET, 0, REG_ACIOPHY_BIST_CFG0);
		reg_rmwl(REG_ACIOPHY_BIST_CFG1_LN0_PWRDN_MASK,
			REG_ACIOPHY_BIST_CFG1_LN0_PWRDN_OFF, REG_ACIOPHY_BIST_CFG1);
		reg_rmwl(0, REG_ACIOPHY_BIST_OV_LN0_PWRDN, REG_ACIOPHY_BIST_OV);
		reg_rmwl(0, REG_ACIOPHY_BIST_EN_CLK, REG_ACIOPHY_BIST_EN);
		reg_rmwl(0, REG_ACIOPHY_BIST_EN_CIOBIST, REG_ACIOPHY_BIST_EN);
		writel(0, atc->reg[1] + REG_ACIOPHY_BIST_EN);

		ret = reg_poll(REG_ACIOPHY_BIST_STAT_LN0_READY, 0, 1, REG_ACIOPHY_BIST_STAT,
			"Lane 0 ready");
		if(ret < 0)
			return ret;

		ret = reg_poll(REG_ACIOPHY_BIST_STAT_LN0_BUSY, 0, 0, REG_ACIOPHY_BIST_STAT,
			"Lane 0 not busy");
		if(ret < 0)
			return ret;

		reg_rmwl(REG_PIPEHDLR_NONSEL_OVRD_STATE_MASK,
			REG_PIPEHDLR_NONSEL_OVRD_STATE_OFF, REG_PIPEHDLR_NONSEL_OVRD);
		reg_rmwl(REG_PIPEHDLR_NONSEL_OVRD_RESET, 0, REG_PIPEHDLR_NONSEL_OVRD);

		writel(0, atc->reg[1] + REG_ACIOPHY_BIST_OV);
		reg_rmwl(0, REG_ACIOPHY_BIST_EN_CLK, REG_ACIOPHY_BIST_EN);
		reg_rmwl(0, REG_ACIOPHY_BIST_EN_CIOBIST, REG_ACIOPHY_BIST_EN);

		reg_rmwl(REG_PIPEHDLR_MUXSEL_CLKEN_MASK, 0, REG_PIPEHDLR_MUXSEL);
		reg_rmwl(REG_PIPEHDLR_MUXSEL_MODE_MASK, REG_PIPEHDLR_MUXSEL_MODE_USB30,
			REG_PIPEHDLR_MUXSEL);
		reg_rmwl(REG_PIPEHDLR_MUXSEL_CLKEN_MASK, REG_PIPEHDLR_MUXSEL_CLKEN_USB30,
			REG_PIPEHDLR_MUXSEL);

		reg_rmwl(REG_PIPEHDLR_OVRD_RXVALID, 0, REG_PIPEHDLR_OVRD);
		reg_rmwl(REG_PIPEHDLR_OVRD_RXDET, 0, REG_PIPEHDLR_OVRD);

		if((ret = apple_atcphy_m1_usbpipe_req(atc, 0)) < 0)
			return ret;
	}

	atc->mode = mode;

	return 0;
}

static int apple_atcphy_m1_cio_disable(struct apple_atcphy_m1 *atc)
{
	int ret = 0;

	/* disable USB3 PHY receiver detect */

	reg_rmwl(REG_PIPEHDLR_OVRD_VAL0_RXDET, 0, REG_PIPEHDLR_OVRD_VAL0);
	reg_rmwl(0, REG_PIPEHDLR_OVRD_RXVALID, REG_PIPEHDLR_OVRD);
	reg_rmwl(0, REG_PIPEHDLR_OVRD_RXDET, REG_PIPEHDLR_OVRD);

	/* disable USB3 PHY clocks */

	ret |= apple_atcphy_m1_usbpipe_req(atc, 1);
	reg_rmwl(REG_PIPEHDLR_MUXSEL_CLKEN_MASK, 0, REG_PIPEHDLR_MUXSEL);
	reg_rmwl(REG_PIPEHDLR_MUXSEL_MODE_MASK, REG_PIPEHDLR_MUXSEL_MODE_USB2,
		REG_PIPEHDLR_MUXSEL);
	reg_rmwl(REG_PIPEHDLR_MUXSEL_CLKEN_MASK, REG_PIPEHDLR_MUXSEL_CLKEN_TBT,
		REG_PIPEHDLR_MUXSEL);
	ret |= apple_atcphy_m1_usbpipe_req(atc, 0);

	/* reset and power off high speed PHY */

	reg_rmwl(REG_ATCPHY_PWRDN_CTRL_PHY_RESETN, 0, REG_ATCPHY_PWRDN_CTRL);
	reg_rmwl(0, REG_ATCPHY_PWRDN_CTRL_CLAMP, REG_ATCPHY_PWRDN_CTRL);
	reg_rmwl(REG_ATCPHY_MISC_AON_RESETN, 0, REG_ATCPHY_MISC);
	reg_rmwl(REG_ATCPHY_PWRDN_CTRL_APB_RESETN, 0, REG_ATCPHY_PWRDN_CTRL);

	reg_rmwl(REG_ATCPHY_PWRDN_CTRL_SLEEP_BIG, 0, REG_ATCPHY_PWRDN_CTRL);
	ret |= reg_poll(REG_ATCPHY_PWRDN_STAT_SLEEP_BIG, 0, 0, REG_ATCPHY_PWRDN_STAT,
		"CIO big sleep");
	reg_rmwl(REG_ATCPHY_PWRDN_CTRL_SLEEP_SML, 0, REG_ATCPHY_PWRDN_CTRL);
	ret |= reg_poll(REG_ATCPHY_PWRDN_STAT_SLEEP_SML, 0, 0, REG_ATCPHY_PWRDN_STAT,
		"CIO small sleep");

	/* reset pipe handler */

	reg_rmwl(REG_PIPEHDLR_NONSEL_OVRD_STATE_MASK,
		REG_PIPEHDLR_NONSEL_OVRD_STATE_DOWN, REG_PIPEHDLR_NONSEL_OVRD);
	reg_rmwl(0, REG_PIPEHDLR_NONSEL_OVRD_RESET, REG_PIPEHDLR_NONSEL_OVRD);

	atc->mode = 0;

	return ret;
}

static int apple_atcphy_m1_power_on(struct phy *phy)
{
	struct apple_atcphy_m1 *atc = phy_get_drvdata(phy);
	dev_info(atc->dev, "power_on\n");
	return 0;
}

static int apple_atcphy_m1_power_off(struct phy *phy)
{
	struct apple_atcphy_m1 *atc = phy_get_drvdata(phy);
	dev_info(atc->dev, "power_off\n");
	return 0;
}

static int apple_atcphy_m1_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct apple_atcphy_m1 *atc = phy_get_drvdata(phy);
	dev_info(atc->dev, "set_mode(%d, %d)\n", mode, submode);
	return 0;
}

static int apple_atcphy_m1_set_role(struct usb_role_switch *sw, enum usb_role role)
{
	struct apple_atcphy_m1 *atc = usb_role_switch_get_drvdata(sw);
	dev_info(atc->dev, "set_role(%d)\n", role);
	atc->role = role;
	return 0;
}

static int apple_atcphy_m1_set_orientation(struct typec_switch *sw,
					   enum typec_orientation orientation)
{
	struct apple_atcphy_m1 *atc = typec_switch_get_drvdata(sw);
	dev_info(atc->dev, "set_orientation(%d)\n", orientation);
	atc->orientation = orientation;
	return 0;
}

static int apple_atcphy_m1_set_mux(struct typec_mux *mux, struct typec_mux_state *state)
{
	struct apple_atcphy_m1 *atc = typec_mux_get_drvdata(mux);
	unsigned long mode = state->mode;
	int ret;

	dev_info(atc->dev, "set_mux(%ld, 0x%x)\n", mode, state->alt ? state->alt->svid : 0);

	if(atc->role == USB_ROLE_NONE)
		mode = 0;

	if(mode) {
		ret = apple_atcphy_m1_cio_enable(atc, mode,
					atc->orientation == TYPEC_ORIENTATION_REVERSE);
		if(ret < 0) {
			apple_atcphy_m1_cio_disable(atc);
			return ret;
		}
	} else
		apple_atcphy_m1_cio_disable(atc);

	return 0;
}

static const struct phy_ops apple_atcphy_m1_ops = {
	.power_on	= apple_atcphy_m1_power_on,
	.power_off	= apple_atcphy_m1_power_off,
	.set_mode	= apple_atcphy_m1_set_mode,
};

static int clk_apple_atcphy_m1_enable(struct clk_hw *hw)
{
	struct apple_atcphy_m1 *atc = container_of(hw, struct apple_atcphy_m1, clkhw);
	int ret;

	dev_info(atc->dev, "utmiclk_enable()\n");

	ret = apple_atcphy_m1_usb2_enable(atc, atc->role != USB_ROLE_DEVICE);
	if(ret < 0) {
		apple_atcphy_m1_usb2_disable(atc);
		return ret;
	}

	return 0;
}

static void clk_apple_atcphy_m1_disable(struct clk_hw *hw)
{
	struct apple_atcphy_m1 *atc = container_of(hw, struct apple_atcphy_m1, clkhw);

	dev_info(atc->dev, "utmiclk_disable()\n");

	apple_atcphy_m1_usb2_disable(atc);
}

static int clk_apple_atcphy_m1_is_enabled(struct clk_hw *hw)
{
	struct apple_atcphy_m1 *atc = container_of(hw, struct apple_atcphy_m1, clkhw);
	return atc->usb2en;
}

const struct clk_ops clk_apple_atcphy_m1_ops = {
	.enable = clk_apple_atcphy_m1_enable,
	.disable = clk_apple_atcphy_m1_disable,
	.is_enabled = clk_apple_atcphy_m1_is_enabled,
};

/* this lets us check that functions aren't grossly incompatible, should the arguments change */
static void wrap_typec_switch_unregister(void *p) { typec_switch_unregister(p); }
static void wrap_typec_mux_unregister(void *p) { typec_mux_unregister(p); }
static void wrap_usb_role_switch_unregister(void *p) { usb_role_switch_unregister(p); }

static int apple_atcphy_m1_probe(struct platform_device *pdev)
{
	struct apple_atcphy_m1 *atc;
	struct usb_role_switch_desc dr_desc = { };
	struct typec_switch_desc sw_desc = { };
	struct typec_mux_desc mux_desc = { };
	struct clk_init_data clk_init = {};
	struct resource *rsrc;
	int err, i;

	atc = devm_kzalloc(&pdev->dev, sizeof(*atc), GFP_KERNEL);
	if(!atc)
		return -ENOMEM;
	atc->dev = &pdev->dev;

	err = clk_bulk_get_all(atc->dev, &atc->clks);
	if(err < 0) {
		dev_err(&pdev->dev, "clk_bulk_get_all failed.\n");
		return err;
	}
	atc->num_clks = err;

	for(i=0; i<pdev->num_resources; i++) {
		rsrc = &pdev->resource[i];
		if(resource_type(rsrc) != IORESOURCE_MEM)
			continue;

		if(atc->num_reg >= MAX_REG) {
			dev_warn(atc->dev, "too many 'reg' entries, expected up to %d.\n",
				 MAX_REG);
			break;
		}

		atc->reg[atc->num_reg] = devm_ioremap(atc->dev, rsrc->start, resource_size(rsrc));
		if(IS_ERR(atc->reg[atc->num_reg])) {
			err = PTR_ERR(atc->reg[atc->num_reg]);
			dev_err(atc->dev, "failed to map MMIO %d: %d.\n", atc->num_reg, err);
			return err;
		}
		atc->num_reg ++;
	}

	err = clk_bulk_prepare_enable(atc->num_clks, atc->clks);
	if(err) {
		dev_err(&pdev->dev, "clk_bulk_prepare_enable failed.\n");
		return err;
	}

	platform_set_drvdata(pdev, atc);

	atc->phy = devm_phy_create(atc->dev, NULL, &apple_atcphy_m1_ops);
	if(IS_ERR(atc->phy)) {
		dev_err(atc->dev, "failed to create PHY: %ld.\n", PTR_ERR(atc->phy));
		return PTR_ERR(atc->phy);
	}
	phy_set_drvdata(atc->phy, atc);

	atc->phy_provider = devm_of_phy_provider_register(atc->dev, of_phy_simple_xlate);
	if(IS_ERR(atc->phy_provider)) {
		dev_err(atc->dev, "failed to register PHY provider: %ld.\n",
			PTR_ERR(atc->phy_provider));
		return PTR_ERR(atc->phy_provider);
	}

	sw_desc.fwnode = atc->dev->fwnode;
	sw_desc.drvdata = atc;
	sw_desc.set = apple_atcphy_m1_set_orientation;
	atc->typec_sw = typec_switch_register(atc->dev, &sw_desc);
	if(IS_ERR(atc->typec_sw)) {
		dev_err(atc->dev, "failed to register orientation switch: %ld.\n",
			PTR_ERR(atc->typec_sw));
		return PTR_ERR(atc->typec_sw);
	}
	err = devm_add_action(atc->dev, wrap_typec_switch_unregister, atc->typec_sw);
	if(err)
		return err;

	mux_desc.fwnode = atc->dev->fwnode;
	mux_desc.drvdata = atc;
	mux_desc.set = apple_atcphy_m1_set_mux;
	atc->typec_mux = typec_mux_register(atc->dev, &mux_desc);
	if(IS_ERR(atc->typec_mux)) {
		dev_err(atc->dev, "failed to register lane mux: %ld.\n", PTR_ERR(atc->typec_mux));
		return PTR_ERR(atc->typec_mux);
	}
	err = devm_add_action(atc->dev, wrap_typec_mux_unregister, atc->typec_mux);
	if(err)
		return err;

	dr_desc.fwnode = atc->dev->fwnode;
	dr_desc.driver_data = atc;
	dr_desc.set = apple_atcphy_m1_set_role;
	atc->role_sw = usb_role_switch_register(atc->dev, &dr_desc);
	if(IS_ERR(atc->role_sw)) {
		dev_err(atc->dev, "failed to register role switch: %ld.\n", PTR_ERR(atc->role_sw));
		return PTR_ERR(atc->role_sw);
	}
	err = devm_add_action(atc->dev, wrap_usb_role_switch_unregister, atc->role_sw);
	if(err)
		return err;

	err = of_property_read_string(atc->dev->of_node, "clock-output-names", &clk_init.name);
	if(err < 0)
		return err;
	clk_init.num_parents = 0;
	clk_init.ops = &clk_apple_atcphy_m1_ops;

	atc->clkhw.init = &clk_init;
	err = devm_clk_hw_register(atc->dev, &atc->clkhw);
	if(err)
		return err;

	return of_clk_add_provider(atc->dev->of_node, of_clk_src_simple_get, atc->clkhw.clk);
}

static const struct of_device_id apple_atcphy_m1_match[] = {
	{ .compatible = "apple,atc-phy-m1" },
	{},
};
MODULE_DEVICE_TABLE(of, apple_atcphy_m1_match);

static struct platform_driver apple_atcphy_m1_driver = {
	.driver = {
		.name = "phy-apple-atc-m1",
		.of_match_table = apple_atcphy_m1_match,
	},
	.probe = apple_atcphy_m1_probe,
};

module_platform_driver(apple_atcphy_m1_driver);

MODULE_AUTHOR("Corellium LLC");
MODULE_DESCRIPTION("Apple M1 SoC type-C PHY driver");
MODULE_LICENSE("GPL");
