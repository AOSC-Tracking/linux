/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Thunderbolt driver - NHI driver
 *
 * Copyright (c) 2014 Andreas Noever <andreas.noever@gmail.com>
 * Copyright (C) 2018, Intel Corporation
 */

#ifndef DSL3510_H_
#define DSL3510_H_

#include <linux/thunderbolt.h>

enum nhi_fw_mode {
	NHI_FW_SAFE_MODE,
	NHI_FW_AUTH_MODE,
	NHI_FW_EP_MODE,
	NHI_FW_CM_MODE,
};

enum nhi_mailbox_cmd {
	NHI_MAILBOX_SAVE_DEVS = 0x05,
	NHI_MAILBOX_DISCONNECT_PCIE_PATHS = 0x06,
	NHI_MAILBOX_DRV_UNLOADS = 0x07,
	NHI_MAILBOX_DISCONNECT_PA = 0x10,
	NHI_MAILBOX_DISCONNECT_PB = 0x11,
	NHI_MAILBOX_ALLOW_ALL_DEVS = 0x23,
};

int nhi_mailbox_cmd(struct tb_nhi *nhi, enum nhi_mailbox_cmd cmd, u32 data);
enum nhi_fw_mode nhi_mailbox_mode(struct tb_nhi *nhi);

/**
 * struct tb_nhi_ops - NHI specific optional operations
 * @init: NHI specific initialization
 * @suspend_noirq: NHI specific suspend_noirq hook
 * @resume_noirq: NHI specific resume_noirq hook
 * @runtime_suspend: NHI specific runtime_suspend hook
 * @runtime_resume: NHI specific runtime_resume hook
 * @shutdown: NHI specific shutdown
 */
struct tb_nhi_ops {
	int (*init)(struct tb_nhi *nhi);
	int (*suspend_noirq)(struct tb_nhi *nhi, bool wakeup);
	int (*resume_noirq)(struct tb_nhi *nhi);
	int (*runtime_suspend)(struct tb_nhi *nhi);
	int (*runtime_resume)(struct tb_nhi *nhi);
	void (*shutdown)(struct tb_nhi *nhi);

	int (*mailbox_cmd)(struct tb_nhi *nhi, enum nhi_mailbox_cmd cmd, u32 data);
	enum nhi_fw_mode (*mailbox_mode)(struct tb_nhi *nhi);

	void (*ring_start)(struct tb_ring *ring);
	void (*ring_stop)(struct tb_ring *ring);
	struct tb_ring *(*ring_alloc_tx)(struct tb_nhi *nhi, int hop, int size,
				 unsigned int flags);
	struct tb_ring *(*ring_alloc_rx)(struct tb_nhi *nhi, int hop, int size,
				 unsigned int flags, int e2e_tx_hop,
				 u16 sof_mask, u16 eof_mask,
				 void (*start_poll)(void *), void *poll_data);
	void (*ring_free)(struct tb_ring *ring);
	int (*ring_enqueue)(struct tb_ring *ring, struct ring_frame *frame);

	struct ring_frame *(*ring_poll)(struct tb_ring *ring);
	void (*ring_poll_complete)(struct tb_ring *ring);

	void (*notify_pci_tunnel)(struct tb_nhi *nhi, unsigned down_adapter);
	int (*read_drom)(struct tb_nhi *nhi, unsigned offs, void *buf, unsigned size);
};

int pci_nhi_mailbox_cmd(struct tb_nhi *nhi, enum nhi_mailbox_cmd cmd, u32 data);
enum nhi_fw_mode pci_nhi_mailbox_mode(struct tb_nhi *nhi);

void pci_nhi_ring_start(struct tb_ring *ring);
void pci_nhi_ring_stop(struct tb_ring *ring);
struct tb_ring *pci_nhi_ring_alloc_tx(struct tb_nhi *nhi, int hop, int size,
			 unsigned int flags);
struct tb_ring *pci_nhi_ring_alloc_rx(struct tb_nhi *nhi, int hop, int size,
			 unsigned int flags, int e2e_tx_hop,
			 u16 sof_mask, u16 eof_mask,
			 void (*start_poll)(void *), void *poll_data);
void pci_nhi_ring_free(struct tb_ring *ring);
int pci_nhi_ring_enqueue(struct tb_ring *ring, struct ring_frame *frame);

struct ring_frame *pci_nhi_ring_poll(struct tb_ring *ring);
void pci_nhi_ring_poll_complete(struct tb_ring *ring);

extern const struct tb_nhi_ops pci_nhi_ops;
extern const struct tb_nhi_ops icl_nhi_ops;

/*
 * Notify NHI infrastructure about a new PCI tunnel. This is used on
 * systems where extra MMIO has to be done to finalize a tunnel.
 */

void nhi_notify_pci_tunnel(struct tb_nhi *nhi, unsigned down_adapter);

/*
 * Read root switch DROM. Returns number of bytes read or negative
 * for failure. If buf == NULL, returns size of DROM.
 */

int nhi_read_drom(struct tb_nhi *nhi, unsigned offs, void *buf, unsigned size);

/*
 * PCI IDs used in this driver from Win Ridge forward. There is no
 * need for the PCI quirk anymore as we will use ICM also on Apple
 * hardware.
 */
#define PCI_DEVICE_ID_INTEL_MAPLE_RIDGE_4C_NHI		0x1137
#define PCI_DEVICE_ID_INTEL_WIN_RIDGE_2C_NHI            0x157d
#define PCI_DEVICE_ID_INTEL_WIN_RIDGE_2C_BRIDGE         0x157e
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_LP_NHI		0x15bf
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_LP_BRIDGE	0x15c0
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_4C_NHI	0x15d2
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_4C_BRIDGE	0x15d3
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_2C_NHI	0x15d9
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_2C_BRIDGE	0x15da
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_LP_USBONLY_NHI	0x15dc
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_USBONLY_NHI	0x15dd
#define PCI_DEVICE_ID_INTEL_ALPINE_RIDGE_C_USBONLY_NHI	0x15de
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_2C_BRIDGE	0x15e7
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_2C_NHI		0x15e8
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_4C_BRIDGE	0x15ea
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_4C_NHI		0x15eb
#define PCI_DEVICE_ID_INTEL_TITAN_RIDGE_DD_BRIDGE	0x15ef
#define PCI_DEVICE_ID_INTEL_ICL_NHI1			0x8a0d
#define PCI_DEVICE_ID_INTEL_ICL_NHI0			0x8a17
#define PCI_DEVICE_ID_INTEL_TGL_NHI0			0x9a1b
#define PCI_DEVICE_ID_INTEL_TGL_NHI1			0x9a1d
#define PCI_DEVICE_ID_INTEL_TGL_H_NHI0			0x9a1f
#define PCI_DEVICE_ID_INTEL_TGL_H_NHI1			0x9a21

#define PCI_CLASS_SERIAL_USB_USB4			0x0c0340

#endif
