/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2025 Loongson Technology Corporation Limited
 */

#ifndef __ASM_KVM_DMSINTC_H
#define __ASM_KVM_DMSINTC_H


struct loongarch_dmsintc {
	struct kvm *kvm;
	uint64_t msg_addr_base;
	uint64_t msg_addr_size;
	uint32_t cpu_mask;
};

struct dmsintc_state {
	atomic64_t  vector_map[4];
};

void dmsintc_inject_irq(struct kvm_vcpu *vcpu);
int dmsintc_deliver_msi_to_vcpu(struct kvm *kvm, struct kvm_vcpu *vcpu,
				u32 vector, int level);
int dmsintc_set_msi_irq(struct kvm *kvm, u64 addr, int data, int level);
int kvm_loongarch_register_dmsintc_device(void);
#endif
