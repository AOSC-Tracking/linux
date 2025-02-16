#include <perf/arm_pmuv3.h>

#include "vgic.h"
#include "test_util.h"
#include "processor.h"

static bool undef_taken;

#define test_read(sr)							\
do {									\
	u64 __val = read_sysreg(sr);					\
									\
	if (READ_ONCE(undef_taken))					\
		GUEST_PRINTF("read_sysreg("#sr"): UNDEFINED\n");	\
	else								\
		GUEST_PRINTF("read_sysreg("#sr"): %lx\n", __val);	\
	WRITE_ONCE(undef_taken, false);					\
} while (0)

#define test_write(val, sr)							\
do {										\
	write_sysreg(val, sr);							\
										\
	if (READ_ONCE(undef_taken))						\
		GUEST_PRINTF("write_sysreg(%x, "#sr"): UNDEFINED\n", val);	\
	else									\
		GUEST_PRINTF("write_sysreg(%x, "#sr"): OK\n", val);		\
	WRITE_ONCE(undef_taken, false);						\
} while (0)

static void guest_undef_handler(struct ex_regs *regs)
{
	WRITE_ONCE(undef_taken, true);
	regs->pc += 4;
}

#define READ_PMEVCNTRN(n)	test_read(pmevcntr##n##_el0)
static void test_read_evcntr(int n)
{
	PMEVN_SWITCH(n, READ_PMEVCNTRN);
}

#define READ_PMEVTYPERN(n)	test_read(pmevtyper##n##_el0);
static void test_read_evtyper(int n)
{
	PMEVN_SWITCH(n, READ_PMEVTYPERN);
}

static void guest_code(void)
{
	test_read(pmcr_el0);
	test_read(pmcntenset_el0);
	test_read(pmcntenclr_el0);
	test_read(pmovsset_el0);
	test_read(pmovsclr_el0);
	test_read(pmintenset_el1);
	test_read(pmintenclr_el1);
	test_read(pmceid0_el0);
	test_read(pmceid1_el0);

	test_read(pmccntr_el0);
	test_read(pmccfiltr_el0);
	test_write(0, pmswinc_el0);

	test_write(0, pmselr_el0);
	test_read(pmxevcntr_el0);
	test_read(pmxevtyper_el0);

	test_read(pmuserenr_el0);

	for (int i = 0; i < 31; i++) {
		test_read_evcntr(i);
		test_read_evtyper(i);
	}

	GUEST_DONE();
}

static void run_test(struct kvm_vcpu *vcpu)
{
	struct ucall uc;

	while (true) {
		vcpu_run(vcpu);

		switch (get_ucall(vcpu, &uc)) {
		case UCALL_PRINTF:
			REPORT_GUEST_PRINTF(uc);
			break;
		case UCALL_DONE:
			return;
		default:
			TEST_FAIL("Unknown ucall %lu", uc.cmd);
		}
	}
}

int main(void)
{
	struct kvm_device_attr attr;
	struct kvm_vcpu_init init;
	struct kvm_vcpu *vcpu;
	struct kvm_vm *vm;
	int irq = 23;

	TEST_REQUIRE(kvm_has_cap(KVM_CAP_ARM_PMU_V3));

	vm = vm_create(1);
	vm_ioctl(vm, KVM_ARM_PREFERRED_TARGET, &init);
	init.features[0] |= (1 << KVM_ARM_VCPU_PMU_V3);
	vcpu = aarch64_vcpu_add(vm, 0, &init, guest_code);

	__TEST_REQUIRE(vgic_v3_setup(vm, 1, 64) >= 0,
		       "Failed to create vgic-v3, skipping");

	vm_init_descriptor_tables(vm);
	vcpu_init_descriptor_tables(vcpu);
	vm_install_sync_handler(vm, VECTOR_SYNC_CURRENT, ESR_ELx_EC_UNKNOWN,
				guest_undef_handler);

	attr = (struct kvm_device_attr) {
		.group	= KVM_ARM_VCPU_PMU_V3_CTRL,
		.attr	= KVM_ARM_VCPU_PMU_V3_IRQ,
		.addr	= (u64)&irq,
	};
	vcpu_ioctl(vcpu, KVM_SET_DEVICE_ATTR, &attr);

	attr = (struct kvm_device_attr) {
		.group	= KVM_ARM_VCPU_PMU_V3_CTRL,
		.attr	= KVM_ARM_VCPU_PMU_V3_INIT,
	};
	vcpu_ioctl(vcpu, KVM_SET_DEVICE_ATTR, &attr);

	run_test(vcpu);
}
