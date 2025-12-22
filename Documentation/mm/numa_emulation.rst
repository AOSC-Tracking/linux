.. SPDX-License-Identifier: GPL-2.0

==============
NUMA emulation
==============

If CONFIG_NUMA_EMU is enabled, you can create fake NUMA nodes with
``numa=fake=`` kernel cmdline option.
See Documentation/admin-guide/kernel-parameters.txt and
Documentation/arch/x86/x86_64/fake-numa-for-cpusets.rst for more information.


Multiple Memory Tiers Creation
==============================

The "numa_emulation.adistance=" kernel cmdline option allows you to set
the abstract distance for each NUMA node.

For example, you can create two fake nodes, each in a different memory
tier by booting with "numa=fake=2 numa_emulation.adistance=576,704".
Here, the abstract distances of node0 and node1 are set to 576 and 706,
respectively.

Each memory tier covers an abstract distance chunk size of 128. Thus,
nodes with abstract distances between 512 and 639 are classified into the
same memory tier, and nodes with abstract distances between 640 and 767
are classified into the next slower memory tier.

The abstract distance of fake nodes not specified in the parameter will
be the default DRAM abstract distance of 576.
