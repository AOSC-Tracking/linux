# LS CAN FD ip core Linux driver Makefile

# Invoked make without parameters for native build for current running kernel
# Set KERNEL_VERSION variable to compile for specific installed kernel version
# Exact location of Linux kernel build tree can be chosen by KDIR variable
# If you build for different CPU architecture than native and you do not
# use GNUmakefile trick to set CROSS_COMPILE and ARCH then set these to
# matching values else you do not screw only LS CAN FD build but even
# base kernel build

quiet_cmd_build_lscanfd_dma = GEN	$@
      cmd_build_lscanfd_dma = cp $(obj)/lscanfd_dma.o_shipped $@

quiet_cmd_build_lscanfd_platform = GEN	$@
      cmd_build_lscanfd_platform = cp $(obj)/lscanfd_platform.o_shipped $@

$(obj)/lscanfd_dma.o: FORCE
	$(call if_changed,build_lscanfd_dma)

$(obj)/lscanfd_platform.o: FORCE
	$(call if_changed,build_lscanfd_platform)

obj-$(CONFIG_CAN_LSCANFD) := lscanfd_dma.o
obj-$(CONFIG_CAN_LSCANFD_PLATFORM) += lscanfd_platform.o

