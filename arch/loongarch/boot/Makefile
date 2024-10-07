#
# arch/loongarch/boot/Makefile
#
# Copyright (C) 2020-2022 Loongson Technology Corporation Limited
#

drop-sections := .comment .note .options .note.gnu.build-id
strip-flags   := $(addprefix --remove-section=,$(drop-sections)) -S
OBJCOPYFLAGS_vmlinux.efi := -O binary $(strip-flags)

quiet_cmd_strip = STRIP	  $@
      cmd_strip = $(STRIP) -s -o $@ $<

targets := vmlinux.elf
$(obj)/vmlinux.elf: vmlinux FORCE
	$(call if_changed,strip)

targets += vmlinux.efi
$(obj)/vmlinux.efi: vmlinux FORCE
	$(call if_changed,objcopy)

EFI_ZBOOT_PAYLOAD      := vmlinux.efi
EFI_ZBOOT_BFD_TARGET   := elf64-loongarch
EFI_ZBOOT_MACH_TYPE    := LOONGARCH64

suffix-y			:= bin
suffix-$(CONFIG_KERNEL_BZIP2)	:= bz2
suffix-$(CONFIG_KERNEL_GZIP)	:= gz
suffix-$(CONFIG_KERNEL_LZMA)	:= lzma
suffix-$(CONFIG_KERNEL_LZO)	:= lzo

targets += uImage
targets += uImage.bin
targets += uImage.bz2
targets += uImage.gz
targets += uImage.lzma
targets += uImage.lzo

#
# Compressed vmlinux images
#

extra-y += vmlinux.bin.bz2
extra-y += vmlinux.bin.gz
extra-y += vmlinux.bin.lzma
extra-y += vmlinux.bin.lzo

$(obj)/vmlinux.bin.bz2: $(obj)/vmlinux.bin FORCE
	$(call if_changed,bzip2)

$(obj)/vmlinux.bin.gz: $(obj)/vmlinux.bin FORCE
	$(call if_changed,gzip)

$(obj)/vmlinux.bin.lzma: $(obj)/vmlinux.bin FORCE
	$(call if_changed,lzma)

$(obj)/vmlinux.bin.lzo: $(obj)/vmlinux.bin FORCE
	$(call if_changed,lzo)


UIMAGE_LOADADDR  = $(VMLINUX_LOAD_ADDRESS)
UIMAGE_ENTRYADDR = $(VMLINUX_ENTRY_ADDRESS)
#
# Compressed u-boot images
#

$(obj)/uImage.bin: $(obj)/vmlinux.bin FORCE
	$(call if_changed,uimage,none)

$(obj)/uImage.bz2: $(obj)/vmlinux.bin.bz2 FORCE
	$(call if_changed,uimage,bzip2)

$(obj)/uImage.gz: $(obj)/vmlinux.bin.gz FORCE
	$(call if_changed,uimage,gzip)

$(obj)/uImage.lzma: $(obj)/vmlinux.bin.lzma FORCE
	$(call if_changed,uimage,lzma)

$(obj)/uImage.lzo: $(obj)/vmlinux.bin.lzo FORCE
	$(call if_changed,uimage,lzo)

$(obj)/uImage: $(obj)/uImage.$(suffix-y)
	@ln -sf $(notdir $<) $@
	@echo '  Image $@ is ready'

include $(srctree)/drivers/firmware/efi/libstub/Makefile.zboot
