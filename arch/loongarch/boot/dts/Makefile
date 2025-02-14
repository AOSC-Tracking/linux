# SPDX-License-Identifier: GPL-2.0-only

# dtb-y = loongson-2k0500-ref.dtb loongson-2k1000-ref.dtb loongson-2k2000-ref.dtb

ifdef CONFIG_DTB_MATCH_BY_BOARD_NAME
dtb-$(CONFIG_LOONGSON_2K300) += ls2k300_mini_dp.dtb ls2k300_vanguard_pi.dtb
obj-y += $(patsubst %.dtb, %.dtb.o, $(dtb-y))
else
obj-$(CONFIG_BUILTIN_DTB)	+= $(addsuffix .dtb.o, $(CONFIG_BUILTIN_DTB_NAME))
endif
