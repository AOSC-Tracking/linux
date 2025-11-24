CONFIG_AIC8800_BTLPM_SUPPORT = m

obj-$(CONFIG_AIC8800_BTLPM_SUPPORT) := aic8800_btlpm.o

ccflags-y += -I$(srctree)/$(src)/../aic8800_bsp

CONFIG_SUPPORT_LPM ?= n
CONFIG_AUTO_PM ?= n

aic8800_btlpm-y := \
	aic_bluetooth_main.o \
	rfkill.o \

aic8800_btlpm-$(CONFIG_SUPPORT_LPM) += lpm.o

ccflags-y += -DAIC_TRACE_INCLUDE_PATH=$(src)

ccflags-$(CONFIG_AUTO_PM) += -DCONFIG_AUTO_PM
