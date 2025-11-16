#ifndef __DT_BINDINGS_CLOCK_LOONGSON2K500_H
#define __DT_BINDINGS_CLOCK_LOONGSON2K500_H

#define CLK_REF     0

#define CLK_NODE    1
#define CLK_CPU     2
#define CLK_SCACHE  3
#define CLK_IODMA   4

#define CLK_DDR_P   5
#define CLK_DDR     6
#define CLK_HDA     7
#define CLK_NET     8
#define CLK_PRINT   CLK_NET
#define CLK_PRINT_LSU   9

#define CLK_PIX0    10
#define CLK_DC0     CLK_PIX0

#define CLK_PIX1    11
#define CLK_DC1     CLK_PIX1

#define CLK_SOC     12
#define CLK_GPU     13
#define CLK_GMAC    14
#define CLK_SB      15
#define CLK_BOOT    16
#define CLK_BOOT_CONF    CLK_BOOT
#define CLK_BOOT_SPI     CLK_BOOT
#define CLK_BOOT_LPC     CLK_BOOT
#define CLK_BOOT_LIO     CLK_BOOT
#define CLK_SATA    17
#define CLK_USB     18
#define CLK_APB     19
#define CLK_UART    CLK_APB
#define CLK_CAN     CLK_APB
#define CLK_I2C     CLK_APB
#define CLK_PS2     CLK_APB
#define CLK_SPI     CLK_APB
#define CLK_AC97    CLK_APB
#define CLK_NAND    CLK_APB
#define CLK_PWM     CLK_APB
#define CLK_DPM     CLK_APB
#define CLK_SDIO    CLK_APB
#define CLK_HPET    CLK_APB
#define CLK_ACPI    CLK_APB
#define CLK_RTC     CLK_APB

#define CLK_CNT    20

#endif