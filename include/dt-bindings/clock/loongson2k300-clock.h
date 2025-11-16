#ifndef __DT_BINDINGS_CLOCK_LOONGSON2K300_H
#define __DT_BINDINGS_CLOCK_LOONGSON2K300_H

#define CLK_REF     0

#define CLK_NODE    1
#define CLK_CPU     2
#define CLK_SCACHE  3
#define CLK_IODMA   4
#define CLK_GMAC    5
#define CLK_I2S     6

#define CLK_DDR_P   7
#define CLK_DDR     8
#define CLK_NET     9
#define CLK_DEVS    10
#define CLK_USB     11
#define CLK_APB     12
#define CLK_BOOT    13
#define CLK_SDIO    14

#define CLK_PIX_P   15
#define CLK_PIX     16
#define CLK_GMACBP  17

#define CLK_UART    CLK_APB
#define CLK_CAN     CLK_APB
#define CLK_I2C     CLK_APB
#define CLK_SPI     CLK_APB
#define CLK_AC97    CLK_APB
#define CLK_NAND    CLK_APB
#define CLK_PWM     CLK_APB
#define CLK_HPET    CLK_APB
#define CLK_RTC     CLK_APB

#define CLK_CNT    18

#endif