#include <linux/serial_reg.h>
#include <linux/serial.h>
#include <asm/addrspace.h>
#include <asm/io.h>

#define LOONGUART1 0x1fe001e0

static inline void loong_uart_putc(unsigned char c){
	*(volatile unsigned char *)TO_UNCACHE(LOONGUART1 + UART_TX) = c;
	int status;
	for(int count = 0; count < 25000000; count++){
		status = *(volatile unsigned char *)TO_UNCACHE(LOONGUART1 + UART_LSR);
		if (uart_lsr_tx_empty(status))
			break;
		cpu_relax();
	}
}

static inline void loong_uart_puts(const char *str){
	while(*str){
		loong_uart_putc(*str);
		str++;
	}
}
