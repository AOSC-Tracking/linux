#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/serial_reg.h>
#include <linux/serial.h>
#include <asm/addrspace.h>
#include <asm/io.h>

#define LOONGUART1 0x1fe001e0

static void loong_uart_putc(unsigned char c){
	*(volatile unsigned char *)TO_UNCACHE(LOONGUART1 + UART_TX) = c;
	int status;
	for(int count = 0; count < 25000000; count++){
		status = *(volatile unsigned char *)TO_UNCACHE(LOONGUART1 + UART_LSR);
		if (uart_lsr_tx_empty(status))
			break;
		cpu_relax();
	}
}

static void early_serial_write(struct console *con, const char *s, unsigned n){
	while(*s && n-- > 0){
		if ( *s == '\n' ){
			loong_uart_putc('\r');
		}
		loong_uart_putc(*s);
		s++;
	}
}

static struct console early_serial_console = {
	.name =		"earlyser",
	.write =	early_serial_write,
	.flags =	CON_PRINTBUFFER | CON_BOOT,
	.index =	-1,
};

static int __init setup_early_printk(char *buf)
{
	static int registered = 0;
	if (registered) {
		return 0;
	}
	if (early_serial_console.index != -1) {
		return 0;
	}
	register_console(&early_serial_console);
	return 0;
}

early_param("earlyprintk", setup_early_printk);
