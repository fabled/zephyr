/**
 * @file Board config file
 */

#include <device.h>
#include <init.h>

#include <kernel.h>

#include "soc.h"

static int soc_init(struct device *dev)
{
	__enable_irq();					/* Enable IRQs as ROM loader does PRIMASK=1 */
	PCR->CLK_REQ_1 |= 1 << 0;			/* Clock: Interrupt block */
	PCR->CLK_REQ_1 |= 1 << 8;			/* Clock: Processor */
#ifdef CONFIG_UART_NS16550_PORT_0
	PCR->CLK_REQ_2 |= 1 << 1;			/* Clock: UART0 */
	GPIO_100_137->GPIO_104_PIN_CONTROL = 1 << 12;	/* GPIO104 Alt1 = UART0_TX */
	GPIO_100_137->GPIO_105_PIN_CONTROL = 1 << 12;	/* GPIO105 Alt1 = UART0_RX */
	UART0->CONFIG = 0;				/* Reset on RESET_SYS */
	UART0->ACTIVATE = 1;				/* Power up UART0 */
#endif
#ifdef CONFIG_UART_NS16550_PORT_1
	PCR->CLK_REQ_2 |= 1 << 2;			/* Clock: UART1 */
	GPIO_140_176->GPIO_170_PIN_CONTROL = 2 << 12;	/* GPIO170 Alt2 = UART1_TX */
	GPIO_140_176->GPIO_171_PIN_CONTROL = 2 << 12;	/* GPIO171 Alt2 = UART1_RX */
	GPIO_100_137->GPIO_113_PIN_CONTROL = 1 << 9;	/* GPIO113 drives UART1_RX_EN on secureiot1702 board */
	UART1->CONFIG = 0;				/* Reset on RESET_SYS */
	UART1->ACTIVATE = 1;				/* Power up UART1 */
#endif
	return 0;
}

SYS_INIT(soc_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
