#ifndef __NUART_H
#define __NUART_H



void UART_SendBytes(UART_Regs *port,uint8_t *ubuf, uint32_t len);
void UART_SendByte(UART_Regs *port,uint8_t data);

void usart_irq_config(void);

#endif






