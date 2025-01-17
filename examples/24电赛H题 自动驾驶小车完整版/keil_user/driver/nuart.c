#include "ti_msp_dl_config.h"
#include "headfile.h"


void usart_irq_config(void)
{
	NVIC_ClearPendingIRQ(UART_2_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_2_INST_INT_IRQN);
	DL_UART_clearInterruptStatus(UART_2_INST,DL_UART_INTERRUPT_RX);//清除中断标志位
}

void UART_2_INST_IRQHandler(void)
{
  if(DL_UART_getEnabledInterruptStatus(UART_2_INST,DL_UART_INTERRUPT_RX) == DL_UART_INTERRUPT_RX)
  {
		uint8_t ch = DL_UART_receiveData(UART_2_INST);
		bluetooth_app_prase(ch);
		DL_UART_clearInterruptStatus(UART_2_INST,DL_UART_INTERRUPT_RX);//清除中断标志位
  }
}

int fputc(int ch, FILE *f)
{
  DL_UART_Main_transmitDataBlocking(UART_2_INST, ch);
  return ch;
}

