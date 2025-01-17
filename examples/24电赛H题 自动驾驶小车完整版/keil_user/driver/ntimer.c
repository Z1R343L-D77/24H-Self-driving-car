#include "ti_msp_dl_config.h"
#include "system.h"
#include "ntimer.h"


extern void maple_duty_200hz(void);
extern void duty_200hz(void);
extern void duty_1000hz(void);
u8 f_5ms=0;

void timer_irq_config(void)
{
  //pwm //差速左右轮电机
  DL_TimerA_startCounter(PWM_0_INST);
  DL_TimerA_startCounter(PWM_1_INST);
	
  NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);
  DL_TimerG_startCounter(TIMER_1_INST);
	
  NVIC_EnableIRQ(TIMER_G12_INST_INT_IRQN);
  DL_TimerG_startCounter(TIMER_G12_INST);

}
void TIMER_1_INST_IRQHandler(void)
{
	f_5ms=1;
	duty_200hz();
}

void TIMER_G12_INST_IRQHandler(void)//地面站数据发送中断函数
{
	duty_1000hz();
}


