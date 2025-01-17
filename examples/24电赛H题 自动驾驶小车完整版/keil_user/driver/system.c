#include "ti_msp_dl_config.h"
#include "system.h"


volatile uint32_t sysTickUptime = 0;
void SysTick_Handler(void)
{
  sysTickUptime++;
}

//返回 us
uint32_t micros(void)
{
  uint32_t systick_period = CPUCLK_FREQ / 1000U;
  return sysTickUptime * 1000 + (1000 * (systick_period - SysTick->VAL)) / systick_period;
}

//返回 ms
uint32_t millis(void) {
	return micros() / 1000UL;
}

//延时us
void delayMicroseconds(uint32_t us) {
  uint32_t start = micros();
  while((int32_t)(micros() - start) < us) {
    // Do nothing
  };
}
void delay(uint32_t ms) {
  delayMicroseconds(ms * 1000UL);
}
void delay_ms(uint32_t x)
{
  delay(x);
}

void delay_us(uint32_t x)
{
  delayMicroseconds(x);
}

void Delay_Ms(uint32_t time)  //延时函数  
{   
	delay_ms(time);
}  











