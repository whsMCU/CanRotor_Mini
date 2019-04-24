//#include "System.h"
#include "Board.h"
static volatile uint32_t msTicks = 0;


void HAL_SYSTICK_Callback(void)
{
	msTicks++;
}

uint32_t micros(void)
{
	 register uint32_t ms, cycle_cnt;
	    do {
	        ms = msTicks;
	        cycle_cnt = SysTick->VAL;
	    } while (ms != msTicks);
	    return (ms * 1000) + (72 * 1000 - cycle_cnt) / 72; //168
}

uint32_t millis(void)
{
  return HAL_GetTick();
}
