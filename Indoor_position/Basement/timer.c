#include "timer.h"
/**
  * @brief  initialise Timer
  * @param  period_num
  * @retval None
  */
void Timer_Init(u16 period_num)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);	

	TIM_TimeBaseStructure.TIM_Period=period_num;
	TIM_TimeBaseStructure.TIM_Prescaler=72-1;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM1,TIM_FLAG_Update);
}
/**
  * @brief  Control Timer
  * @param  sta £º1 ON
	*	 							0 OFF
  * @retval None
  */
void Timer_Control(u8 sta)
{
	if(sta==0)
		TIM_Cmd(TIM1,DISABLE);
	if(sta==1)
		TIM_Cmd(TIM1,ENABLE);
}
