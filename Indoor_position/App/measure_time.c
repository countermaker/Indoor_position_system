#include "measure_time.h"
/*the init of time measure*/
void Measure_TimeInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
}
/*start the measurement */
void Measure_TimeStart(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_8);
}
/*stop the measurement */
void Measure_TimeStop(void)
{
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
}
