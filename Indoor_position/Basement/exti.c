#include "exti.h"
/**
  * @brief  initialise Mpu Pin Interrupt
  * @param  period_num
  * @retval None
  */
void EXTI_Pinconfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	/*mpu*/
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
  
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	/*sonar*/
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/**
  * @brief  Initialise EXTI
  * @param  period_num
  * @retval None
  */
void Exti_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//AFIO
	EXTI_Pinconfig();
	
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource13);
	
  EXTI_InitStructure.EXTI_Line=EXTI_Line13 ;	//MPU-INIT
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);		

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;			
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;	
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02;				
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
  NVIC_Init(&NVIC_InitStructure);  	  
	
}
