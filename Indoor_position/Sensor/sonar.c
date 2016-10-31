#include "sonar.h"
extern u8 Sonar_Flag;

/**
  * @brief  Get the pulse width
  * @param  float* sonar width
  * @retval None
  */
void Sonar_Getwidth(float*sonar_width,Chx_Width*p)
{
	static int32_t cnt,cnt_1,cnt_2;
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line=EXTI_Line1 ;	
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;	
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;

	Timer_Control(0);
	cnt=TIM_GetCounter(TIM1);
	Timer_Control(1);
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1))
	{
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
		EXTI_Init(&EXTI_InitStructure);	
		cnt_1=cnt;
		p1=cnt_1;
	}
	else
	{
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
		EXTI_Init(&EXTI_InitStructure);	
		cnt_2=cnt;
		p2=cnt_2;
		if(cnt_2>=cnt_1)
		{
			*sonar_width=cnt_2-cnt_1;
		}
		else
		{
			*sonar_width=0xffff+cnt_2-cnt_1+1;
		}
	}
}
/**
  * @brief  Init the sonar GPIO
  * @param  None
  * @retval None
  */
void Sonar_GpioInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	

	GPIO_Init(GPIOA,&GPIO_InitStructure); 
}
/**
  * @brief  Delay 10us
  * @param  None
  * @retval None
  */
void Sonar_Delay10us(void)
{
	int16_t cnt;
	for(cnt=0;cnt<90;cnt++)
	{
		__nop();
	}
}
/**
  * @brief  give a pluse to sonar
  * @param  None
  * @retval None
  */
void Sonar_Encourage(void)
{
	GPIO_SetBits(GPIOA,GPIO_Pin_11);
	Sonar_Delay10us();
	GPIO_ResetBits(GPIOA,GPIO_Pin_11);
}
/**
  * @brief  Get the distance
  * @param  None
  * @retval None
  */
void Sonar_GetDist(float*sonar_width,float*distance)
{
	*distance = 340.0f*((*sonar_width)/1000000.0f)/2.0f; /* unit: m */
}
