#include "led.h"

#define GPIO_Remap_SWJ_JTAGDisable ((uint32_t)0x00300200)/*!<JTAG-DP Disabled and SW-PW Enabled>*/

/**
  * @brief  Init led
  * @param  None
  * @retval None
  */
void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO ,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	

	GPIO_Init(GPIOB,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	GPIO_SetBits(GPIOB,GPIO_Pin_4);
	GPIO_SetBits(GPIOB,GPIO_Pin_3);
}
/**
  * @brief  Light the led0
  * @param  None
  * @retval None
  */
void Led_On0(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_4);
}
/**
  * @brief  Off the led0
  * @param  None
  * @retval None
  */
void Led_Off0(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_4);
}
/**
  * @brief  Light the led1
  * @param  None
  * @retval None
  */
void Led_On1(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_3);
}
/**
  * @brief  Off the led1
  * @param  None
  * @retval None
  */
void Led_Off1(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_3);
}
/**
  * @brief  Transpose the led1
  * @param  None
  * @retval None
  */
void Led_Transpose1(void)
{
	if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_3))
	{
		Led_On1();
	}
	else
	{
		Led_Off1();
	}
}
/**
  * @brief  Transpose the led0
  * @param  None
  * @retval None
  */
void Led_Transpose0(void)
{
	if(!GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_4))
	{
		Led_Off0();
		return;
	}
	else
	{
		Led_On0();
		return;
	}
}
