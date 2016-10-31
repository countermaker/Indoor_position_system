#include "iic.h"

#define IIC_SCL_1 GPIO_SetBits(GPIOB,GPIO_Pin_10)
#define IIC_SDA_1 GPIO_SetBits(GPIOB,GPIO_Pin_11)
#define IIC_SCL_0 GPIO_ResetBits(GPIOB,GPIO_Pin_10)
#define IIC_SDA_0 GPIO_ResetBits(GPIOB,GPIO_Pin_11)
#define IIC_SDA_OUT {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}
#define IIC_SDA_IN  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}
#define IIC_READ_SDA GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11)
/**
  * @brief  initialise the IIC GPIO
  * @param  None
  * @retval None
  */
void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	

	GPIO_Init(GPIOB,&GPIO_InitStructure); 

	IIC_SCL_1;
	IIC_SDA_1;
}
/**
  * @brief  SDA output mode
  * @param  None
  * @retval None
  */
void IIC_SDAOut(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
}
/**
  * @brief  SDA input mode
  * @param  None
  * @retval None
  */
void IIC_SDAIn(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//up?
	GPIO_Init(GPIOB,&GPIO_InitStructure); 
}
/**
  * @brief  IIC Start
  * @param  None
  * @retval None
  */
void IIC_Start(void)
{
	IIC_SDA_OUT;
	IIC_SDA_1;	
	IIC_SCL_1;	
	Delay_us(2);
	IIC_SDA_0;
	Delay_us(2);
	IIC_SCL_0;
}
/**
  * @brief  IIC Stop
  * @param  None
  * @retval None
  */
void IIC_Stop(void)
{
	IIC_SDA_OUT;
	IIC_SCL_0;
	IIC_SDA_0;
	Delay_us(2);
	IIC_SCL_1;	
	IIC_SDA_1;
	Delay_us(2);
}
/**
  * @brief  IIC Waitack
  * @param  None
  * @retval None
  */
uint8_t IIC_Waitack(void)
{
	uint8_t IIC_Waitlimit=0;
	IIC_SDA_IN;
	IIC_SDA_1;	
	Delay_us(1);
	IIC_SCL_1;	
	Delay_us(1);
	while(IIC_READ_SDA)
	{
		IIC_Waitlimit++;
		if(IIC_Waitlimit>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_0;
	return 0;
}
/**
  * @brief  IIC inspire ack
  * @param  None
  * @retval None
  */
void IIC_Ack(void)
{
	IIC_SCL_0;
	IIC_SDA_OUT;
	IIC_SDA_0;
	Delay_us(2);
	IIC_SCL_1;
	Delay_us(2);
	IIC_SCL_0;
}
/**
  * @brief  IIC inspire no ack
  * @param  None
  * @retval None
  */
void IIC_Nack(void)
{
	IIC_SCL_0;
	IIC_SDA_OUT;
	IIC_SDA_1;
	Delay_us(2);
	IIC_SCL_1;
	Delay_us(2);
	IIC_SCL_0;
}
/**
  * @brief  IIC Send a byte
  * @param  IIC_Text: a byte which will be sent
  * @retval None
  */
void IIC_Sendbyte(uint8_t IIC_Text)
{
	uint8_t IIC_Temp;
	IIC_SDA_OUT;
	IIC_SCL_0;
	for(IIC_Temp=0;IIC_Temp<8;IIC_Temp++)
	{
		if(IIC_Text&0x80)
			IIC_SDA_1;
		else
			IIC_SDA_0;
		IIC_Text<<=1;
		Delay_us(1);
		IIC_SCL_1;
		Delay_us(1);
		IIC_SCL_0;
		Delay_us(1);
	}
}
/**
  * @brief  IIC read a byte
  * @param  IIC_ack :1:ack
	*									 0:no ack
  * @retval IIC_Reseave
  */
uint8_t IIC_Readbyte(uint8_t IIC_ack)
{
	uint8_t IIC_Temp,IIC_Reseave=0;
	IIC_SDA_IN;
	for(IIC_Temp=0;IIC_Temp<8;IIC_Temp++)
	{
		IIC_SCL_0;
		Delay_us(1);
		IIC_SCL_1;
		IIC_Reseave<<=1;
		if(IIC_READ_SDA)
			IIC_Reseave++;
		Delay_us(1);
	}
	if(!IIC_ack)
		IIC_Nack();
	else
		IIC_Ack();
	return IIC_Reseave;
}
