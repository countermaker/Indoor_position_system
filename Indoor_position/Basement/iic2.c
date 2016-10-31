#include "iic2.h"

#define IIC2_SCL_1 GPIO_SetBits(GPIOA,GPIO_Pin_6)
#define IIC2_SCL_0 GPIO_ResetBits(GPIOA,GPIO_Pin_6)
#define IIC2_SDA_1 GPIO_SetBits(GPIOA,GPIO_Pin_7)
#define IIC2_SDA_0 GPIO_ResetBits(GPIOA,GPIO_Pin_7)
#define IIC2_SDA_OUT IIC2_SDAOut()//{GPIOA->CRL&=0X0FFFFFFF;GPIOA->CRL|=0X30000000;}
#define IIC2_SDA_IN  IIC2_SDAIn()//{GPIOA->CRL&=0X0FFFFFFF;GPIOA->CRL|=0X80000000;}
#define IIC2_READ_SDA GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_7)

/**
  * @brief  initialise the IIC GPIO
  * @param  None
  * @retval None
  */
void IIC2_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	

	GPIO_Init(GPIOA,&GPIO_InitStructure); 

	IIC2_SCL_1;
	IIC2_SDA_1;
}
/**
  * @brief  SDA output mode
  * @param  None
  * @retval None
  */
void IIC2_SDAOut(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
}
/**
  * @brief  SDA input mode
  * @param  None
  * @retval None
  */
void IIC2_SDAIn(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//up?
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
}

/**
  * @brief  IIC Start
  * @param  None
  * @retval None
  */
void IIC2_Start(void)
{
	IIC2_SDA_OUT;
	/*SDA is first*/
	IIC2_SDA_1;	
	IIC2_SCL_1;	
	Delay_us(5);
	IIC2_SDA_0;
	Delay_us(5);
	IIC2_SCL_0;
}
/**
  * @brief  IIC Stop
  * @param  None
  * @retval None
  */
void IIC2_Stop(void)
{
	IIC2_SDA_OUT;
	IIC2_SCL_0;
	IIC2_SDA_0;
	Delay_us(5);
	IIC2_SCL_1;	
	IIC2_SDA_1;
	Delay_us(5);
}
/**
  * @brief  IIC Waitack
  * @param  None
  * @retval None
  */
uint8_t IIC2_Waitack(void)
{
	uint8_t IIC2_Waitlimit=0;
	IIC2_SDA_IN;
	IIC2_SDA_1;	
	Delay_us(5);
	IIC2_SCL_1;	
	Delay_us(5);
	while(IIC2_READ_SDA)
	{
		IIC2_Waitlimit++;
		if(IIC2_Waitlimit>250)
		{
			IIC2_Stop();
			return 1;
		}
	}
	IIC2_SCL_0;
	return 0;
}
/**
  * @brief  IIC inspire ack
  * @param  None
  * @retval None
  */
void IIC2_Ack(void)
{
	IIC2_SCL_0;
	IIC2_SDA_OUT;
	IIC2_SDA_0;
	Delay_us(5);
	IIC2_SCL_1;
	Delay_us(5);
	IIC2_SCL_0;
}
/**
  * @brief  IIC inspire no ack
  * @param  None
  * @retval None
  */
void IIC2_Nack(void)
{
	IIC2_SCL_0;
	IIC2_SDA_OUT;
	IIC2_SDA_1;
	Delay_us(5);
	IIC2_SCL_1;
	Delay_us(5);
	IIC2_SCL_0;
}
/**
  * @brief  IIC Send a byte
  * @param  IIC_Text: a byte which will be sent
  * @retval None
  */
void IIC2_Sendbyte(uint8_t IIC_Text)
{
	uint8_t IIC2_Temp;
	IIC2_SDA_OUT;
	IIC2_SCL_0;
	for(IIC2_Temp=0;IIC2_Temp<8;IIC2_Temp++)
	{
		if(IIC_Text&0x80)
			IIC2_SDA_1;
		else
			IIC2_SDA_0;
		/*From high to low*/
		IIC_Text<<=1;
		Delay_us(5);
		IIC2_SCL_1;
		Delay_us(5);
		IIC2_SCL_0;
		Delay_us(5);
	}
}
/**
  * @brief  IIC read a byte
  * @param  IIC_ack :1:ack
	*									 0:no ack
  * @retval IIC_Reseave
  */
uint8_t IIC2_Readbyte(uint8_t IIC_ack)
{
	uint8_t IIC_Temp,IIC2_Reseave=0;
	IIC2_SDA_IN;
	for(IIC_Temp=0;IIC_Temp<8;IIC_Temp++)
	{
		IIC2_SCL_0;
		Delay_us(5);
		IIC2_SCL_1;
		IIC2_Reseave<<=1;
		if(IIC2_READ_SDA)
			IIC2_Reseave++;
		Delay_us(5);
	}
	if(!IIC_ack)
		IIC2_Nack();
	else
		IIC2_Ack();
	return IIC2_Reseave;
}
