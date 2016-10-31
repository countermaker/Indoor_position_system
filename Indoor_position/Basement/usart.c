#include "usart.h"
#include "global.h"
/**
  * @brief  printf
  * @param  None
  * @retval None
  */
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);
	USART1->DR = (u8) ch;      
	return ch;
}
/**
  * @brief  Init the usart for debuging and for bluetooth
  * @param  None
  * @retval None
  */
void Uart1_Init(u32 baud)
{
	/*Configure paraments */
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	/*Open clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
  /*GPIO config*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA,&GPIO_InitStructure); 
	
	
  /*USART config*/
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	
  USART_Init(USART1, &USART_InitStructure); 
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);  
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;	
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;				
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;								
  NVIC_Init(&NVIC_InitStructure);  	  
	
	/*Clear flag*/
	USART_ClearFlag(USART1, USART_FLAG_TC);
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	USART_Cmd(USART1, ENABLE);
}
/**
  * @brief  Get Data from Uart1
  * @param  None
  * @retval The recent data 
  */
void Uart1_GetData(void)
{
	uint8_t c;
  static uint32_t buf_cnt = 0;
  static uint8_t capture_flag = 0;

  if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET) 
	{
    c = USART_ReceiveData(USART1) & 0xFF;
    if (c == '$') 
		{
      capture_flag = 1;
      buf_cnt = 0;
      g_packet_buf[buf_cnt] = c;
      buf_cnt++;
    }
    else if (capture_flag == 1) 
		{
      g_packet_buf[buf_cnt] = c;
      buf_cnt++;
      if (COMM_PACKET_BUF_LEN == buf_cnt) 
			{
        if (c == '*')
          Flag_UartRx = 1;
        else
          capture_flag = 0;
      }
    }
  }
}
