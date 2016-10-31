#include "delay.h"
/**
  * @brief  Delay severl time.
  * @param  None
  * @param  None
  * @retval None
  */
void Delay_ms(uint32_t nCount)
{
	int32_t Delay_i;
	uint32_t i;
	
  for(i = 0;i<nCount;i++){
		Delay_i= 8000;
		while(Delay_i--);
	}
}
/**
  * @brief  Delay severl time.
  * @param  nCount
  * @retval None
  */
void Delay_us(uint32_t nCount)   
{
	uint32_t Delay_i=nCount*1;//5
	for(;Delay_i>0;Delay_i--)
	for(;nCount>0;nCount--)
	__nop();
}
