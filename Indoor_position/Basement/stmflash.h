#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "stm32f10x.h"  

#define STM32_FLASH_SIZE 512 	 		
#define STM32_FLASH_WREN 1            

#define STM32_FLASH_BASE 0x08000000 	
 
u16 FLASH_ReadHalfWord(u32 faddr);		  
void FLASH_WriteLenByte(u32 WriteAddr,u32 DataToWrite,u16 Len);	
u32 FLASH_ReadLenByte(u32 ReadAddr,u16 Len);						
void FLASH_Write(u32 WriteAddr,int16_t *pBuffer,u16 NumToWrite);		
void FLASH_Read(u32 ReadAddr,int16_t *pBuffer,u16 NumToRead);   		
void Test_Write(u32 WriteAddr,u16 WriteData);								   
#endif

















