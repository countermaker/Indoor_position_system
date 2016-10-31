#ifndef _IIC_H_
#define _IIC_H_

#include "mcu.h"
#include "delay.h"

void IIC_Init(void);

void IIC_SDAOut(void);

void IIC_SDAIn(void);

void IIC_Start(void);

void IIC_Stop(void);

uint8_t IIC_Waitack(void);

void IIC_Ack(void);

void IIC_Nack(void);

void IIC_Sendbyte(uint8_t IIC_Text);

uint8_t IIC_Readbyte(uint8_t IIC_ack);

#endif
