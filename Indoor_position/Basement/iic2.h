#ifndef _IIC2_H_
#define _IIC2_H_

#include "mcu.h"
#include "delay.h"

void IIC2_Init(void);

void IIC2_SDAOut(void);

void IIC2_SDAIn(void);

void IIC2_Start(void);

void IIC2_Stop(void);

uint8_t IIC2_Waitack(void);

void IIC2_Ack(void);

void IIC2_Nack(void);

void IIC2_Sendbyte(uint8_t IIC_Text);

uint8_t IIC2_Readbyte(uint8_t IIC_ack);

#endif
