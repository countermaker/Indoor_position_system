#ifndef _RECEIVE_H_
#define _RECEIVE_H_

#include "mcu.h"
typedef struct
{
	int16_t Ch1;
	int16_t Ch2;
	int16_t Ch3;
	int16_t Ch4;
	int16_t Ch5;
	int16_t Ch6;
	int16_t Ch7;
	int16_t Ch8;
	
}Chx_Width;

void TimerCapture_Init(void);

void Reseave_IrqTIM2(Chx_Width*p);

void Reseave_IrqTIM3(Chx_Width*p);

#endif
