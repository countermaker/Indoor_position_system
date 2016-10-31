#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "pwm.h"
#include "lock.h"
#include "global.h"

#define MOTO_MAX 	7999
#define MOTO_BASE 8000
#define PWM_WITH 	8000

typedef struct
{
	int16_t MOTO1;
	int16_t MOTO2;
	int16_t MOTO3;
	int16_t MOTO4;
} Moto_Pwm;

void Moto_SetPwm(Ctrl_DataType*control_data);

void Moto_PwmRflash(Moto_Pwm* MOTO_PWM);

void Moto_SetPwm(Ctrl_DataType*control_data);

void Moto_Clear(void);

void Moto_Full(void);

#endif
