#ifndef _TIMER_H
#define _TIMER_H


#include "mcu.h"
#include "data.h"
#include "monitor.h"
#include "LED.h"

void Pwm_init(void);/*Init the Moto*/

void Pwm_Set1(uint16_t cnt);

void Pwm_Set2(uint16_t cnt);

void Pwm_Set3(uint16_t cnt);

void Pwm_Set4(uint16_t cnt);

#endif
