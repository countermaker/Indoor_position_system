#ifndef SONAR2_H_
#define SONAR2_H_
#include "mcu.h"
#include "delay.h"
#include "receive.h"
#include "timer.h"
#include "monitor.h"
#include "bsp.h"
#include "global.h"

void Sonar2_GPIOInit(void);

void Sonar2_Getwidth(float*sonar_width,Chx_Width*p);

void Sonar2_Delay10us(void);

void Sonar2_Encourage(void);

void Sonar2_GetDist(float*sonar_width,float*distance);
#endif
