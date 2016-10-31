#ifndef _SONAR_H_
#define _SONAR_H_

#include "mcu.h"
#include "delay.h"
#include "receive.h"
#include "timer.h"
#include "monitor.h"
#include "bsp.h"
#include "global.h"

void Sonar_Getwidth(float*sonar_width,Chx_Width*p);

void Sonar_GpioInit(void);

void Sonar_Encourage(void);

void Sonar_GetDist(float*sonar_width,float*distance);

#endif
